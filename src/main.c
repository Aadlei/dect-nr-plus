/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/conn_mgr_connectivity.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/hostname.h>
#include <zephyr/posix/netdb.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/sys/socket.h>

#if defined(CONFIG_DK_LIBRARY)
#include <dk_buttons_and_leds.h>
#endif

#if defined(CONFIG_NRF_MODEM_LIB)
#include <modem/nrf_modem_lib.h>
#endif

#include <nrf_modem.h>

#include <net/dect/dect_net_l2_mgmt.h>
#include <net/dect/dect_net_l2.h>
#include <net/dect/dect_utils.h>

#include "spi.h"
#include "uart.h"

LOG_MODULE_REGISTER(main, CONFIG_HELLO_DECT_MAC_LOG_LEVEL);

// CHANGE THIS BASED ON DEVICE TYPE
#if defined(CONFIG_DECT_RELAY_FT)
const static dect_device_type_t current_device_type = DECT_DEVICE_TYPE_FT;
#elif defined(CONFIG_DECT_RELAY_PT)
const static dect_device_type_t current_device_type = DECT_DEVICE_TYPE_PT;
#else
const static dect_device_type_t current_device_type = DECT_DEVICE_TYPE_PT;
#endif

#define DECT_EDGE_PT_LONG_RD_ID  		0xAABBCCDDU // PT edge
#define DECT_FT_LONG_RD_ID 				0x12345678U // Change this for each FT
#define DECT_SINK_LONG_RD_ID 			0x67214200U
#define DECT_PT_LONG_RD_ID				0x11223344U // Change this for each PT

#define SYNC_MAGIC_SIGNATURE			0xFEFDU	// The G.O.A.T
#define SYNC_T_SIZE						4
#define COMMON_PORT 					12345
#define SYNC_PORT						12346
#define MESH_PREFIX_STR 				"fd12:3456:789a"
#define NW_SCAN_RETRY_MS 				2000
#define SOCKET_SYNC_RX_TIMEOUT_SEC		10
#define SOCKET_RX_TIMEOUT_SEC 			5
#define SYNC_TIMEOUT					5000
#define WORK_RESCHEDULE_TIME_SEC 		10

struct SYNC_data
{
	uint32_t magic_signature;
	uint32_t T[SYNC_T_SIZE];
};

static struct in6_addr mesh_prefix;

// Networ interface
static struct net_if *dect_iface;

// Sockets
static int common_socket = -1;
static int SYNC_socket = -1;

// Application state
static bool nw_beacon_started = false; // TODO: Fix this to more robust solution
static uint32_t best_long_rd_id = 0;
static uint8_t best_route_cost = 0xFF;
static bool dect_connected;
uint32_t message_counter;
uint32_t current_long_rd_id;
static int32_t SYNC_offset_parent;	// The offset time (negative means the FT clock is behind)
static uint32_t SYNC_network_delay_parent;
static uint32_t sibling_ft_long_rd_id = 0; // For FT relay and PT relay to avoid associating between these two

// Semaphores for controlling flow
K_SEM_DEFINE(sem_if_up, 0, 1);
K_SEM_DEFINE(sem_activate, 0, 1);
K_SEM_DEFINE(sem_deactivate, 0, 1);
K_SEM_DEFINE(sem_network_created, 0, 1); // For FT
K_SEM_DEFINE(sem_network_joined, 0, 1); // For PT
K_SEM_DEFINE(sem_association_created, 0, 1);
// Network management callback 
static struct net_mgmt_event_callback net_conn_mgr_cb;
static struct net_mgmt_event_callback net_if_cb;
static struct net_mgmt_event_callback net_activate_cb;
static struct net_mgmt_event_callback dect_event_cb;

// --- Forward declarations ---
#if defined(CONFIG_DK_LIBRARY)
static void button_handler(uint32_t button_states, uint32_t has_changed);
#endif

static void main_mac_print_network_info(struct net_if *iface);

// Sockets
static int open_common_socket(void);
static void close_common_socket(void);
static int open_SYNC_socket(void);
static void close_SYNC_socket(void);

// SYNC
static int SYNC_pt_operation(void);
static int SYNC_ft_operation(void);

// TX and RX threads
static void rx_thread(void);
static void tx_img_data(const uint8_t *image_data, size_t image_size, uint32_t dst_long_rd_id);

// Helper functions
static bool create_ipv6_from_long_rd_id(struct in6_addr *address, uint32_t long_rd_id);
static uint32_t get_parent_long_rd_id(void);
static uint32_t get_first_child_long_rd_id();

// IPv6 creation for device
static void create_and_set_device_ipv6(void);

// Write device settings
static void write_ft_settings(void);
static void write_pt_settings(void);

// DECT NR+ operations
static void start_nw_beacon(void);
static void start_network_scan(void);
static void join_network(uint32_t long_rd_id);

// Main thread operations
static void run_as_ft(void);
static void run_as_pt(void);

// TX work
#if !IS_ENABLED(CONFIG_DECT_RELAY_PT) && !IS_ENABLED(CONFIG_DECT_RELAY_FT)
static void check_spi_image_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tx_work, check_spi_image_work_handler);
static void check_spi_image_work_handler(struct k_work *work)
{
	// First check if dect is connected so device can transmit
	if (!dect_connected)
	{
		LOG_ERR("DECT not connected! Rescheduling work in %d seconds...", WORK_RESCHEDULE_TIME_SEC);
		k_work_schedule(&tx_work, K_SECONDS(WORK_RESCHEDULE_TIME_SEC));
		return;
	}

	// No TX if no new image is available
    if (!spi_slave_is_new_image_available())
	{
		LOG_WRN("No new image available. Rescheduling work in %d seconds...", WORK_RESCHEDULE_TIME_SEC);
		k_work_schedule(&tx_work, K_SECONDS(WORK_RESCHEDULE_TIME_SEC));
		return;
	}

	const uint8_t *image_data = spi_slave_get_image_buffer();
	size_t image_size = spi_slave_get_image_size();
	
	LOG_INF("New image received: %zu bytes", image_size);

	// Get FT parent long RD ID
	uint32_t parent_long_rd_id = get_parent_long_rd_id();
	if (parent_long_rd_id == 0)
	{
		LOG_WRN("Invalid parent long RD ID. Rescheduling work in %d seconds", WORK_RESCHEDULE_TIME_SEC);
		k_work_schedule(&tx_work, K_SECONDS(WORK_RESCHEDULE_TIME_SEC));
		return;
	}

	// Transmit over DECT
	tx_img_data(image_data, image_size, parent_long_rd_id);

	spi_slave_clear_image_flag();

	// Reschedule work
	LOG_INF("Rescheduling work in %d seconds...", WORK_RESCHEDULE_TIME_SEC);
	k_work_schedule(&tx_work, K_SECONDS(WORK_RESCHEDULE_TIME_SEC));
}
#endif /* !CONFIG_DECT_RELAY_PT && !CONFIG_DECT_RELAY_FT */

// LED 2 turn-off work
#if defined(CONFIG_DK_LIBRARY)
static void main_led2_off_work_handler(struct k_work *work)
{
	// Turn off LED 2 after 1 second delay 
	dk_set_led_off(DK_LED2);
}
static K_WORK_DELAYABLE_DEFINE(led2_off_work, main_led2_off_work_handler);
#endif

// Modem fault handler
void nrf_modem_fault_handler(struct nrf_modem_fault_info *fault_info)
{
	LOG_ERR("Modem fault: reason=%d, program_counter=0x%x",
		fault_info->reason, fault_info->program_counter);

	__ASSERT(false, "Modem crash detected, halting application");
}

// Function declarations
#if defined(CONFIG_DK_LIBRARY)
static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	int ret;

	if (has_changed & button_states & DK_BTN1_MSK) {
		/*LOG_INF("Button 1 pressed - initiating connection");
		ret = conn_mgr_if_connect(dect_iface);
		if (ret < 0) {
			LOG_ERR("Failed to initiate connection: %d", ret);
		} else {
			LOG_INF("Connection initiated");
		}*/
	}

	if (has_changed & button_states & DK_BTN2_MSK) {
		LOG_INF("Button 2 pressed - disconnecting");
		ret = conn_mgr_if_disconnect(dect_iface);
		if (ret < 0) {
			LOG_ERR("Failed to disconnect: %d", ret);
		} else {
			LOG_INF("Disconnect initiated");
		}
	}
}
#endif

static int open_common_socket()
{
	common_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (common_socket < 0)
	{
		LOG_ERR("Failed to create common socket: %d", errno);
		common_socket = -1;
		return -errno;
	}

	// Socket options
	struct timeval timeout =
	{
		.tv_sec = SOCKET_RX_TIMEOUT_SEC,
		.tv_usec = 0
	};
	int dect_iface_idx = net_if_get_by_iface(dect_iface);
	int reuse = 1;

	setsockopt(common_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)); // Set timeout
	setsockopt(common_socket, SOL_SOCKET, SO_BINDTODEVICE, &dect_iface_idx, sizeof(dect_iface_idx)); // Bind to DECT interface
	setsockopt(common_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));	// Reuse same socket if connection is restarted

	struct sockaddr_in6 rx_addr = {
		.sin6_family = AF_INET6,
		.sin6_port = htons(COMMON_PORT),
		.sin6_addr = in6addr_any
	};

	int ret = bind(common_socket, (struct sockaddr *)&rx_addr, sizeof(rx_addr));
	if (ret < 0)
	{
		LOG_ERR("Failed to bind common socket: %d", errno);
		close(common_socket);
		common_socket = -1;
		return -errno;
	}

	LOG_INF("Successfully opened common socket");

	return 0;
}

static void close_common_socket(void)
{
	if (common_socket >= 0)
	{
		close(common_socket);
		common_socket = -1;
	}

	LOG_INF("Succesfully closed commonTX and RX sockets");
}

static int open_SYNC_socket(void)
{
	SYNC_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (SYNC_socket < 0)
	{
		LOG_ERR("Failed to create SYNC socket: %d", errno);
		SYNC_socket = -1;
		return -errno;
	}

	// Socket options
	struct timeval timeout =
	{
		.tv_sec = SOCKET_SYNC_RX_TIMEOUT_SEC,
		.tv_usec = 0
	};
	int dect_iface_idx = net_if_get_by_iface(dect_iface);
	int reuse = 1;

	setsockopt(SYNC_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)); // Set timeout
	setsockopt(SYNC_socket, SOL_SOCKET, SO_BINDTODEVICE, &dect_iface_idx, sizeof(dect_iface_idx)); // Bind to DECT interface
	setsockopt(SYNC_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));	// Reuse same socket if connection is restarted

	struct sockaddr_in6 SYNC_rx_addr = {
		.sin6_family = AF_INET6,
		.sin6_port = htons(SYNC_PORT),
		.sin6_addr = in6addr_any
	};

	int ret = bind(SYNC_socket, (struct sockaddr *)&SYNC_rx_addr, sizeof(SYNC_rx_addr));
	if (ret < 0)
	{
		LOG_ERR("Failed to bind SYNC socket: %d", errno);
		close(SYNC_socket);
		SYNC_socket = -1;
		return -errno;
	}

	LOG_INF("Successfully opened SYNC socket");

	return 0;
}

static void close_SYNC_socket(void)
{
	if (SYNC_socket >= 0)
	{
		close(SYNC_socket);
		SYNC_socket = -1;
	}

	LOG_INF("Succesfully closed SYNC TX and RX sockets");
}

static void main_mac_print_network_info(struct net_if *iface)
{
	LOG_INF("=== Network Interface Information ===");
	LOG_INF("Interface: %s", net_if_get_device(iface)->name);
}

static int SYNC_pt_operation(void)
{
	int ret;

	struct SYNC_data SYNC_timestamps = {0};

	uint32_t parent_long_rd_id = get_parent_long_rd_id();
	if (parent_long_rd_id == 0)
	{
		LOG_WRN("Invalid long RD ID for parent");
		return -1;
	}

	struct sockaddr_in6 dst_addr =
	{
		.sin6_family = AF_INET6,
		.sin6_port = htons(SYNC_PORT)
	};
	bool ok = create_ipv6_from_long_rd_id(&dst_addr.sin6_addr, parent_long_rd_id);
	if(!ok)
	{
		LOG_WRN("Failed to create IPv6 address");
		return -1;
	}

	// TX
	if (SYNC_socket < 0)
	{
		LOG_WRN("SYNC socket not open");
		return -1;
	}

	SYNC_timestamps.magic_signature = SYNC_MAGIC_SIGNATURE;
	SYNC_timestamps.T[0] = k_uptime_get_32();
	ret = sendto(SYNC_socket, &SYNC_timestamps, sizeof(SYNC_timestamps), 0,
				(struct sockaddr *)&dst_addr, sizeof(dst_addr));

	if (ret < 0)
	{
		LOG_WRN("Failed to send SYNC packet: %d", errno);
		return -1;
	}
	else LOG_INF("SYNC packet sent");

	struct sockaddr_in6 src_addr;
	socklen_t addr_len = sizeof(src_addr);
	char addr_str[NET_IPV6_ADDR_LEN];

	int max_recv_retries = 5;
	int current_retries = 0;

	while (1)
	{
		if (current_retries > ++max_recv_retries)
		{
			LOG_WRN("Ran out of RX retries. Exiting RX...");
		}

		struct SYNC_data rx_from_parent;

		LOG_INF("Starting SYNC RX...");
		ret = recvfrom(SYNC_socket, &rx_from_parent, sizeof(rx_from_parent), 0,
				(struct sockaddr *)&src_addr, &addr_len);
		uint32_t T_temp = k_uptime_get_32();

		if (ret < 0) // Handle whatever errors
		{
			if (errno == EAGAIN) // Socket timeout
			{
				LOG_WRN("SYNC socket timeout (errno=%d). Retrying RX...", errno);
				continue;
			}

			LOG_WRN("RX packet failed: %d. Retrying RX...", errno);
			continue;
		}

		// Check if packet is correct
		if (rx_from_parent.magic_signature ^ SYNC_MAGIC_SIGNATURE)
		{
			LOG_WRN("Packet signature not matching for SYNC packet. Retrying RX...");
			continue;
		}

		net_addr_ntop(AF_INET6, &src_addr.sin6_addr, addr_str, sizeof(addr_str));
		uint32_t rx_long_rd_id = dect_utils_lib_long_rd_id_from_ipv6_addr(&src_addr.sin6_addr);

		LOG_INF("Received SYNC packet, %d bytes from IPv6=%s (long_rd_id=0x%08x)", ret, addr_str, rx_long_rd_id);

		if (rx_long_rd_id == parent_long_rd_id)
		{
			LOG_INF("RX packet long RD ID matching. Exiting RX...");

			SYNC_timestamps.T[1] = rx_from_parent.T[1];
			SYNC_timestamps.T[2] = rx_from_parent.T[2];
			SYNC_timestamps.T[3] = T_temp;
			break;
		}
		else
		{
			LOG_WRN("Long RD ID not matching. Got 0x%08x, but expected 0x%08x. Retrying RX...", rx_long_rd_id, parent_long_rd_id);
			continue;
		}
	}

	// Calculate total offset
	int32_t signed_T[SYNC_T_SIZE];
	for (int i = 0; i < SYNC_T_SIZE; i++)
	{
		signed_T[i] = (int32_t)SYNC_timestamps.T[i];
	}

	LOG_WRN("T0: %d | T1: %d | T2: %d | T3: %d", signed_T[0], signed_T[1], signed_T[2], signed_T[3]); // TODO: Temp, remove this
	SYNC_offset_parent = ((signed_T[1] - signed_T[0]) + (signed_T[2] - signed_T[3])) / 2;
	SYNC_network_delay_parent = (signed_T[3] - signed_T[0]) - (signed_T[2] - signed_T[1]);

	LOG_INF("PT-FT clock offset: %d", SYNC_offset_parent);

	return 0;
}

static int SYNC_ft_operation(void)
{
	int ret;

	struct SYNC_data SYNC_timestamps = {0};

	uint32_t child_long_rd_id = get_first_child_long_rd_id();
	if (child_long_rd_id == 0)
	{
		LOG_WRN("Invalid long RD ID for child");
		return -1;
	}

	struct sockaddr_in6 src_addr;
	socklen_t addr_len = sizeof(src_addr);
	char addr_str[NET_IPV6_ADDR_LEN];

	// RX
	if (SYNC_socket < 0)
	{
		LOG_WRN("SYNC socket not open");
		return -1;
	}

	int max_recv_retries = 5;
	int current_retries = 0;

	while (1)
	{
		if (current_retries > ++max_recv_retries)
		{
			LOG_WRN("Ran out of RX retries. Exiting RX...");
		}

		struct SYNC_data rx_from_child;

		LOG_INF("Starting SYNC RX...");
		// Timestamp and RX
		ret = recvfrom(SYNC_socket, &rx_from_child, sizeof(rx_from_child), 0,
				(struct sockaddr *)&src_addr, &addr_len);
		uint32_t T_temp = k_uptime_get_32();

		if (ret < 0) // Handle whatever socket errors 
		{
			if (errno == EAGAIN) // Socket timeout
			{
				LOG_WRN("SYNC socket timeout (errno=%d). Retrying RX...", errno);
				continue;
			}

			LOG_WRN("RX packet failed: %d. Retrying RX...", errno);
			continue;
		}

		// Check if packet is correct
		if (rx_from_child.magic_signature ^ SYNC_MAGIC_SIGNATURE)
		{
			LOG_WRN("Packet signature not matching for SYNC packet. Retrying RX...");
			continue;
		}

		net_addr_ntop(AF_INET6, &src_addr.sin6_addr, addr_str, sizeof(addr_str));
		uint32_t rx_long_rd_id = dect_utils_lib_long_rd_id_from_ipv6_addr(&src_addr.sin6_addr);

		LOG_INF("Received SYNC packet, %d bytes from IPv6=%s (long_rd_id=0x%08x)", ret, addr_str, rx_long_rd_id);

		if (rx_long_rd_id == child_long_rd_id)
		{
			LOG_INF("RX packet long RD ID matching. Exiting RX...");

			SYNC_timestamps.T[0] = rx_from_child.T[0];
			SYNC_timestamps.T[1] = T_temp;
			break;
		}
		else
		{
			LOG_WRN("Long RD ID not matching. Got 0x%08x, but got 0x%08x. Retrying RX...", rx_long_rd_id, child_long_rd_id);
			continue;
		}
	}

	struct sockaddr_in6 dst_addr =
	{
		.sin6_family = AF_INET6,
		.sin6_port = htons(SYNC_PORT)
	};
	bool ok = create_ipv6_from_long_rd_id(&dst_addr.sin6_addr, child_long_rd_id);
	if(!ok)
	{
		LOG_ERR("Failed to create IPv6 address");
		return -1;
	}

	struct SYNC_data SYNC_tx_packet = SYNC_timestamps;

	SYNC_tx_packet.magic_signature = SYNC_MAGIC_SIGNATURE;
	SYNC_tx_packet.T[2] = k_uptime_get_32();
	ret = sendto(SYNC_socket, &SYNC_tx_packet, sizeof(SYNC_tx_packet), 0,
				(struct sockaddr *)&dst_addr, sizeof(dst_addr));

	if (ret < 0)
	{
		LOG_WRN("Failed to send SYNC packet: %d", errno);
		return -1;
	}
	else LOG_INF("SYNC packet sent");

	return 0;
}

static void rx_thread(void)
{
    int ret;

    struct sockaddr_in6 src_addr;
    socklen_t addr_len = sizeof(src_addr);

    while (true)
	{
		if (common_socket < 0)
		{
			LOG_WRN("Common socket not open. Sleeping for 1 second...");
			k_sleep(K_SECONDS(1));
			continue;
		}

        struct rx_chunk *chunk = uart_get_free_chunk();

		// RX
        ret = recvfrom(common_socket, chunk->data, CHUNK_BUF_SIZE, 0,
                   (struct sockaddr *)&src_addr, &addr_len);

        if (ret < 0)
		{
            uart_return_free_chunk(chunk);
            LOG_WRN("RX receive failed: %d", errno);
            k_sleep(K_SECONDS(1));
            continue;
        }

        if (ret < (int)sizeof(struct data_packet))
		{
            uart_return_free_chunk(chunk);
            LOG_WRN("Packet too small: %d bytes", ret);
            continue;
        }

        struct data_packet *pkt = (struct data_packet *)chunk->data;
        LOG_INF("Chunk %d/%d (%d bytes)",
            pkt->packet_idx + 1, pkt->total_packets, pkt->payload_len);

        chunk->data_len = ret;
        uart_queue_chunk(chunk);
    }
}

static void tx_img_data(const uint8_t *image_data, size_t image_size, uint32_t dst_long_rd_id)
{
	int ret = -1;

	// Destination address
	struct sockaddr_in6 dst_addr =
	{
		.sin6_family = AF_INET6,
		.sin6_port = htons(COMMON_PORT),
	};

	bool ok = create_ipv6_from_long_rd_id(&dst_addr.sin6_addr, dst_long_rd_id);
	if(!ok)
	{
		LOG_ERR("Failed to create IPv6 address. Aborting transmission");
		return;
	}

	// Send chunks to destination
	uint16_t total_chunks = image_size / MAX_PAYLOAD_SIZE + 1;

	if (common_socket < 0)
	{
		LOG_WRN("Common socket not open. Aborting transmission");
		return;
	}

	for (uint16_t i=0; i < total_chunks; i++)
	{
		size_t offset = i * MAX_PAYLOAD_SIZE;
		size_t payload_len = MIN(MAX_PAYLOAD_SIZE, image_size - offset);
		size_t total_size = sizeof(struct data_packet) + payload_len;

		struct data_packet *packet = malloc(total_size);
		if (packet == NULL)
		{
			LOG_ERR("Memory allocation failed!");
			return;
		}

		packet->packet_idx = i;
		packet->total_data_size = image_size;
		packet->total_packets = total_chunks;
		packet->payload_len = payload_len;

		memcpy(packet->payload, image_data + offset, packet->payload_len);

		ret = sendto(common_socket, packet, total_size, 0,
			(struct sockaddr *)&dst_addr, sizeof(dst_addr));

		if (ret >= 0) // Success
			LOG_INF("Sending chunk %d/%d (%d bytes)", i+1, total_chunks, ret);
		else
			LOG_ERR("Failed to send image chunk to destination: %d", ret);
		
		// Free the packet memory
		free(packet);
	}
	
	LOG_INF("Sent packet to destination");

#if defined(CONFIG_DK_LIBRARY)
		// Cancel any pending LED 2 turn-off work 
		k_work_cancel_delayable(&led2_off_work);
		// Turn on LED 2 to indicate successful transmission 
		dk_set_led_on(DK_LED2);
		// Schedule LED 2 to turn off after 1 second 
		k_work_schedule(&led2_off_work, K_SECONDS(1));
#endif
}

static bool create_ipv6_from_long_rd_id(struct in6_addr *address, uint32_t long_rd_id)
{
	// 64-bit prefix + 32-bit sink long rd id + 32-bit long rd id of device (same as sink)
	bool create_ok = dect_utils_lib_net_ipv6_addr_create_from_sink_and_long_rd_id(
		mesh_prefix,
		DECT_SINK_LONG_RD_ID,
		long_rd_id,
		address
	);
	if(!create_ok)
	{
		LOG_ERR("Faied to create IPv6 address");
	}

	return create_ok;
}

static uint32_t get_parent_long_rd_id(void)
{
	struct dect_status_info dev_info = {0};

	int ret = net_mgmt(NET_REQUEST_DECT_STATUS_INFO_GET, dect_iface, &dev_info, sizeof(dev_info));
	if (ret)
	{
		LOG_ERR("Failed to get device status info: %d", ret);
		return 0;
	}

	return dev_info.parent_associations->long_rd_id;
}

static uint32_t get_first_child_long_rd_id()
{
	struct dect_status_info dev_info = {0};

	int ret = net_mgmt(NET_REQUEST_DECT_STATUS_INFO_GET, dect_iface, &dev_info, sizeof(dev_info));
	if (ret)
	{
		LOG_ERR("Failed to get device status info: %d", ret);
		return 0;
	}

	return dev_info.child_associations[0].long_rd_id;
}

static void create_and_set_device_ipv6(void)
{
	// Read settings
	struct dect_settings dev_settings = {0};

	int ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_READ, dect_iface, &dev_settings, sizeof(dev_settings));
	if (ret)
	{
		LOG_ERR("Failed to read settings: %d", ret);
		return;
	}

	// Write prefix string to prefix struct
	net_addr_pton(AF_INET6, MESH_PREFIX_STR, &mesh_prefix);

	// Construct global IPv6 address
	uint32_t this_rd_id = dev_settings.identities.transmitter_long_rd_id;

	struct in6_addr global_addr;
	if (!create_ipv6_from_long_rd_id(&global_addr, this_rd_id))
	{
		LOG_ERR("Failed to create global IPv6");
		return;
	}

	net_if_ipv6_addr_add(dect_iface, &global_addr, NET_ADDR_MANUAL, 0);

	char addr_str[NET_IPV6_ADDR_LEN];
	net_addr_ntop(AF_INET6, &global_addr, addr_str, sizeof(addr_str));

	LOG_INF("Adding global IPv6: %s", addr_str);
}

static void write_ft_settings(void)
{
	// Read settings
	struct dect_settings dev_settings = {0};

	int ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_READ, dect_iface, &dev_settings, sizeof(dev_settings));
	if (ret)
	{
		LOG_ERR("Failed to read settings: %d", ret);
		return;
	}

	// Device type and long rd id
	dev_settings.device_type = current_device_type;
	#if IS_ENABLED(CONFIG_DECT_RELAY_FT)
		dev_settings.identities.transmitter_long_rd_id = DECT_FT_LONG_RD_ID;
	#else
		dev_settings.identities.transmitter_long_rd_id = DECT_SINK_LONG_RD_ID;
	#endif

	// Network beacon
	// TODO: Fix from magic numbers
	dev_settings.nw_beacon.channel = 1657;
	dev_settings.nw_beacon.beacon_period = DECT_NW_BEACON_PERIOD_1000MS;

	// Write bitmap
	dev_settings.cmd_params.write_scope_bitmap = 
		DECT_SETTINGS_WRITE_SCOPE_DEVICE_TYPE 	|
		DECT_SETTINGS_WRITE_SCOPE_IDENTITIES 	|
		DECT_SETTINGS_WRITE_SCOPE_NW_BEACON;

	// Write settings
	ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_WRITE, dect_iface, &dev_settings, sizeof(dev_settings));
	if (ret)
	{
		LOG_ERR("Failed to write settings: %d", ret);
		return;
	}

	current_long_rd_id = dev_settings.identities.transmitter_long_rd_id;
	#if IS_ENABLED(CONFIG_DECT_RELAY_FT)
		dev_settings.identities.transmitter_long_rd_id = DECT_FT_LONG_RD_ID;
	#else
		dev_settings.identities.transmitter_long_rd_id = DECT_SINK_LONG_RD_ID;
	#endif

	// Create the device IPv6 address
	create_and_set_device_ipv6();

	LOG_INF("DECT sink FT settings successfully set");
}

static void write_pt_settings(void)
{
	// Read settings
	struct dect_settings dev_settings = {0};

	int ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_READ, dect_iface, &dev_settings, sizeof(dev_settings));
	if (ret)
	{
		LOG_ERR("Failed to read settings: %d", ret);
		return;
	}

	// Device type and long RD ID
	dev_settings.device_type = current_device_type;
	#if IS_ENABLED(CONFIG_DECT_RELAY_PT)
		dev_settings.identities.transmitter_long_rd_id = DECT_PT_LONG_RD_ID;
	#else
		dev_settings.identities.transmitter_long_rd_id = DECT_EDGE_PT_LONG_RD_ID;
	#endif

	// Write bitmap
	dev_settings.cmd_params.write_scope_bitmap = 
		DECT_SETTINGS_WRITE_SCOPE_DEVICE_TYPE	|
		DECT_SETTINGS_WRITE_SCOPE_IDENTITIES;

	// Write settings
	ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_WRITE, dect_iface, &dev_settings, sizeof(dev_settings));
	if (ret)
	{
		LOG_ERR("Failed to write settings: %d", ret);
		return;
	}

	current_long_rd_id = dev_settings.identities.transmitter_long_rd_id;

	// Create the device IPv6 address
	create_and_set_device_ipv6();

	#if IS_ENABLED(CONFIG_DECT_RELAY_PT)
		dev_settings.identities.transmitter_long_rd_id = DECT_PT_LONG_RD_ID;
	#else
		dev_settings.identities.transmitter_long_rd_id = DECT_EDGE_PT_LONG_RD_ID;
	#endif
	LOG_INF("DECT PT settings successfully set");
}

static void start_nw_beacon(void)
{
	// TODO: Currently hardcoded. Change this to dynamic channel
	struct dect_nw_beacon_start_req_params nw_beacon_params = {
		.channel = 1657,
		.additional_ch_count = 0,
	};

	int ret = net_mgmt(NET_REQUEST_DECT_NW_BEACON_START, dect_iface, &nw_beacon_params, sizeof(nw_beacon_params)); // Callback to NET_EVENT_DECT_NW_BEACON_START_RESULT
	if (ret)
	{
		LOG_ERR("Network beacon start failed: %d", ret);
	}
}

static void start_network_scan(void)
{
	best_long_rd_id = 0;
	best_route_cost = 0xFF;

	struct dect_scan_params scan_params = 
	{
		.band = 1,
		.channel_count = 0,
		// Maybe add list here
		.channel_scan_time_ms = 500,
	};

	int ret = net_mgmt(NET_REQUEST_DECT_SCAN, dect_iface, &scan_params, sizeof(scan_params)); // Callback to NET_EVENT_DECT_SCAN_RESULT and NET_EVENT_DECT_SCAN_DONE
	if (ret)
	{
		LOG_ERR("Failed to start network scan: %d", ret);
	}
}

static void join_network(uint32_t long_rd_id)
{
	// Write long rd id to settings
	struct dect_settings dev_settings = {0};
	int ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_READ, dect_iface, &dev_settings, sizeof(dev_settings));
	if (ret)
	{
		LOG_ERR("Failed to read settings: %d", ret);
		return;
	}

	dev_settings.network_join.target_ft_long_rd_id = best_long_rd_id;
	dev_settings.cmd_params.write_scope_bitmap |= DECT_SETTINGS_WRITE_SCOPE_NETWORK_JOIN;

	ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_WRITE, dect_iface, &dev_settings, sizeof(dev_settings));
	if (ret)
	{
		LOG_INF("Failed to write settings: %d", ret);
		return;
	}

	// Join network
	ret = net_mgmt(NET_REQUEST_DECT_NETWORK_JOIN, dect_iface, NULL, 0); // Callback to NET_EVENT_DECT_NETWORK_STATUS->Joined
	if (ret)
	{
		LOG_ERR("Network joined failed: %d", ret);
	}
}

#if IS_ENABLED(CONFIG_DECT_RELAY_PT)
static void main_tx_image_message(const uint8_t *data, size_t size)
{
    uint32_t parent_long_rd_id = get_parent_long_rd_id();
    if (parent_long_rd_id == 0) {
        LOG_ERR("No parent RD ID, dropping relayed image");
        return;
    }

    LOG_INF("Relaying image (%zu bytes) to parent 0x%08x", size, parent_long_rd_id);
    tx_img_data(data, size, parent_long_rd_id);
}
#endif

static void run_as_ft(void)
{
	LOG_WRN("Starting as FT");

	int ret = net_mgmt(NET_REQUEST_DECT_NETWORK_CREATE, dect_iface, NULL, 0); // Callback to NET_EVENT_DECT_NETWORK_STATUS->Created
	if (ret == -EALREADY)
	{
		LOG_ERR("Network already created: %d", ret);
	}
	else if (ret)
	{
		LOG_ERR("Network create failed: %d", ret);
	}

	LOG_INF("Blocking until network created...");
	k_sem_take(&sem_network_created, K_FOREVER);

	LOG_INF("Blocking until association created...");
	k_sem_take(&sem_association_created, K_FOREVER);

	// Start SYNC rx
	int success = SYNC_ft_operation();
	while (success < 0)
	{
		success = SYNC_ft_operation();
		k_sleep(K_SECONDS(2)); // Do SYNC operation and sleep retry
	}
	close_SYNC_socket(); // Close SYNC sockets when done to spare resources

	k_sleep(K_SECONDS(20));	// TODO: Temp, remove this

	ret = uart_data_init();
	if (ret)
	{
		LOG_ERR("Failed to initialize UART: %d", ret);
		return;
	}

	rx_thread();
}

static void run_as_pt(void)
{
	LOG_WRN("Starting as PT");

	start_network_scan();

	LOG_INF("Blocking until network joined...");
	k_sem_take(&sem_network_joined, K_FOREVER);

	// TODO: Fix stopping here, if devices are not started at the same time

	LOG_INF("Blocking until association created...");
	k_sem_take(&sem_association_created, K_FOREVER);

	// Start SYNC TX
	int success = SYNC_pt_operation();
	while (success < 0)
	{
		success = SYNC_pt_operation();
		k_sleep(K_SECONDS(10)); // Do SYNC operation and sleep retry
	}
	close_SYNC_socket(); // Close SYNC sockets to save resources

	k_sleep(K_SECONDS(20)); // TODO: Temp, remove this

	#if IS_ENABLED(CONFIG_DECT_RELAY_PT)
    uart_rx_set_frame_callback(main_tx_image_message);
    int ret = uart_data_init();
    if (ret) {
        LOG_ERR("Failed to initialize UART RX: %d", ret);
        return;
    }

	#elif !IS_ENABLED(CONFIG_DECT_RELAY_FT)
	// SPI slave start
	int ret = spi_slave_init();
	if (ret)
	{
		LOG_ERR("Failed to initialize SPI slave: %d", ret);
		return;
	}
	ret = spi_slave_start_thread();
	if (ret)
	{
		LOG_ERR("Failed to start SPI slave thread: %d", ret);
		return;
	}

	k_work_schedule(&tx_work, K_SECONDS(5)); // Start transmitting first after 5 seconds
	#endif /* !CONFIG_DECT_RELAY_PT && !CONFIG_DECT_RELAY_FT */
}

static void net_conn_mgr_event_handler(struct net_mgmt_event_callback *cb,
				       uint64_t mgmt_event, struct net_if *iface)
{
	// Only handle events for our DECT interface 
	if (iface != dect_iface) {
		return;
	}
	if (mgmt_event == NET_EVENT_L4_CONNECTED) {
		LOG_INF("NET_EVENT_L4_CONNECTED");
	} else if (mgmt_event == NET_EVENT_L4_DISCONNECTED) {
		LOG_INF("NET_EVENT_L4_DISCONNECTED");
	}
}

static void net_if_event_handler(struct net_mgmt_event_callback *cb,
				 uint64_t mgmt_event, struct net_if *iface)
{
	// Only handle events for our DECT interface 
	if (iface != dect_iface) return;

	if (mgmt_event == NET_EVENT_IF_UP)
	{
		LOG_INF("DECT NR+ interface is UP");
		main_mac_print_network_info(iface);

		// Update flags
		dect_connected = true;

		// Open sockets
		open_common_socket();
		open_SYNC_socket();

#if defined(CONFIG_DK_LIBRARY)
		// Turn on LED 1 to indicate connection 
		dk_set_led_on(DK_LED1);
#endif

		k_sem_give(&sem_if_up);
	}
	else if (mgmt_event == NET_EVENT_IF_DOWN)
	{
		LOG_INF("DECT NR+ interface is DOWN");

		// Update flags and fields
		dect_connected = false;
		nw_beacon_started = false;
		message_counter = 0;

		// Close sockets
		close_common_socket();
		close_SYNC_socket();

		#if !IS_ENABLED(CONFIG_DECT_RELAY_PT) && !IS_ENABLED(CONFIG_DECT_RELAY_FT)
		k_work_cancel_delayable(&tx_work);
		#endif /* !CONFIG_DECT_RELAY_PT && !CONFIG_DECT_RELAY_FT */

#if defined(CONFIG_DK_LIBRARY)
		// Turn off LED 1 to indicate disconnection 
		dk_set_led_off(DK_LED1);
#endif
	}
}

static void net_activate_handler(struct net_mgmt_event_callback *cb,
				 uint64_t event, struct net_if *iface)
{
	// Only handle events for our DECT interface
	if (iface != dect_iface) return;
	
	if (event == NET_EVENT_DECT_ACTIVATE_DONE)
	{
		const enum dect_status_values *status = cb->info;

		if(*status == DECT_STATUS_OK)
		{
			LOG_INF("DECT stack activated successfully");
			k_sem_give(&sem_activate);
		}
		else
		{
			LOG_ERR("DECT stack activation failed: %d", *status);
		}
	}
	else if (event == NET_EVENT_DECT_DEACTIVATE_DONE)
	{
		const enum dect_status_values *status = cb->info;

		if(*status == DECT_STATUS_OK)
		{
			LOG_INF("DECT stack deactivated successfully");
			k_sem_give(&sem_deactivate);
		}
		else
		{
			LOG_ERR("DECT stack deactivation failed: %d", *status);
		}
	}
}

static void dect_event_handler(struct net_mgmt_event_callback *cb,
                               uint64_t event, struct net_if *iface)
{
    switch (event)
	{
	case NET_EVENT_DECT_NETWORK_STATUS:
		const struct dect_network_status_evt *status = cb->info;

		if (status->network_status == DECT_NETWORK_STATUS_CREATED)
		{
			LOG_INF("Network created");
			if (!nw_beacon_started)
			{
				nw_beacon_started = true;
				start_nw_beacon();
			}
		}
		else if (status->network_status == DECT_NETWORK_STATUS_JOINED)
		{
			LOG_INF("Network joined. Safe to start own cluster");
			k_sem_give(&sem_network_joined);
		}
		else if (status->network_status == DECT_NETWORK_STATUS_UNJOINED)
		{
			LOG_INF("Network unjoined");
		}
		else if (status->network_status == DECT_NETWORK_STATUS_FAILURE)
		{
			LOG_ERR("Network failure");
		}
		else if (status->network_status == DECT_NETWORK_STATUS_REMOVED)
		{
			LOG_ERR("Network removed");
		}

		break;

	case NET_EVENT_DECT_CLUSTER_CREATED_RESULT:
		LOG_INF("Cluster created");
		break;

	case NET_EVENT_DECT_NW_BEACON_START_RESULT:
		const struct dect_common_resp_evt *res = cb->info;

		if (res->status == DECT_STATUS_OK)
		{
			LOG_INF("Network beacon successfully created and running");
			k_sem_give(&sem_network_created);
		}
		else
		{
			LOG_ERR("Network beacon failed: 0x%08x", res->status);
		}

		break;

    case NET_EVENT_DECT_SCAN_RESULT:
		const struct dect_scan_result_evt *result = cb->info;
		const struct dect_route_info *route = &result->route_info;

		// Skip if FT is sibling, since that would mean joining our own FT, which would cause a loop (RELAYS)
		if (sibling_ft_long_rd_id != 0 && result->transmitter_long_rd_id == sibling_ft_long_rd_id) {
			LOG_INF("Skipping sibling FT 0x%08x", sibling_ft_long_rd_id);
			break;
    	}


		LOG_INF("Scan: FT 0x%08x, route_cost=%d",
				result->transmitter_long_rd_id, route->route_cost);

		if (route->route_cost < best_route_cost) {
			best_route_cost = route->route_cost;
			best_long_rd_id = result->transmitter_long_rd_id;
		}
		break;

	case NET_EVENT_DECT_SCAN_DONE:
		if (best_long_rd_id != 0)
		{
			LOG_INF("Scan done. Found RD (long_rd_id=0x%08x) network to join...", best_long_rd_id);
			join_network(best_long_rd_id);
		}
		else
		{
			LOG_WRN("No sink FT found. Retrying...");
			k_msleep(NW_SCAN_RETRY_MS);
			start_network_scan();
		}
        break;

    case NET_EVENT_DECT_RSSI_SCAN_RESULT:
        const struct dect_rssi_scan_result_evt *rssi_result = cb->info;
		const struct dect_rssi_scan_result_data *data = &rssi_result->rssi_scan_result;
		LOG_INF("RSSI scan result: channel=%u, rssi=%d",
				data->channel,
				data->possible_subslot_cnt);  // Log RSSI if channel is free, otherwise log -128 to indicate busy
        break;

	case NET_EVENT_DECT_NEIGHBOR_LIST:
		const struct dect_neighbor_list_evt *neighbor_list = cb->info;
		LOG_INF("Neighbor list received: %d neighbors found", neighbor_list->neighbor_count);
    
		for (int i = 0; i < neighbor_list->neighbor_count; i++) {
			LOG_INF("  Neighbor %d: long_rd_id=0x%08x (%u)",
					i,
					neighbor_list->neighbor_long_rd_ids[i],
					neighbor_list->neighbor_long_rd_ids[i]);
		}
		break;

	case NET_EVENT_DECT_ASSOCIATION_CHANGED:
		const struct dect_association_changed_evt *evt = cb->info;

		if(evt->association_change_type == DECT_ASSOCIATION_CREATED)
		{
			k_sem_give(&sem_association_created);

			LOG_INF("Association created with RD 0x%08x (role: %s)",
                    evt->long_rd_id,
                    evt->neighbor_role == DECT_NEIGHBOR_ROLE_PARENT ? "parent" : "child"
			);
		}
		else if(evt->association_change_type == DECT_ASSOCIATION_RELEASED ||
				evt->association_change_type == DECT_ASSOCIATION_REQ_REJECTED)
		{
			LOG_INF("Association lost with RD 0x%08x", evt->long_rd_id);
		}

		break;

    default:
        LOG_WRN("Unhandled DECT event: 0x%llx", event);
        break;
    }
}

int main(void)
{
	int ret;

	LOG_INF("=== Hello DECT NR+ Sample Application ===");

	/* --- Independent setup and initialization ---*/

	// Setup network management callbacks for L4 connected/disconnected events
	net_mgmt_init_event_callback(&net_conn_mgr_cb, net_conn_mgr_event_handler,
		NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED);
	net_mgmt_add_event_callback(&net_conn_mgr_cb);

	// Setup network interface callbacks
	net_mgmt_init_event_callback(&net_if_cb, net_if_event_handler,
		NET_EVENT_IF_UP | NET_EVENT_IF_DOWN);
	net_mgmt_add_event_callback(&net_if_cb);

	// Setup callback for modem activation event
	net_mgmt_init_event_callback(&net_activate_cb, net_activate_handler,
		NET_EVENT_DECT_ACTIVATE_DONE			|
		NET_EVENT_DECT_DEACTIVATE_DONE);
	net_mgmt_add_event_callback(&net_activate_cb);

	// Setup callbacks for DECT event callbacks
	net_mgmt_init_event_callback(&dect_event_cb, dect_event_handler,
		NET_EVENT_DECT_NETWORK_STATUS			|
		NET_EVENT_DECT_SCAN_RESULT				|
		NET_EVENT_DECT_SCAN_DONE				|
		NET_EVENT_DECT_NW_BEACON_START_RESULT	|
		NET_EVENT_DECT_CLUSTER_CREATED_RESULT	|
		NET_EVENT_DECT_ASSOCIATION_CHANGED);
	net_mgmt_add_event_callback(&dect_event_cb);

	// Get the DECT network interface 
	dect_iface = net_if_get_first_by_type(&NET_L2_GET_NAME(DECT));
	if (!dect_iface) {
		LOG_ERR("No DECT interface found");
		return -ENODEV;
	}

#if defined(CONFIG_DK_LIBRARY)
	// Initialize DK library for buttons and LEDs 
	ret = dk_buttons_init(button_handler);
	if (ret) {
		LOG_WRN("Failed to initialize buttons: %d", ret);
	}

	ret = dk_leds_init();
	if (ret) {
		LOG_WRN("Failed to initialize LEDs: %d", ret);
	} else {
		// Initialize LEDs to OFF state 
		dk_set_led_off(DK_LED1);
		dk_set_led_off(DK_LED2);
	}

	LOG_INF("Press button 1 to connect, button 2 to disconnect");
#endif

	// Write settings
	if (current_device_type & DECT_DEVICE_TYPE_FT) write_ft_settings();
	else if(current_device_type & DECT_DEVICE_TYPE_PT) write_pt_settings();

	// Initialize modem library and this triggers DECT NR+ stack initialization
#if defined(CONFIG_NRF_MODEM_LIB)
	ret = nrf_modem_lib_init();
	if (ret) {
		LOG_ERR("Failed to initialize modem library: %d", ret);
		return ret;
	}
#endif

	// Block until DECT is activated
	LOG_INF("Wait for DECT stack to activate and settings to write...");
	k_sem_take(&sem_activate, K_FOREVER);

	LOG_INF("Hello DECT application started successfully");

	/// --- Sibling FT-PT handshake (only for relay devices) 
	#if IS_ENABLED(CONFIG_DECT_RELAY_FT) || IS_ENABLED(CONFIG_DECT_RELAY_PT)
    	uart_handshake_init();
    	#if IS_ENABLED(CONFIG_DECT_RELAY_FT)
        	uart_handshake_send_id(current_long_rd_id);
    	#elif IS_ENABLED(CONFIG_DECT_RELAY_PT)
        	if (uart_handshake_receive_id(&sibling_ft_long_rd_id, 30)) {
            	LOG_ERR("No sibling FT ID received, scanning without filter");
        	}
    	#endif
	#endif
	///

	// --- Sink FT and regular PT specific ---
	if (current_device_type & DECT_DEVICE_TYPE_FT) // FT
		run_as_ft();
	else if (current_device_type & DECT_DEVICE_TYPE_PT) // PT
		run_as_pt();

	while(1)
		k_sleep(K_SECONDS(1));

	return 0;
}
