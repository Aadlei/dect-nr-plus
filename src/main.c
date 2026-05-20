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

/* Headers */
#include "spi.h"
#include "uart.h"
#include "sync.h"
#include "dect_net.h"

LOG_MODULE_REGISTER(main, CONFIG_HELLO_DECT_MAC_LOG_LEVEL);

#if defined(CONFIG_DECT_RELAY_FT)
const static dect_device_type_t current_device_type = DECT_DEVICE_TYPE_FT;
#elif defined(CONFIG_DECT_RELAY_PT)
const static dect_device_type_t current_device_type = DECT_DEVICE_TYPE_PT;
#else
const static dect_device_type_t current_device_type = DECT_DEVICE_TYPE_PT;
#endif

#define COMMON_PORT 					12345
#define NW_SCAN_RETRY_MS 				2000
#define SOCKET_RX_TIMEOUT_SEC 			5
#define WORK_RESCHEDULE_TIME_MSEC 		500


// Networ interface
static struct net_if *dect_iface;

// Sockets
static int common_socket = -1;

// Application state
static bool nw_beacon_started = false; // TODO: Fix this to more robust solution
static bool dect_connected;
uint32_t message_counter;
static int32_t SYNC_offset_parent;	// The offset time (negative means the FT clock is behind)
static uint32_t sibling_ft_long_rd_id = 0; // For FT relay and PT relay to avoid associating between these two
static uint32_t pending_sync_child_id = 0;
#if IS_ENABLED(CONFIG_DECT_RELAY_PT)
static int32_t sibling_ft_offset = 0; // For FT relay and PT relay to calculate clock offset over UART
#endif

// Semaphores for controlling flow
K_SEM_DEFINE(sem_activate, 0, 1);
K_SEM_DEFINE(sem_network_created, 0, 1); // For FT
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


// TX and RX threads
static void rx_thread(void);
static void tx_img_data(const uint8_t *image_data, uint32_t image_size, struct hop_delays delay_information, uint32_t dst_long_rd_id, uint16_t seq_num);


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
		LOG_ERR("DECT not connected! Rescheduling work in %d ms...", WORK_RESCHEDULE_TIME_MSEC);
		k_work_schedule(&tx_work, K_MSEC(WORK_RESCHEDULE_TIME_MSEC));
		return;
	}

	// No TX if no new image is available
    if (!spi_slave_is_new_image_available())
	{
		LOG_WRN("No new image available. Rescheduling work in %d ms...", WORK_RESCHEDULE_TIME_MSEC);
		k_work_schedule(&tx_work, K_MSEC(WORK_RESCHEDULE_TIME_MSEC));
		return;
	}

	const uint8_t *image_data = spi_slave_get_image_buffer();
	uint32_t image_size = (uint32_t)spi_slave_get_image_size();
	
	LOG_INF("New image received: %zu bytes", image_size);

	// Get FT parent long RD ID
	uint32_t parent_long_rd_id = dect_net_get_parent_long_rd_id();
	if (parent_long_rd_id == 0)
	{
		LOG_WRN("Invalid parent long RD ID. Rescheduling work in %d ms", WORK_RESCHEDULE_TIME_MSEC);
		k_work_schedule(&tx_work, K_MSEC(WORK_RESCHEDULE_TIME_MSEC));
		return;
	}

	// Transmit over DECT (as edge the delay information is empty)
	struct hop_delays empty_delay_information = {
		.num_links = 0,
		.devices_visited = {0},
		.per_link_delay = {0},
	};
	empty_delay_information.devices_visited[0] = dect_net_get_current_long_rd_id();

	static uint16_t image_seq_num = 0;
	tx_img_data(image_data, image_size, empty_delay_information, parent_long_rd_id, image_seq_num++);

	spi_slave_clear_image_flag();

	// Reschedule work
	LOG_INF("Rescheduling work in %d ms...", WORK_RESCHEDULE_TIME_MSEC);
	k_work_schedule(&tx_work, K_MSEC(WORK_RESCHEDULE_TIME_MSEC));
} /* !CONFIG_DECT_RELAY_PT && !CONFIG_DECT_RELAY_FT */

#elif IS_ENABLED(CONFIG_DECT_RELAY_PT)
static void main_relay_tx(const uint8_t *data, uint32_t len, const struct packet_metadata *meta);
#endif

#if !IS_ENABLED(CONFIG_DECT_RELAY_PT)
static void ft_sync_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ft_sync_work, ft_sync_work_handler);

static void ft_sync_work_handler(struct k_work *work)
{
    LOG_INF("FT: running SYNC with child 0x%08x", pending_sync_child_id);
    int ret = sync_ft_operation(pending_sync_child_id);
    if (ret == -ENOTCONN) {
		LOG_WRN("FT SYNC: socket not ready, retrying in 1s");
		k_work_schedule(&ft_sync_work, K_SECONDS(1));
		return;
	}
    if (ret < 0) {
        LOG_WRN("FT SYNC failed (child may have disconnected)");
    }
}
#endif

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


#if !IS_ENABLED(CONFIG_DECT_RELAY_FT)

static bool pt_modem_resetting = false;
static void pt_watchdog_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(pt_watchdog_work, pt_watchdog_work_handler);

static bool pt_operational_init_done = false;

static void pt_reconnect_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(pt_reconnect_work, pt_reconnect_work_handler);

static void pt_post_assoc_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(pt_post_assoc_work, pt_post_assoc_work_handler);

static void pt_reconnect_work_handler(struct k_work *work)
{
    LOG_INF("PT: re-scanning for parent...");
    dect_net_start_scan();
}

K_THREAD_STACK_DEFINE(pt_sync_stack, 4096);
static struct k_thread pt_sync_thread;
static uint32_t pt_sync_parent_id;

static void pt_watchdog_work_handler(struct k_work *work)
{
    if (pt_modem_resetting) {
        k_work_schedule(&pt_watchdog_work, K_SECONDS(5));
        return;
    }
    if (dect_net_get_parent_long_rd_id() != 0) {
        k_work_schedule(&pt_watchdog_work, K_SECONDS(30));
        return;
    }
    LOG_WRN("PT watchdog: not associated, resetting modem");
    pt_modem_resetting = true;
    int ret = net_mgmt(NET_REQUEST_DECT_DEACTIVATE, dect_iface, NULL, 0);
    if (ret) {
        LOG_ERR("Deactivate failed: %d, falling back to rescan", ret);
        pt_modem_resetting = false;
        dect_net_start_scan();
        k_work_schedule(&pt_watchdog_work, K_SECONDS(20));
    }
}

static void pt_sync_thread_fn(void *p1, void *p2, void *p3)
{
    int ret = sync_pt_operation(pt_sync_parent_id, &SYNC_offset_parent);
    if (ret == -ENOTCONN) {
        LOG_WRN("PT SYNC: socket not ready, retrying in 1s");
        k_work_schedule(&pt_post_assoc_work, K_SECONDS(1));
        return;
    }
    if (ret < 0) {
        LOG_WRN("PT SYNC failed: %d", ret);
        return;
    }

    if (pt_operational_init_done) {
        return;
    }
    pt_operational_init_done = true;

    #if IS_ENABLED(CONFIG_DECT_RELAY_PT)
    uart_rx_set_frame_callback(main_relay_tx);
    int init_ret = uart_data_init();
    if (init_ret) {
        LOG_ERR("Failed to initialize UART RX: %d", init_ret);
        return;
    }
    #else
    int init_ret = spi_slave_init();
    if (init_ret) {
        LOG_ERR("Failed to initialize SPI slave: %d", init_ret);
        return;
    }
    init_ret = spi_slave_start_thread();
    if (init_ret) {
        LOG_ERR("Failed to start SPI slave thread: %d", init_ret);
        return;
    }
    k_work_schedule(&tx_work, K_SECONDS(5));
    #endif
}

static void pt_post_assoc_work_handler(struct k_work *work)
{
    uint32_t parent_long_rd_id = dect_net_get_parent_long_rd_id();
    if (parent_long_rd_id == 0) {
        LOG_WRN("PT post-assoc: no parent yet, retrying");
        k_work_schedule(&pt_reconnect_work, K_SECONDS(2));
        return;
    }

    pt_sync_parent_id = parent_long_rd_id;
    k_thread_create(&pt_sync_thread, pt_sync_stack,
                    K_THREAD_STACK_SIZEOF(pt_sync_stack),
                    pt_sync_thread_fn, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
    k_thread_name_set(&pt_sync_thread, "pt_sync");
}
#endif

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

static void main_mac_print_network_info(struct net_if *iface)
{
	LOG_INF("=== Network Interface Information ===");
	LOG_INF("Interface: %s", net_if_get_device(iface)->name);
}


static void rx_thread(void)
{
    int ret;

    struct sockaddr_in6 src_addr;
    socklen_t addr_len = sizeof(src_addr);

    struct data_packet *pkt_recv = malloc(CHUNK_BUF_SIZE);
    if (!pkt_recv)
    {
        LOG_ERR("Failed to allocate memory for RX buffer");
        return;
    }

    while (true)
    {
        if (common_socket < 0)
        {
            LOG_WRN("Common socket not open. Sleeping for 1 second...");
            k_sleep(K_SECONDS(1));
            continue;
        }

        ret = recvfrom(common_socket, pkt_recv, CHUNK_BUF_SIZE, 0,
            (struct sockaddr *)&src_addr, &addr_len);

        if (ret < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				// Do not sleep
				continue;
			}

            LOG_WRN("RX receive failed: %d. Sleeping for 1 second...", errno);
            k_sleep(K_SECONDS(1));
            continue;
        }

        if (ret < (int)sizeof(struct data_packet))
        {
            LOG_WRN("Packet too small: %d bytes", ret);
            continue;
        }

        LOG_INF("Chunk %d/%d (%d bytes)",
            pkt_recv->packet_idx + 1, pkt_recv->total_packets, sizeof(struct data_packet) + pkt_recv->payload_len);

        #if IS_ENABLED(CONFIG_DECT_RELAY_FT)
        {
            uint8_t rssi_idx = pkt_recv->route_delays.num_links + 1;
			if (rssi_idx < ROUTING_MAX_HOPS) {
				pkt_recv->route_delays.per_link_rssi[rssi_idx] = dect_net_get_rx_rssi(&src_addr);
			}
					}
        #endif

        /* Sink FT: stamp delay and RSSI for the final incoming hop. */
        #if !IS_ENABLED(CONFIG_DECT_RELAY_PT) && !IS_ENABLED(CONFIG_DECT_RELAY_FT)
        uint8_t route_delays_idx = pkt_recv->route_delays.num_links;

        if (route_delays_idx >= ROUTING_MAX_HOPS - 1) {
            LOG_ERR("route_delays_idx %d out of bounds, dropping chunk", route_delays_idx);
            continue;
        }

        int32_t current_delay = (route_delays_idx > 0)
		? (int32_t)pkt_recv->route_delays.per_link_delay[route_delays_idx]
		: 0;
		
        uint32_t ft_this_timestamp = k_uptime_get_32();          // T_B
        uint32_t pt_prev_timestamp = pkt_recv->timestamp_pt;     // T_A
        int32_t  offset_pt_to_ft   = pkt_recv->offset_pt_to_ft; // O_AB
        int32_t  cumulative_delay = current_delay + (ft_this_timestamp - (pt_prev_timestamp + offset_pt_to_ft));

		// Only print for last packet in sequence
		if (pkt_recv->packet_idx + 1 == pkt_recv->total_packets)
		{
			LOG_INF("current_delay: %d", current_delay);
			LOG_INF("ft_this_timestamp: %u", ft_this_timestamp);
			LOG_INF("pt_prev_timestamp: %u", pt_prev_timestamp);
			LOG_INF("offset_pt_to_ft: %d", offset_pt_to_ft);
			LOG_INF("cumulative_delay: %d", cumulative_delay);
		}	

        int8_t rx_rssi = dect_net_get_rx_rssi(&src_addr);

        struct hop_delays delay_information = {
            .num_links = ++route_delays_idx,
        };

        for (int i = 0; i < ROUTING_MAX_HOPS; i++) {
            delay_information.per_link_delay[i]  = pkt_recv->route_delays.per_link_delay[i];
            delay_information.devices_visited[i] = pkt_recv->route_delays.devices_visited[i];
            delay_information.per_link_rssi[i]   = pkt_recv->route_delays.per_link_rssi[i];
        }
        delay_information.per_link_delay[route_delays_idx]  = cumulative_delay;
        delay_information.devices_visited[route_delays_idx] = dect_net_get_current_long_rd_id();
        delay_information.per_link_rssi[route_delays_idx]   = rx_rssi; 

        pkt_recv->route_delays = delay_information;
        #endif

        struct rx_chunk *chunk = uart_get_free_chunk();
        memcpy(chunk->data, pkt_recv, ret);
        chunk->data_len = ret;
        uart_queue_chunk(chunk);
    }

    free(pkt_recv);
}
static void tx_img_data(const uint8_t *image_data, uint32_t image_size, struct hop_delays delay_information, uint32_t dst_long_rd_id, uint16_t seq_num)
{
	int ret = -1;

	// Destination address
	struct sockaddr_in6 dst_addr =
	{
		.sin6_family = AF_INET6,
		.sin6_port = htons(COMMON_PORT),
	};

	bool ok = dect_net_create_ipv6(&dst_addr.sin6_addr, dst_long_rd_id);
	if(!ok)
	{
		LOG_ERR("Failed to create IPv6 address. Aborting transmission");
		return;
	}

	// Send chunks to destination
	uint16_t total_chunks = (image_size + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;

	if (common_socket < 0)
	{
		LOG_WRN("Common socket not open. Aborting transmission");
		return;
	}

	uint32_t time_tx = k_uptime_get_32(); // PT edge: Time at the start of TX of all chunks

	struct data_packet *packet = malloc(sizeof(struct data_packet) + MAX_PAYLOAD_SIZE);
	if (packet == NULL)
	{
		LOG_ERR("Memory allocation failed!");
		return;
	}

	for (uint16_t i = 0; i < total_chunks; i++)
	{
		size_t data_offset = i * MAX_PAYLOAD_SIZE;
		size_t payload_len = MIN(MAX_PAYLOAD_SIZE, image_size - data_offset);
		size_t total_size = sizeof(struct data_packet) + payload_len;

		// Packet detail overhead
		packet->packet_idx = i;
		packet->total_packets = total_chunks;
		packet->total_data_size = image_size;
		packet->seq_num = seq_num;
		
		// Time/delays related
		packet->timestamp_pt = time_tx;
		packet->offset_pt_to_ft = SYNC_offset_parent;
		packet->route_delays = delay_information;
	
		// Payload
		packet->payload_len = payload_len;
		memcpy(packet->payload, image_data + data_offset, packet->payload_len);

		ret = sendto(common_socket, packet, total_size, 0,
			(struct sockaddr *)&dst_addr, sizeof(dst_addr));

		if (ret >= 0) { // Success
			LOG_INF("Sending chunk %d/%d (%d bytes)", i+1, total_chunks, ret);
		} else {
			LOG_ERR("Failed to send image chunk to destination: %d", ret);
		}
	}

	// Free the packet memory
	free(packet);
	
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


#if IS_ENABLED(CONFIG_DECT_RELAY_PT)
static void main_relay_tx(const uint8_t *data, uint32_t data_size, const struct packet_metadata *meta)
{
    uint32_t parent_long_rd_id = dect_net_get_parent_long_rd_id();
    if (parent_long_rd_id == 0) {
        LOG_ERR("No parent RD ID, dropping relayed image");
        return;
    }

	// Calculate cumulative delay
	uint8_t route_delays_idx = meta->route_delays.num_links;

	if (route_delays_idx >= ROUTING_MAX_HOPS) {
		LOG_ERR("route_delays_idx %d out of bounds (max %d), dropping frame",
				route_delays_idx, ROUTING_MAX_HOPS);
		return;
	}

	int32_t current_delay = (route_delays_idx > 0)
    ? (int32_t)meta->route_delays.per_link_delay[route_delays_idx]
    : 0;
	uint32_t pt_this_timestamp = k_uptime_get_32(); // T_C_2
	uint32_t pt_prev_timestamp = meta->timestamp_pt; // T_A
	int32_t offset_pt_to_ft = meta->offset_pt_to_ft; // O_AB
	// sibling_ft_offset // O_CB
	int32_t cumulative_delay = current_delay + (pt_this_timestamp - (pt_prev_timestamp + offset_pt_to_ft - sibling_ft_offset));
	
	LOG_INF("current_delay: %d", current_delay);
	LOG_INF("pt_this_timestamp: %u", pt_this_timestamp);
	LOG_INF("pt_prev_timestamp: %u", pt_prev_timestamp);
	LOG_INF("offset_pt_to_ft: %d", offset_pt_to_ft);
	LOG_INF("sibling_ft_offset: %d", sibling_ft_offset);
	LOG_INF("cumulative_delay: %d", cumulative_delay);

	// Update values in struct
	struct hop_delays delay_information = {
		.num_links = ++route_delays_idx,
	};
	
	for (int i = 0; i < ROUTING_MAX_HOPS; i++) {
		delay_information.per_link_delay[i]  = meta->route_delays.per_link_delay[i];
		delay_information.devices_visited[i] = meta->route_delays.devices_visited[i];
		delay_information.per_link_rssi[i]   = meta->route_delays.per_link_rssi[i]; // add this
	}
	
	delay_information.per_link_delay[route_delays_idx] = cumulative_delay;
	delay_information.devices_visited[route_delays_idx] = dect_net_get_current_long_rd_id();

    LOG_INF("Relaying image (%zu bytes) to parent 0x%08x", data_size, parent_long_rd_id);
	tx_img_data(data, data_size, delay_information, parent_long_rd_id, meta->seq_num);
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

	ret = uart_data_init();
	if (ret)
	{
		LOG_ERR("Failed to initialize UART: %d", ret);
		return;
	}

	rx_thread();
}

/* Is event driven, which is why there is little lines here */
static void run_as_pt(void)
{
	LOG_WRN("Starting as PT");
    dect_net_start_scan();
	#if !IS_ENABLED(CONFIG_DECT_RELAY_FT)
    k_work_schedule(&pt_watchdog_work, K_SECONDS(30));
	#endif
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
		sync_open_socket();

#if defined(CONFIG_DK_LIBRARY)
		// Turn on LED 1 to indicate connection 
		dk_set_led_on(DK_LED1);
#endif

		
	}
	else if (mgmt_event == NET_EVENT_IF_DOWN)
	{
		LOG_INF("DECT NR+ interface is DOWN");

		// Update flags and fields
		dect_connected = false;
		nw_beacon_started = false;
		message_counter = 0; // TODO: Not used at all?

		// Close sockets
		close_common_socket();
		sync_close_socket();

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
	
	if (event == NET_EVENT_DECT_ACTIVATE_DONE) {
    const enum dect_status_values *status = cb->info;
    if (*status == DECT_STATUS_OK) {
        LOG_INF("DECT stack activated successfully");
        #if !IS_ENABLED(CONFIG_DECT_RELAY_FT)
        if (pt_modem_resetting) {
            pt_modem_resetting = false;
            LOG_WRN("PT modem reset complete, rescanning");
            dect_net_start_scan();
            k_work_schedule(&pt_watchdog_work, K_SECONDS(30));
            return;
        }
        #endif
        k_sem_give(&sem_activate);
		} else {
			LOG_ERR("DECT stack activation failed: %d", *status);
		}
		}
		else if (event == NET_EVENT_DECT_DEACTIVATE_DONE) {
			const enum dect_status_values *status = cb->info;
			if (*status == DECT_STATUS_OK) {
				#if !IS_ENABLED(CONFIG_DECT_RELAY_FT)
				if (pt_modem_resetting) {
					LOG_WRN("PT modem deactivated, reactivating");
					net_mgmt(NET_REQUEST_DECT_ACTIVATE, dect_iface, NULL, 0);
					return;
				}
				#endif
				LOG_INF("DECT stack deactivated successfully");
				k_sem_give(&sem_activate);
			} else {
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
				dect_net_start_beacon();
			}
		}
		else if (status->network_status == DECT_NETWORK_STATUS_JOINED)
		{
			LOG_INF("Network joined. Safe to start own cluster");
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

    case NET_EVENT_DECT_SCAN_RESULT: {
		const struct dect_scan_result_evt *result = cb->info;
		const struct dect_route_info *route = &result->route_info;

		// Edge PT: skip the sink, force relay path. TODO: Remove this after testing relays.
		#if !IS_ENABLED(CONFIG_DECT_RELAY_PT) && !IS_ENABLED(CONFIG_DECT_RELAY_FT)
		/* if (result->transmitter_long_rd_id == DECT_SINK_LONG_RD_ID) {
			LOG_INF("Edge PT ignoring sink FT 0x%08x (forcing relay)", 
					result->transmitter_long_rd_id);
			break;
		} */

		// Skip if FT is sibling, since that would mean joining our own FT, which would cause a loop (RELAYS)
		if (sibling_ft_long_rd_id != 0 && result->transmitter_long_rd_id == sibling_ft_long_rd_id) {
			LOG_INF("Skipping sibling FT 0x%08x", sibling_ft_long_rd_id);
			break;
    	}
		#endif

		// PT cheat: Skip specific FT long RD IDs
		#if !IS_ENABLED(CONFIG_DECT_RELAY_FT)
		uint32_t this_long_rd_id = dect_net_get_current_long_rd_id();
		uint32_t transmitter_long_rd_id = result->transmitter_long_rd_id;

		if (this_long_rd_id == 0xAAAAAAAAU) {
			if (transmitter_long_rd_id == 0xDDDDDDDDU || transmitter_long_rd_id == DECT_SINK_LONG_RD_ID) {
				LOG_INF("Edge PT ignoring unintended FT device: 0x%08x as 0x%08x",
				transmitter_long_rd_id, this_long_rd_id);
				break;
			}
		}
		else if (this_long_rd_id == 0xCCCCCCCCU) {
			if (transmitter_long_rd_id == 0xBBBBBBBBU || transmitter_long_rd_id == DECT_SINK_LONG_RD_ID) {
				LOG_INF("Edge PT ignoring unintended FT device: 0x%08x as 0x%08x",
				transmitter_long_rd_id, this_long_rd_id);
				break;
			}
		}
		else if (this_long_rd_id == 0xEEEEEEEEU) {
			if (transmitter_long_rd_id == 0xBBBBBBBBU || transmitter_long_rd_id == 0xDDDDDDDDU) {
				LOG_INF("Edge PT ignoring unintended FT device: 0x%08x as 0x%08x",
				transmitter_long_rd_id, this_long_rd_id);
				break;
			}
		}
		else {
			LOG_ERR("current_device_type: %u | long_rd_id: %u", current_device_type, this_long_rd_id);
			break;
		}
		#endif 

		LOG_INF("Scan: FT 0x%08x, route_cost=%d",
				result->transmitter_long_rd_id, route->route_cost);

		dect_net_consider_ft(result->transmitter_long_rd_id, route->route_cost);

		break;
	}

	case NET_EVENT_DECT_SCAN_DONE: {
		if (dect_net_has_best_ft()) {
			dect_net_join_best();
		} else {
			k_msleep(NW_SCAN_RETRY_MS);
			dect_net_start_scan();
		}
        break;
	}

    case NET_EVENT_DECT_RSSI_SCAN_RESULT: {
        const struct dect_rssi_scan_result_evt *rssi_result = cb->info;
		const struct dect_rssi_scan_result_data *data = &rssi_result->rssi_scan_result;
		LOG_INF("RSSI scan result: channel=%u, rssi=%d",
				data->channel,
				data->possible_subslot_cnt);  // Log RSSI if channel is free, otherwise log -128 to indicate busy
        break;
	}

	case NET_EVENT_DECT_NEIGHBOR_LIST: {
		const struct dect_neighbor_list_evt *neighbor_list = cb->info;
		LOG_INF("Neighbor list received: %d neighbors found", neighbor_list->neighbor_count);
    
		for (int i = 0; i < neighbor_list->neighbor_count; i++) {
			LOG_INF("  Neighbor %d: long_rd_id=0x%08x (%u)",
					i,
					neighbor_list->neighbor_long_rd_ids[i],
					neighbor_list->neighbor_long_rd_ids[i]);
		}
		break;
	}

	case NET_EVENT_DECT_ASSOCIATION_CHANGED: {
    const struct dect_association_changed_evt *evt = cb->info;

    if (evt->association_change_type == DECT_ASSOCIATION_CREATED) {
        k_sem_give(&sem_association_created);
        LOG_INF("Association created with RD 0x%08x (role: %s)",
                evt->long_rd_id,
                evt->neighbor_role == DECT_NEIGHBOR_ROLE_PARENT ? "parent" : "child");

        #if !IS_ENABLED(CONFIG_DECT_RELAY_PT)
        if (evt->neighbor_role == DECT_NEIGHBOR_ROLE_CHILD) {
            pending_sync_child_id = evt->long_rd_id;
            k_work_schedule(&ft_sync_work, K_NO_WAIT);
        }
        #endif

        #if !IS_ENABLED(CONFIG_DECT_RELAY_FT)
        if (evt->neighbor_role == DECT_NEIGHBOR_ROLE_PARENT) {
            k_work_schedule(&pt_post_assoc_work, K_NO_WAIT);
        }
        #endif
    }
    else if (evt->association_change_type == DECT_ASSOCIATION_RELEASED ||
             evt->association_change_type == DECT_ASSOCIATION_REQ_REJECTED) {
        LOG_INF("Association lost with RD 0x%08x", evt->long_rd_id);

        #if !IS_ENABLED(CONFIG_DECT_RELAY_FT)
        if (evt->neighbor_role == DECT_NEIGHBOR_ROLE_PARENT) {
            LOG_WRN("PT lost parent, reconnecting in 2s");
            k_work_schedule(&pt_reconnect_work, K_SECONDS(2));
        }
        #endif
    }
    break;
}
	case NET_EVENT_DECT_NEIGHBOR_INFO: {
        const struct dect_neighbor_info_evt *info = cb->info;
 
        if (info->status != DECT_STATUS_OK) {
            LOG_WRN("Neighbor info failed for 0x%08x: %d", info->long_rd_id, info->status);
            break;
        }
 
        int8_t rssi = info->last_rx_signal_info.rssi_2;
        dect_net_update_rssi(info->long_rd_id, rssi);
 
        LOG_DBG("RSSI cache update: 0x%08x → %d dBm (SNR %d dB, MCS %u)",
                info->long_rd_id, rssi,
                info->last_rx_signal_info.snr,
                info->last_rx_signal_info.mcs);
        break;
    }

    default: {
        LOG_WRN("Unhandled DECT event: 0x%llx", event);
        break;
	}
	}
}

int main(void)
{
	int ret;

	// Read reset fields
	uint32_t reset_reason = NRF_POWER->RESETREAS;
	LOG_WRN("Reset reason before clearing: 0x%08x", reset_reason);
	NRF_POWER->RESETREAS = reset_reason;
	reset_reason = NRF_POWER->RESETREAS;
	LOG_WRN("Reset register after clearing: 0x%08x", reset_reason);

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
        NET_EVENT_DECT_NETWORK_STATUS           |
        NET_EVENT_DECT_SCAN_RESULT              |
        NET_EVENT_DECT_SCAN_DONE                |
        NET_EVENT_DECT_NW_BEACON_START_RESULT   |
        NET_EVENT_DECT_CLUSTER_CREATED_RESULT   |
        NET_EVENT_DECT_ASSOCIATION_CHANGED      |
        NET_EVENT_DECT_NEIGHBOR_INFO);          
		
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

	dect_net_init(dect_iface);

	// Write settings
	if (current_device_type & DECT_DEVICE_TYPE_FT) dect_net_write_ft_settings();
	else if (current_device_type & DECT_DEVICE_TYPE_PT) dect_net_write_pt_settings();

	sync_init(dect_iface, dect_net_get_mesh_prefix(), DECT_SINK_LONG_RD_ID);

	// Initialize modem library and this triggers DECT NR+ stack initialization
#if defined(CONFIG_NRF_MODEM_LIB)
	ret = nrf_modem_lib_init();
	if (ret) {
		LOG_ERR("Failed to initialize modem library: %d", ret);
		return ret;
	}
#endif

	ret = net_mgmt(NET_REQUEST_DECT_ACTIVATE, dect_iface, NULL, 0);
	if (ret) {
		LOG_ERR("Failed to activate stack: %d", ret);
	}

	// Block until DECT is activated
	LOG_INF("Wait for DECT stack to activate and settings to write...");
	k_sem_take(&sem_activate, K_FOREVER);

	LOG_INF("Hello DECT application started successfully");

	/// --- Sibling FT-PT handshake (only for relay devices) 
	#if IS_ENABLED(CONFIG_DECT_RELAY_FT) || IS_ENABLED(CONFIG_DECT_RELAY_PT)
    	uart_handshake_init();
    	#if IS_ENABLED(CONFIG_DECT_RELAY_FT)
			uart_handshake_send_id_timestamp(dect_net_get_current_long_rd_id());

			#elif IS_ENABLED(CONFIG_DECT_RELAY_PT)
        	if (uart_handshake_receive_id_timestamp(&sibling_ft_long_rd_id, &sibling_ft_offset, 30)) {
            	LOG_ERR("No sibling FT ID received, scanning without filter");
        	}
    	#endif
	#endif

	// --- Sink FT and regular PT specific ---
	if (current_device_type & DECT_DEVICE_TYPE_FT) // FT
		run_as_ft();
	else if (current_device_type & DECT_DEVICE_TYPE_PT) // PT
		run_as_pt();

	while(1)
		k_sleep(K_SECONDS(1));

	return 0;
}