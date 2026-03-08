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
#include <zephyr/net/hostname.h>
#include <zephyr/posix/netdb.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/sys/socket.h>

#if defined(CONFIG_DK_LIBRARY)
#include <dk_buttons_and_leds.h>
#endif

#include <modem/nrf_modem_lib.h>
#include <nrf_modem.h>

#include <net/dect/dect_net_l2_mgmt.h>
#include <net/dect/dect_net_l2.h>

#include <math.h>

LOG_MODULE_REGISTER(hello_dect, CONFIG_HELLO_DECT_MAC_LOG_LEVEL);

/* ============================================================
 * Modem fault handler
 * ============================================================ */
void nrf_modem_fault_handler(struct nrf_modem_fault_info *fault_info)
{
	LOG_ERR("Modem fault: reason=%d, program_counter=0x%x",
		fault_info->reason, fault_info->program_counter);
	__ASSERT(false, "Modem crash detected, halting application");
}

/* ============================================================
 * Network interface
 * ============================================================ */
static struct net_if *dect_iface;

uint16_t transmitter_id = CONFIG_DECT_TRANSMITTER_ID;

/* ============================================================
 * Network management callbacks
 * ============================================================ */
static struct net_mgmt_event_callback net_conn_mgr_cb;
static struct net_mgmt_event_callback net_if_cb;
static struct net_mgmt_event_callback dect_scan_cb;
static struct net_mgmt_event_callback dect_assoc_cb;
static struct net_mgmt_event_callback dect_event_cb;

/* ============================================================
 * Application state
 * ============================================================ */
static bool dect_connected;
static uint32_t message_counter;
static atomic_t recv_socket_atomic = ATOMIC_INIT(-1);

/* Socket receive timeout in seconds */
#define SOCKET_RECV_TIMEOUT_SEC 5

/* ============================================================
 * Peer addressing — populated from association events,
 * no hostname resolution needed.
 * ============================================================ */
static struct in6_addr peer_ipv6_addr;
static bool peer_addr_known = false;

/* ============================================================
 * Scan state — used during startup role determination
 * ============================================================ */
static uint32_t best_ft_long_rd_id  = 0;
static uint8_t  best_ft_route_cost  = 0xFF;
static uint32_t sink_long_rd_id     = 0;

#if defined(CONFIG_DK_LIBRARY)
static void hello_dect_led2_off_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(led2_off_work, hello_dect_led2_off_work_handler);

static void hello_dect_led2_off_work_handler(struct k_work *work)
{
	dk_set_led_off(DK_LED2);
}
#endif

/* ============================================================
 * Forward declarations
 * ============================================================ */
static void hello_dect_mac_rx_thread(void);
static void configure_and_connect(dect_device_type_t role, uint32_t target_ft_id);
static void hello_dect_max_tx_work_handler(struct k_work *work);
static void hello_dect_mac_tx_demo_message(void);

/* ============================================================
 * Work items
 * ============================================================ */
static K_WORK_DELAYABLE_DEFINE(tx_work, hello_dect_max_tx_work_handler);

/* ============================================================
 * Settings write + connect
 * ============================================================ */
static void configure_and_connect(dect_device_type_t role, uint32_t target_ft_id)
{
	const struct device *dev = net_if_get_device(dect_iface);
	const struct dect_nr_hal_api *api = dev->api;
	struct dect_settings settings = {0};
	int ret;

	/* Read current settings first to avoid overwriting unrelated fields */
	ret = api->settings_read(dev, &settings);
	if (ret) {
		LOG_ERR("Failed to read settings: %d", ret);
		return;
	}

	/* Set role bitmap: FT, PT, or FT|PT for relay */
	settings.device_type = role;

	/* Set target FT — 0 means ANY */
	settings.network_join.target_ft_long_rd_id =
		(target_ft_id != 0) ? target_ft_id : DECT_SETT_NETWORK_JOIN_TARGET_FT_ANY;

	settings.cmd_params.write_scope_bitmap =
		DECT_SETTINGS_WRITE_SCOPE_DEVICE_TYPE |
		DECT_SETTINGS_WRITE_SCOPE_NETWORK_JOIN;

	ret = api->settings_write(dev, &settings);
	if (ret) {
		LOG_ERR("Failed to write settings: %d", ret);
		return;
	}

	const char *role_str =
		(role == DECT_DEVICE_TYPE_FT)                          ? "FT (root)" :
		(role == DECT_DEVICE_TYPE_PT)                          ? "PT (leaf)" :
		(role == (DECT_DEVICE_TYPE_FT | DECT_DEVICE_TYPE_PT)) ? "FT|PT (relay)" :
		"unknown";

	LOG_INF("Connecting as %s (target FT: 0x%08x)", role_str, target_ft_id);

	ret = conn_mgr_if_connect(dect_iface);
	if (ret) {
		LOG_ERR("conn_mgr_if_connect failed: %d", ret);
	}
}

/* ============================================================
 * Scan event handler — determines role at boot
 * ============================================================ */
static void dect_scan_event_handler(struct net_mgmt_event_callback *cb,
				    uint64_t event, struct net_if *iface)
{
	if (iface != dect_iface) {
		return;
	}

	switch (event) {
	case NET_EVENT_DECT_SCAN_RESULT: {
		const struct dect_scan_result_evt *result = cb->info;

		LOG_INF("Scan: FT 0x%08x ch=%u rssi=%d has_route=%d",
			result->transmitter_long_rd_id,
			result->channel,
			result->rx_signal_info.rssi_2,
			result->has_route_info);

		if (result->has_route_info) {
			LOG_INF("  route_cost=%d sink=0x%08x",
				result->route_info.route_cost,
				result->route_info.sink_address);

			/* Track the best (lowest cost) upstream FT */
			if (result->route_info.route_cost < best_ft_route_cost) {
				best_ft_route_cost  = result->route_info.route_cost;
				best_ft_long_rd_id  = result->transmitter_long_rd_id;
				sink_long_rd_id     = result->route_info.sink_address;
			}
		} else {
			/*
			 * A beacon without route info is still a valid FT to
			 * associate to (e.g. the root FT itself). Track it as
			 * a fallback if nothing with route_info is found.
			 */
			if (best_ft_long_rd_id == 0) {
				best_ft_long_rd_id = result->transmitter_long_rd_id;
				sink_long_rd_id    = result->transmitter_long_rd_id;
			}
		}
		break;
	}

	case NET_EVENT_DECT_SCAN_DONE:
		LOG_INF("Scan complete. Best FT: 0x%08x (route_cost=%d) sink: 0x%08x",
			best_ft_long_rd_id, best_ft_route_cost, sink_long_rd_id);

		if (best_ft_long_rd_id != 0) {
			/*
			 * Upstream FT found.
			 * Use FT|PT (relay) role so this device also beacons
			 * downstream and can accept further associations.
			 * If you want pure leaf behaviour, use DECT_DEVICE_TYPE_PT.
			 */
			configure_and_connect(
				DECT_DEVICE_TYPE_FT | DECT_DEVICE_TYPE_PT,
				best_ft_long_rd_id);
		} else {
			/* No upstream FT — become the root */
			configure_and_connect(DECT_DEVICE_TYPE_FT, 0);
		}
		break;

	case NET_EVENT_DECT_RSSI_SCAN_RESULT: {
		const struct dect_rssi_scan_result_evt *rssi = cb->info;

		LOG_DBG("RSSI: ch=%u free=%d busy_pct=%d",
			rssi->rssi_scan_result.channel,
			rssi->rssi_scan_result.all_subslots_free,
			rssi->rssi_scan_result.busy_percentage);
		break;
	}

	default:
		break;
	}
}

/* ============================================================
 * Association event handler — populates peer IPv6 address
 * ============================================================ */
static void dect_assoc_event_handler(struct net_mgmt_event_callback *cb,
				     uint64_t event, struct net_if *iface)
{
	if (iface != dect_iface) {
		return;
	}

	if (event != NET_EVENT_DECT_ASSOCIATION_CHANGED) {
		return;
	}

	const struct dect_association_changed_evt *evt = cb->info;

	switch (evt->association_change_type) {
	case DECT_ASSOCIATION_CREATED: {
		LOG_INF("Association CREATED with RD 0x%08x (role: %s)",
			evt->long_rd_id,
			evt->neighbor_role == DECT_NEIGHBOR_ROLE_PARENT ? "parent" : "child");

		/* Read full status to get the IPv6 address of the new neighbor */
		const struct device *dev = net_if_get_device(dect_iface);
		const struct dect_nr_hal_api *api = dev->api;
		struct dect_status_info status = {0};

		if (api->status_info_get(dev, &status) != 0) {
			LOG_ERR("Failed to get status info");
			break;
		}

		char addr_str[NET_IPV6_ADDR_LEN];

		/* Check children (we are the FT side) */
		for (int i = 0; i < status.child_count; i++) {
			if (status.child_associations[i].long_rd_id == evt->long_rd_id) {
				memcpy(&peer_ipv6_addr,
				       &status.child_associations[i].local_ipv6_addr,
				       sizeof(peer_ipv6_addr));
				peer_addr_known = true;
				net_addr_ntop(AF_INET6, &peer_ipv6_addr,
					      addr_str, sizeof(addr_str));
				LOG_INF("Child peer IPv6: %s", addr_str);
			}
		}

		/* Check parents (we are the PT side) */
		for (int i = 0; i < status.parent_count; i++) {
			if (status.parent_associations[i].long_rd_id == evt->long_rd_id) {
				memcpy(&peer_ipv6_addr,
				       &status.parent_associations[i].local_ipv6_addr,
				       sizeof(peer_ipv6_addr));
				peer_addr_known = true;
				net_addr_ntop(AF_INET6, &peer_ipv6_addr,
					      addr_str, sizeof(addr_str));
				LOG_INF("Parent peer IPv6: %s", addr_str);
			}
		}
		break;
	}

	case DECT_ASSOCIATION_RELEASED:
		LOG_INF("Association RELEASED with RD 0x%08x (neighbor_initiated=%d, cause=%d)",
			evt->long_rd_id,
			evt->association_released.neighbor_initiated,
			evt->association_released.release_cause);
		peer_addr_known = false;
		break;

	case DECT_ASSOCIATION_REQ_REJECTED:
		LOG_WRN("Association REQ REJECTED by RD 0x%08x (cause=%d)",
			evt->long_rd_id,
			evt->association_req_rejected.reject_cause);
		break;

	case DECT_ASSOCIATION_REQ_FAILED_MDM:
		LOG_ERR("Association REQ FAILED in modem for RD 0x%08x (cause=%d)",
			evt->long_rd_id,
			evt->association_req_failed_mdm.cause);
		break;

	default:
		break;
	}
}

/* ============================================================
 * Neighbor list / misc DECT events
 * ============================================================ */
static void dect_misc_event_handler(struct net_mgmt_event_callback *cb,
				    uint64_t event, struct net_if *iface)
{
	switch (event) {
	case NET_EVENT_DECT_NEIGHBOR_LIST: {
		const struct dect_neighbor_list_evt *nl = cb->info;

		LOG_INF("Neighbor list: %d neighbors", nl->neighbor_count);
		for (int i = 0; i < nl->neighbor_count; i++) {
			LOG_INF("  [%d] long_rd_id=0x%08x", i,
				nl->neighbor_long_rd_ids[i]);
		}
		break;
	}
	default:
		LOG_DBG("Unhandled DECT event: 0x%llx", event);
		break;
	}
}

/* ============================================================
 * Image TX over UDP
 * ============================================================ */
static void hello_dect_max_tx_work_handler(struct k_work *work)
{
	if (!dect_connected)
		return;

	hello_dect_mac_tx_demo_message();

	k_work_schedule(&tx_work, K_SECONDS(10));
}

static void hello_dect_mac_tx_demo_message(void)
{
	int sock;
	int ret = -1;
	char message[512];
	char addr_str[NET_IPV6_ADDR_LEN];

	struct sockaddr_in6 dst = {
		.sin6_family = AF_INET6,
		.sin6_port   = htons(12345),
	};
	memcpy(&dst.sin6_addr, &peer_ipv6_addr, sizeof(peer_ipv6_addr));

	net_addr_ntop(AF_INET6, &dst.sin6_addr, addr_str, sizeof(addr_str));

	sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Failed to create socket: %d", errno);
		return;
	}

	snprintf(message, sizeof(message), "This is a test message!");

	ret = sendto(sock, message, strlen(message), 0,
			     (struct sockaddr *)&dst, sizeof(dst));

	if (ret >= 0) {
		LOG_INF("Sending message success!");
	} else {
		LOG_ERR("sendto failed: %d", ret);
	}

	close(sock);

#if defined(CONFIG_DK_LIBRARY)
	k_work_cancel_delayable(&led2_off_work);
	dk_set_led_on(DK_LED2);
	k_work_schedule(&led2_off_work, K_SECONDS(1));
#endif

	if (ret >= 0) {
		LOG_INF("Image TX complete");
	}
}

/* ============================================================
 * UDP receive thread (runs in main thread context)
 * ============================================================ */
static int hello_dect_mac_start_udp_listener(void)
{
	struct sockaddr_in6 addr = {0};
	struct timeval timeout = {
		.tv_sec  = SOCKET_RECV_TIMEOUT_SEC,
		.tv_usec = 0,
	};
	int ret;
	int sock;

	if (atomic_get(&recv_socket_atomic) >= 0) {
		return 0; /* Already listening */
	}

	sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		LOG_ERR("Failed to create receive socket: %d", errno);
		return -errno;
	}

	ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	if (ret < 0) {
		LOG_WRN("Failed to set recv timeout: %d", errno);
	}

	addr.sin6_family = AF_INET6;
	addr.sin6_port   = htons(12345);
	addr.sin6_addr   = in6addr_any;

	ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0) {
		LOG_ERR("Failed to bind receive socket: %d", errno);
		close(sock);
		return -errno;
	}

	atomic_set(&recv_socket_atomic, sock);
	LOG_INF("UDP listener started on port 12345");
	return 0;
}

static void hello_dect_mac_stop_udp_listener(void)
{
	int sock = atomic_get(&recv_socket_atomic);

	if (sock >= 0) {
		atomic_set(&recv_socket_atomic, -1);
		close(sock);
		LOG_INF("UDP listener stopped");
	}
}

static void hello_dect_mac_rx_thread(void)
{
	char buffer[256];
	struct sockaddr_in6 src_addr;
	socklen_t addr_len;
	int ret;
	int sock;
	char addr_str[NET_IPV6_ADDR_LEN];

	while (1) {
		sock = atomic_get(&recv_socket_atomic);
		if (sock < 0) {
			k_sleep(K_SECONDS(1));
			continue;
		}

		addr_len = sizeof(src_addr);
		ret = recvfrom(sock, buffer, sizeof(buffer), 0,
			       (struct sockaddr *)&src_addr, &addr_len);

		if (ret < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				continue;
			}
			if (atomic_get(&recv_socket_atomic) < 0) {
				LOG_DBG("Socket closed, waiting for reconnect");
				continue;
			}
			LOG_ERR("recvfrom failed: %d", errno);
			k_sleep(K_SECONDS(1));
			continue;
		}

		if (ret > 0) {
			buffer[ret] = '\0';
			net_addr_ntop(AF_INET6, &src_addr.sin6_addr,
				      addr_str, sizeof(addr_str));
			LOG_INF("Received %d bytes from %s: %s",
				ret, addr_str, buffer);	
		}
	}
}

/* ============================================================
 * Network management event handlers
 * ============================================================ */
static void net_conn_mgr_event_handler(struct net_mgmt_event_callback *cb,
				       uint64_t mgmt_event, struct net_if *iface)
{
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
	if (iface != dect_iface) {
		return;
	}

	if (mgmt_event == NET_EVENT_IF_UP) {
		LOG_INF("DECT NR+ interface UP");
		dect_connected = true;

		LOG_INF("Interface: %s", net_if_get_device(iface)->name);

		hello_dect_mac_start_udp_listener();
		k_work_schedule(&tx_work, K_SECONDS(5));

#if defined(CONFIG_DK_LIBRARY)
		dk_set_led_on(DK_LED1);
#endif

	} else if (mgmt_event == NET_EVENT_IF_DOWN) {
		LOG_INF("DECT NR+ interface DOWN");
		dect_connected   = false;
		peer_addr_known  = false;
		message_counter  = 0;

		hello_dect_mac_stop_udp_listener();
		k_work_cancel_delayable(&tx_work);

#if defined(CONFIG_DK_LIBRARY)
		dk_set_led_off(DK_LED1);
#endif
	}
}

/* ============================================================
 * Button handler
 * ============================================================ */
#if defined(CONFIG_DK_LIBRARY)
static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	int ret;

	if (has_changed & button_states & DK_BTN1_MSK) {
		LOG_INF("Button 1 — initiating connection");
		ret = conn_mgr_if_connect(dect_iface);
		if (ret < 0) {
			LOG_ERR("Failed to connect: %d", ret);
		}
	}

	if (has_changed & button_states & DK_BTN2_MSK) {
		LOG_INF("Button 2 — disconnecting");
		ret = conn_mgr_if_disconnect(dect_iface);
		if (ret < 0) {
			LOG_ERR("Failed to disconnect: %d", ret);
		}
	}
}
#endif

/* ============================================================
 * main
 * ============================================================ */
int main(void)
{
	int err;

	LOG_INF("=== Hello DECT NR+ Mesh Application ===");

	/* Network management callbacks */
	net_mgmt_init_event_callback(&net_conn_mgr_cb, net_conn_mgr_event_handler,
				     NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED);
	net_mgmt_add_event_callback(&net_conn_mgr_cb);

	net_mgmt_init_event_callback(&net_if_cb, net_if_event_handler,
				     NET_EVENT_IF_UP | NET_EVENT_IF_DOWN);
	net_mgmt_add_event_callback(&net_if_cb);

	/* Scan events — used for role determination at startup */
	net_mgmt_init_event_callback(&dect_scan_cb, dect_scan_event_handler,
				     NET_EVENT_DECT_SCAN_RESULT      |
				     NET_EVENT_DECT_RSSI_SCAN_RESULT |
				     NET_EVENT_DECT_SCAN_DONE);
	net_mgmt_add_event_callback(&dect_scan_cb);

	/* Association events — used to learn peer IPv6 addresses */
	net_mgmt_init_event_callback(&dect_assoc_cb, dect_assoc_event_handler,
				     NET_EVENT_DECT_ASSOCIATION_CHANGED);
	net_mgmt_add_event_callback(&dect_assoc_cb);

	/* Misc DECT events (neighbor list, etc.) */
	net_mgmt_init_event_callback(&dect_event_cb, dect_misc_event_handler,
				     NET_EVENT_DECT_NEIGHBOR_LIST);
	net_mgmt_add_event_callback(&dect_event_cb);

	/* Get DECT interface */
	dect_iface = net_if_get_first_by_type(&NET_L2_GET_NAME(DECT));
	if (!dect_iface) {
		LOG_ERR("No DECT interface found");
		return -ENODEV;
	}

	/* Init modem — triggers DECT NR+ stack init */
#if defined(CONFIG_NRF_MODEM_LIB)
	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("Modem lib init failed: %d", err);
		return err;
	}
#endif

#if defined(CONFIG_DK_LIBRARY)
	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_WRN("Button init failed: %d", err);
	}

	err = dk_leds_init();
	if (err) {
		LOG_WRN("LED init failed: %d", err);
	} else {
		dk_set_led_off(DK_LED1);
		dk_set_led_off(DK_LED2);
	}
#endif

	/*
	 * Scan first — role (FT / PT / FT|PT relay) is decided in
	 * dect_scan_event_handler on NET_EVENT_DECT_SCAN_DONE.
	 * configure_and_connect() is called from there.
	 *
	 * If the scan request itself fails we fall back to root FT mode.
	 */
	LOG_INF("Starting DECT scan to determine role...");

	struct dect_scan_params scan_params = {
		.band             = 1,
		.channel_count    = 0,    /* 0 = all channels in band */
		.channel_scan_time_ms = 500,
	};

	err = net_mgmt(NET_REQUEST_DECT_SCAN, dect_iface,
		       &scan_params, sizeof(scan_params));
	if (err) {
		LOG_ERR("Scan request failed (%d) — falling back to root FT", err);
		configure_and_connect(DECT_DEVICE_TYPE_FT, 0);
	}

	LOG_INF("DECT mesh application started");

#if defined(CONFIG_DK_LIBRARY)
	LOG_INF("Button 1: connect  Button 2: disconnect");
#endif

	/* Block here running the UDP receive loop */
	hello_dect_mac_rx_thread();

	return 0;
}
