/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

// NOTE TO SELF: If the tx_work is not scheduled and not running, REMEMBER TO CONNECT ANOTHER DEVICE AS RECEIVER

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
#include <net/dect/dect_utils.h>

// CHANGE THIS BASED ON TYPE OF DEVICE: DECT_DEVICE_TYPE_FT for sink FT; DECT_DEVICE_TYPE_PT for FTPT
const static dect_device_type_t current_device_type = DECT_DEVICE_TYPE_PT;
static char *mesh_prefix_str = "fd12:3456:789a::";
static uint16_t common_port = 12345;
static struct in6_addr mesh_prefix;
#define DECT_SINK_LONG_RD_ID 0x67214200U

LOG_MODULE_REGISTER(main, CONFIG_HELLO_DECT_MAC_LOG_LEVEL);

/* Modem fault handler */
void nrf_modem_fault_handler(struct nrf_modem_fault_info *fault_info)
{
	LOG_ERR("Modem fault: reason=%d, program_counter=0x%x",
		fault_info->reason, fault_info->program_counter);

	__ASSERT(false, "Modem crash detected, halting application");
}

/* Network interface */
static struct net_if *dect_iface;

/* Network management callback */
static struct net_mgmt_event_callback net_conn_mgr_cb;
static struct net_mgmt_event_callback net_if_cb;
static struct net_mgmt_event_callback net_activate_cb;
static struct net_mgmt_event_callback dect_event_cb;

/* Application state */
static char local_hostname[32];
static struct net_in6_addr peer_addr;
static bool peer_addr_known = false;
static bool dect_connected;
static bool nw_beacon_started = false;
static struct dect_settings dev_settings = {0};
static uint32_t message_counter;
static atomic_t recv_socket_atomic = ATOMIC_INIT(-1);

/* Socket receive timeout in seconds */
#define SOCKET_RECV_TIMEOUT_SEC 5

/* Forward declarations */
static void main_max_tx_work_handler(struct k_work *work);
static void main_mac_tx_demo_message(void);
static void main_mac_set_hostname(void);
static void main_mac_rx_thread(void);
static void construct_and_add_global_addr(void);

/* Demo work definition */
static K_WORK_DELAYABLE_DEFINE(tx_work, main_max_tx_work_handler);

// Semaphor for right order of events in main
K_SEM_DEFINE(dect_activate_sem, 0, 1);
K_SEM_DEFINE(dect_network_created_sem, 0, 1); // For sink
K_SEM_DEFINE(dect_network_joined_sem, 0, 1); // For else

/* LED 2 turn-off work definition */
#if defined(CONFIG_DK_LIBRARY)
static void main_led2_off_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(led2_off_work, main_led2_off_work_handler);
#endif

static void main_max_tx_work_handler(struct k_work *work)
{
	if (dect_connected) {
		main_mac_tx_demo_message();

		/* Reschedule work */
		k_work_schedule(&tx_work,
				K_SECONDS(30));
	}
}

#if defined(CONFIG_DK_LIBRARY)
static void main_led2_off_work_handler(struct k_work *work)
{
	/* Turn off LED 2 after 1 second delay */
	dk_set_led_off(DK_LED2);
}
#endif

static void dect_event_handler(struct net_mgmt_event_callback *cb,
                               uint64_t event, struct net_if *iface)
{
    switch (event) {
    case NET_EVENT_DECT_SCAN_RESULT:
        const struct dect_scan_result_evt *result = cb->info;
		const struct dect_route_info *sink_result = &result->route_info;

        LOG_INF("Found FT: long_rd_id=0x%08x, has_route_info=%s sink_long_rd_id=0x%08x channel=%u, rssi=%d",
                result->transmitter_long_rd_id,
				result->has_route_info ? "true" : "false",
				sink_result->sink_address,
                result->channel,
				result->rx_signal_info.rssi_2);  // Just log the first subslot verdict for simplicity
        break;

    case NET_EVENT_DECT_RSSI_SCAN_RESULT:
        const struct dect_rssi_scan_result_evt *rssi_result = cb->info;
		const struct dect_rssi_scan_result_data *data = &rssi_result->rssi_scan_result;
		LOG_INF("RSSI scan result: channel=%u, rssi=%d",
				data->channel,
				data->possible_subslot_cnt);  // Log RSSI if channel is free, otherwise log -128 to indicate busy
        break;

    case NET_EVENT_DECT_SCAN_DONE:
        LOG_INF("Scan complete");
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
			LOG_INF("Association created with RD 0x%08x (role: %s)",
                    evt->long_rd_id,
                    evt->neighbor_role == DECT_NEIGHBOR_ROLE_PARENT ? "parent" : "child"
			);

			// Configure global IPv6
			construct_and_add_global_addr();

			struct dect_status_info status;
			const struct device *dev = net_if_get_device(dect_iface);
			const struct dect_nr_hal_api *api = dev->api;

			if (api->status_info_get(dev, &status) == 0)
			{
				for (int i = 0; i < status.child_count; i++)
				{
					if (status.child_associations[i].long_rd_id == evt->long_rd_id)
					{
						memcpy(&peer_addr, &status.child_associations[i].local_ipv6_addr, sizeof(peer_addr));
						peer_addr_known = true;

						char addr_str[NET_IPV6_ADDR_LEN];
						net_addr_ntop(AF_INET6, &peer_addr, addr_str, sizeof(addr_str));
						LOG_INF("Peer IPv6: %s", addr_str);
					}
				}
				for (int i = 0; i < status.parent_count; i++)
				{
                    if (status.parent_associations[i].long_rd_id == evt->long_rd_id)
					{
                        memcpy(&peer_addr, &status.parent_associations[i].local_ipv6_addr, sizeof(peer_addr));
                        peer_addr_known = true;
                    }
                }
			}

		}
		else if(evt->association_change_type == DECT_ASSOCIATION_RELEASED ||
				evt->association_change_type == DECT_ASSOCIATION_REQ_REJECTED)
		{
			peer_addr_known = false;
			LOG_INF("Association lost with RD 0x%08x", evt->long_rd_id);
		}

		break;

	case NET_EVENT_DECT_NETWORK_STATUS:
		const struct dect_network_status_evt *status = cb->info;

		if (status->network_status == DECT_NETWORK_STATUS_CREATED)
		{
			if (!nw_beacon_started)
			{
				LOG_INF("Network created. Starting beacon...");

				// Configure IPv6 address
				construct_and_add_global_addr();

				struct dect_nw_beacon_start_req_params nw_beacon_params = {
					.channel = 1657,
					.additional_ch_count = 0,
				};

				net_mgmt(NET_REQUEST_DECT_NW_BEACON_START, dect_iface, &nw_beacon_params, sizeof(nw_beacon_params));

				nw_beacon_started = true;
			}
		}
		else if (status->network_status == DECT_NETWORK_STATUS_JOINED)
		{
			LOG_INF("Network joined. Safe to start own cluster");
			k_sem_give(&dect_network_joined_sem);
		}
		else if (status->network_status == DECT_NETWORK_STATUS_UNJOINED)
		{
			LOG_INF("Network unjoined");
		}
		else if (status->network_status == DECT_NETWORK_STATUS_FAILURE)
		{
			LOG_ERR("Network error: %d", status->network_status);
		}

		// TODO: Code here if network is quit or something
		break;

	case NET_EVENT_DECT_NW_BEACON_START_RESULT:
		const struct dect_common_resp_evt *res = cb->info;

		if (res->status == DECT_STATUS_OK)
		{
			LOG_INF("Network beacon successfully created");
			k_sem_give(&dect_network_created_sem);
		}
		else
		{
			LOG_ERR("Network beacon failed: 0x%08x", res->status);
		}

		break;

    default:
        LOG_WRN("Unhandled DECT event: 0x%llx", event);
        break;
    }
}

static void main_mac_tx_demo_message(void)
{
	int sock, ret;
	char message[512];

	// Read long RD ID to put in message
	ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_READ, dect_iface, &dev_settings, sizeof(dev_settings));
	if (ret)
	{
		LOG_ERR("Failed to read settings: %d", ret);
	}

	uint32_t long_rd_id = dev_settings.identities.transmitter_long_rd_id;

	snprintf(message, sizeof(message),
		"Hello DECT NR+ from 0x%08x! Counter #%u",
		long_rd_id, ++message_counter);

	sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0)
	{
		LOG_ERR("Failed to create socket: %d", errno);
		return;
	}

	// Find IPv6 address of sink
	// 64-bit prefix + 32-bit sink long rd id + 32-bit long rd id of device (same as sink)
	struct sockaddr_in6 sock_addr = 
	{
		.sin6_family = AF_INET6,
		.sin6_port = htons(common_port)
	};

	bool ok;
	if (message_counter % 2 == 0)
	{
		ok = dect_utils_lib_net_ipv6_addr_create_from_sink_and_long_rd_id(
		mesh_prefix,
		DECT_SINK_LONG_RD_ID,
		0x12345678,
		&sock_addr.sin6_addr
	);
	}
	else
	{
		ok = dect_utils_lib_net_ipv6_addr_create_from_sink_and_long_rd_id(
		mesh_prefix,
		DECT_SINK_LONG_RD_ID,
		DECT_SINK_LONG_RD_ID,
		&sock_addr.sin6_addr
	);
	}
	
	if(!ok)
	{
		LOG_ERR("Failed to create IPv6 address");
		return;
	}

	ret = sendto(sock, message, strlen(message), 0,
				(struct sockaddr *)&sock_addr, sizeof(sock_addr));
	

	if (ret < 0)
	{
		LOG_ERR("Failed to send message: %d", errno);
	}
	else
	{
		LOG_INF("Sent: %s", message);

#if defined(CONFIG_DK_LIBRARY)
		/* Cancel any pending LED 2 turn-off work */
		k_work_cancel_delayable(&led2_off_work);
		/* Turn on LED 2 to indicate successful transmission */
		dk_set_led_on(DK_LED2);
		/* Schedule LED 2 to turn off after 1 second */
		k_work_schedule(&led2_off_work, K_SECONDS(1));
#endif
	}

	close(sock);
}

static void main_mac_print_network_info(struct net_if *iface)
{
	LOG_INF("=== Network Interface Information ===");
	LOG_INF("Interface: %s", net_if_get_device(iface)->name);
}

static int main_mac_start_udp_listener(void)
{
	struct timeval timeout = {
		.tv_sec = SOCKET_RECV_TIMEOUT_SEC,
		.tv_usec = 0
	};
	int ret;
	int sock;

	if (atomic_get(&recv_socket_atomic) >= 0)
	{
		return 0;  // Already listening
	}

	sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0)
	{
		LOG_ERR("Failed to create receive socket: %d", errno);
		return -errno;
	}

	// Set receive timeout to avoid blocking forever
	ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	if (ret < 0)
	{
		LOG_WRN("Failed to set socket receive timeout: %d", errno);
		// Continue anyway - socket will block indefinitely
	}

	struct sockaddr_in6 recv_addr = {
		.sin6_family = AF_INET6,
		.sin6_port = htons(common_port),
		.sin6_addr = in6addr_any
	};

	ret = bind(sock, (struct sockaddr *)&recv_addr, sizeof(recv_addr));
	if (ret < 0)
	{
		LOG_ERR("Failed to bind receive socket: %d", errno);
		close(sock);
		return -errno;
	}

	atomic_set(&recv_socket_atomic, sock);
	LOG_INF("UDP listener started on port %d (timeout: %ds)", common_port, SOCKET_RECV_TIMEOUT_SEC);
	return 0;
}

static void main_mac_stop_udp_listener(void)
{
	int sock = atomic_get(&recv_socket_atomic);

	if (sock >= 0) {
		atomic_set(&recv_socket_atomic, -1);
		close(sock);
		LOG_INF("UDP listener stopped");
	}
}

static void main_mac_rx_thread(void)
{
	char buffer[256];
	struct sockaddr_in6 src_addr;
	socklen_t addr_len;
	int ret;
	int sock;
	char addr_str[NET_IPV6_ADDR_LEN];

	while (1)
	{
		sock = atomic_get(&recv_socket_atomic);
		if (sock < 0)
		{
			k_sleep(K_SECONDS(1));
			continue;
		}

		addr_len = sizeof(src_addr);
		ret = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
			       (struct sockaddr *)&src_addr, &addr_len);

		if (ret < 0)
		{
			// Timeout - check if socket is still valid and continue
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				continue;
			
			// Check if socket was closed by another thread
			if (atomic_get(&recv_socket_atomic) < 0)
			{
				LOG_DBG("Socket closed, waiting for reconnect");
				continue;
			}

			LOG_ERR("recvfrom failed: %d", errno);
			k_sleep(K_SECONDS(1));
			continue;
		}

		if (ret > 0)
		{
			buffer[ret] = '\0';
			net_addr_ntop(AF_INET6, &src_addr.sin6_addr,
				      addr_str, sizeof(addr_str));
			LOG_INF("Received %d bytes from %s: %s",
				ret, addr_str, buffer);
		}
	}
}

static void net_conn_mgr_event_handler(struct net_mgmt_event_callback *cb,
				       uint64_t mgmt_event, struct net_if *iface)
{
	/* Only handle events for our DECT interface */
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
	/* Only handle events for our DECT interface */
	if (iface != dect_iface) {
		return;
	}
	if (mgmt_event == NET_EVENT_IF_UP) {
		LOG_INF("DECT NR+ interface is UP with local IPv6 addressing");
		dect_connected = true;
		main_mac_print_network_info(iface);

		/* Start UDP listener */
		main_mac_start_udp_listener();

		/* Start demo work and schedule peer resolution */
		if (current_device_type & DECT_DEVICE_TYPE_PT)
			k_work_schedule(&tx_work, K_SECONDS(5));  /* First run after 5 seconds */

#if defined(CONFIG_DK_LIBRARY)
		/* Turn on LED 1 to indicate connection */
		dk_set_led_on(DK_LED1);
#endif

	} else if (mgmt_event == NET_EVENT_IF_DOWN) {
		LOG_INF("DECT NR+ interface is DOWN");
		dect_connected = false;
		peer_addr_known = false;
		/* Reset message counter for new session */
		message_counter = 0;
		main_mac_stop_udp_listener();
		k_work_cancel_delayable(&tx_work);

#if defined(CONFIG_DK_LIBRARY)
		/* Turn off LED 1 to indicate disconnection */
		dk_set_led_off(DK_LED1);
#endif
	}
}

#if defined(CONFIG_DK_LIBRARY)
static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	int ret;

	if (has_changed & button_states & DK_BTN1_MSK) {
		LOG_INF("Button 1 pressed - initiating connection");
		ret = conn_mgr_if_connect(dect_iface);
		if (ret < 0) {
			LOG_ERR("Failed to initiate connection: %d", ret);
		} else {
			LOG_INF("Connection initiated");
		}
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

static void main_mac_set_hostname(void)
{
	snprintf(local_hostname, sizeof(local_hostname),
		"dev-%d", CONFIG_DECT_TRANSMITTER_ID);

	if (net_hostname_set(local_hostname, strlen(local_hostname))) {
		LOG_ERR("Failed to set hostname %s", local_hostname);
	} else {
		LOG_INF("Hostname set to: %s", local_hostname);
	}
}

static void net_activate_handler(struct net_mgmt_event_callback *cb,
				 uint64_t event, struct net_if *iface)
{
	// Only handle events for our DECT interface
	if (iface != dect_iface) {
		return;
	}
	
	if (event == NET_EVENT_DECT_ACTIVATE_DONE)
	{
		const enum dect_status_values *status = cb->info;

		if(*status == DECT_STATUS_OK)
		{
			LOG_INF("DECT stack activated successfully.");
			k_sem_give(&dect_activate_sem);
		}
		else
		{
			LOG_ERR("DECT stack activation failed: %d", *status);
		}
	}
}

static void read_and_write_settings(void)
{
	// Read and write settings
	int ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_READ, dect_iface, &dev_settings, sizeof(dev_settings));
	if (ret)
	{
		LOG_ERR("Failed to read settings: %d", ret);
		return;
	}

	// Also set rd id and prefix fields
	net_addr_pton(AF_INET6, mesh_prefix_str, &mesh_prefix);

	struct dect_settings cp_dev_settings = dev_settings;

	cp_dev_settings.device_type = current_device_type;
	cp_dev_settings.cmd_params.write_scope_bitmap = DECT_SETTINGS_WRITE_SCOPE_DEVICE_TYPE;

	if(current_device_type & DECT_DEVICE_TYPE_FT) // Sink FT specific settings
	{
		cp_dev_settings.identities.transmitter_long_rd_id = DECT_SINK_LONG_RD_ID; // Only change long rd id if this is sink

		cp_dev_settings.cmd_params.write_scope_bitmap |= DECT_SETTINGS_WRITE_SCOPE_IDENTITIES;
	}
	else // FTPT/PT specific settings
	{
		cp_dev_settings.cluster.max_beacon_tx_power_dbm = 23;
		cp_dev_settings.cluster.max_cluster_power_dbm = 23;
		cp_dev_settings.cluster.beacon_period = DECT_CLUSTER_BEACON_PERIOD_500MS;
		cp_dev_settings.cluster.max_num_neighbors = 10;
		cp_dev_settings.cluster.neighbor_inactivity_disconnect_timer_ms = 0;
		cp_dev_settings.cluster.channel_loaded_percent = 75;

		cp_dev_settings.cmd_params.write_scope_bitmap |= DECT_SETTINGS_WRITE_SCOPE_CLUSTER;
	}

	ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_WRITE, dect_iface, &cp_dev_settings, sizeof(cp_dev_settings));
	if (ret)
	{
		LOG_ERR("Failed to write settings: %d", ret);
		return;
	}

	LOG_INF("DECT settings read and set");
}

static void construct_and_add_global_addr(void)
{
	// TODO: Maybe mutex on dev_settings
	net_mgmt(NET_REQUEST_DECT_SETTINGS_READ, dect_iface, &dev_settings, sizeof(dev_settings));

	uint32_t this_rd_id = dev_settings.identities.transmitter_long_rd_id;

	struct in6_addr global_addr;
	bool ok = dect_utils_lib_net_ipv6_addr_create_from_sink_and_long_rd_id(
		mesh_prefix,
		DECT_SINK_LONG_RD_ID,
		this_rd_id,
		&global_addr
	);
	if(!ok)
	{
		LOG_ERR("Failed to create IPv6 address");
		return;
	}

	char addr_str[NET_IPV6_ADDR_LEN];
	net_addr_ntop(AF_INET6, &global_addr, addr_str, sizeof(addr_str));
	LOG_INF("Adding global IPv6: %s", addr_str);

	net_if_ipv6_addr_add(dect_iface, &global_addr, NET_ADDR_MANUAL, 0);

}

static void start_cluster(void)
{
	struct dect_cluster_start_req_params cluster_params = 
	{
		.channel = DECT_CLUSTER_CHANNEL_ANY,
	};

	int ret = net_mgmt(NET_REQUEST_DECT_CLUSTER_START, dect_iface, &cluster_params, sizeof(cluster_params));
	if (ret)
	{
		LOG_ERR("Cluster start failed: %d", ret);
		return;
	}

	LOG_INF("Cluster start request successfull");
}

int main(void)
{
	int err;

	LOG_INF("=== Hello DECT NR+ Sample Application ===");

	/* --- Independent setup and initialization ---*/

	// Set hostname based on device type
	main_mac_set_hostname();

	// Setup network management callbacks for L4 connected/disconnected events
	net_mgmt_init_event_callback(&net_conn_mgr_cb, net_conn_mgr_event_handler,
				     NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED);
	net_mgmt_add_event_callback(&net_conn_mgr_cb);

	// Setup callback for modem activation event
	net_mgmt_init_event_callback(&net_activate_cb, net_activate_handler,
		NET_EVENT_DECT_ACTIVATE_DONE);
	net_mgmt_add_event_callback(&net_activate_cb);

	// Setup callbacks for other DECT stuff 
	net_mgmt_init_event_callback(&dect_event_cb, dect_event_handler,
    	NET_EVENT_DECT_SCAN_RESULT      	|
    	NET_EVENT_DECT_RSSI_SCAN_RESULT 	|
    	NET_EVENT_DECT_SCAN_DONE			|
		NET_EVENT_DECT_NEIGHBOR_LIST		|
		NET_EVENT_DECT_ASSOCIATION_CHANGED	|
		NET_EVENT_DECT_NETWORK_STATUS		|
		NET_EVENT_DECT_NW_BEACON_START_RESULT);
	net_mgmt_add_event_callback(&dect_event_cb);

	net_mgmt_init_event_callback(&net_if_cb,
				     net_if_event_handler,
				     (NET_EVENT_IF_UP |
				      NET_EVENT_IF_DOWN));
	net_mgmt_add_event_callback(&net_if_cb);

	/* Get the DECT network interface */
	dect_iface = net_if_get_first_by_type(&NET_L2_GET_NAME(DECT));
	if (!dect_iface) {
		LOG_ERR("No DECT interface found");
		return -ENODEV;
	}

	// Initialize modem library and this triggers DECT NR+ stack initialization
#if defined(CONFIG_NRF_MODEM_LIB)
	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("Failed to initialize modem library: %d", err);
		return err;
	}
#endif

	// Block until DECT is activated
	LOG_INF("Wait for DECT stack to activate...");
	k_sem_take(&dect_activate_sem, K_FOREVER);

	read_and_write_settings();

#if defined(CONFIG_DK_LIBRARY)
	/* Initialize DK library for buttons and LEDs */
	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_WRN("Failed to initialize buttons: %d", err);
	}

	err = dk_leds_init();
	if (err) {
		LOG_WRN("Failed to initialize LEDs: %d", err);
	} else {
		/* Initialize LEDs to OFF state */
		dk_set_led_off(DK_LED1);
		dk_set_led_off(DK_LED2);
	}

	LOG_INF("Press button 1 to connect, button 2 to disconnect");
#endif

// Todo: Swap this with manual stuff, because we dont know what is going on here
/*#if !defined(CONFIG_NET_L2_DECT_CONN_MGR_AUTO_CONNECT)
	// Initiate connection using connection manager
	LOG_INF("Initiating DECT connection...");
	err = conn_mgr_if_connect(dect_iface);
	if (err) {
		LOG_ERR("Failed to initiate connection: %d", err);
		return err;
	}
#endif
*/
	LOG_INF("Hello DECT application started successfully");


	/* --- Sink FT and regular FTPT specific --- */

	// Sink FT
	if (current_device_type == DECT_DEVICE_TYPE_FT)
	{
		// What autoconnect does:
		// 1. Network scan
		// 2. RSSI scan
		// 3. Starting cluster on best channel
		// 4. Cluster configured
		// 5. Network created

		// Manual stuff:
		// 1. RSSI scan
		// 2. Create network
		// 3. Network beacon
		// 4. Create cluster
		// 5. Cluster beacon

		// RSSI scan
		
		// Create network and beacon
		net_mgmt(NET_REQUEST_DECT_NETWORK_CREATE, dect_iface, NULL, 0);
		
		// Block until network created and beacon transmitted
		LOG_INF("Blocking until network created");
		k_sem_take(&dect_network_created_sem, K_FOREVER);

		// Main application loop - run UDP receive in main thread
		main_mac_rx_thread();
	}
	// Regular FTPT (FTs and PTs)
	else if (current_device_type == DECT_DEVICE_TYPE_PT)
	{
		// What autoconnect does:
		// 1. Network scan
		// 2. Trigger association

		// Manual stuff:
		LOG_INF("Initiating DECT connection...");	
		err = conn_mgr_if_connect(dect_iface);
		if (err) {
			LOG_ERR("Failed to initiate connection: %d", err);
			return err;
		}

		// Block until connection established
		LOG_INF("Blocking until network joined");
		k_sem_take(&dect_network_joined_sem, K_FOREVER);

		struct dect_scan_params scan_params = 
		{
			.band = 0,
			.channel_count = 1,
			.channel_list = { 1657 },
			.channel_scan_time_ms = 500,
		};

		err = net_mgmt(NET_REQUEST_DECT_SCAN, dect_iface, &scan_params, sizeof(scan_params));
		if (err)
		{
			LOG_ERR("Failed to perform DECT scan: %d", err);
		}

		// Start own cluster
		start_cluster();
	}
	else
	{
		LOG_ERR("Invalid device type: 0x%08x", current_device_type);
	}

	while(1)
		k_sleep(K_SECONDS(1));

	return 0;
}