/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <string.h>
#include "jsmn.h"

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <helpers/nrfx_reset_reason.h>
#include <nrf_modem.h>

#include <sys/types.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/dfu/mcuboot.h>

#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

#include <dect_common.h>
#include <dect_common_settings.h>
#include <dect_phy_mac_common.h>
#include <dect_phy_mac_ctrl.h>
#include <dect_phy_mac_nbr.h>
#include "dect_app_time.h"
#include "dect_phy_mac_nbr_bg_scan.h"
#include "dect_phy_mac_nbr.h"
#include "dect_phy_mac_client.h"
#include "dect_phy_mac.h"
#include "dect_phy_ctrl.h"

#include <modem/nrf_modem_lib.h>
#include <modem/nrf_modem_lib_trace.h>

#include <dk_buttons_and_leds.h>

#if defined(CONFIG_DESH_STARTUP_CMDS)
#include "startup_cmd_ctrl.h"
#endif
#include "desh_defines.h"
#include "desh_print.h"

BUILD_ASSERT(IS_ENABLED(CONFIG_SHELL_BACKEND_SERIAL),
	     "CONFIG_SHELL_BACKEND_SERIAL shell backend must be enabled");

/***** Work queue and work item definitions *****/

#if defined(CONFIG_DESH_STARTUP_CMDS)
#define DESH_COMMON_WORKQUEUE_STACK_SIZE 4096
#define DESH_COMMON_WORKQ_PRIORITY 5
K_THREAD_STACK_DEFINE(desh_common_workq_stack, DESH_COMMON_WORKQUEUE_STACK_SIZE);
struct k_work_q desh_common_work_q;
#endif

/* Global variables */
const struct shell *desh_shell;

char desh_at_resp_buf[DESH_AT_CMD_RESPONSE_MAX_LEN];
K_MUTEX_DEFINE(desh_at_resp_buf_mutex);

static const char *modem_crash_reason_get(uint32_t reason)
{
	switch (reason) {
	case NRF_MODEM_FAULT_UNDEFINED:
		return "Undefined fault";

	case NRF_MODEM_FAULT_HW_WD_RESET:
		return "HW WD reset";

	case NRF_MODEM_FAULT_HARDFAULT:
		return "Hard fault";

	case NRF_MODEM_FAULT_MEM_MANAGE:
		return "Memory management fault";

	case NRF_MODEM_FAULT_BUS:
		return "Bus fault";

	case NRF_MODEM_FAULT_USAGE:
		return "Usage fault";

	case NRF_MODEM_FAULT_SECURE_RESET:
		return "Secure control reset";

	case NRF_MODEM_FAULT_PANIC_DOUBLE:
		return "Error handler crash";

	case NRF_MODEM_FAULT_PANIC_RESET_LOOP:
		return "Reset loop";

	case NRF_MODEM_FAULT_ASSERT:
		return "Assert";

	case NRF_MODEM_FAULT_PANIC:
		return "Unconditional SW reset";

	case NRF_MODEM_FAULT_FLASH_ERASE:
		return "Flash erase fault";

	case NRF_MODEM_FAULT_FLASH_WRITE:
		return "Flash write fault";

	case NRF_MODEM_FAULT_POFWARN:
		return "Undervoltage fault";

	case NRF_MODEM_FAULT_THWARN:
		return "Overtemperature fault";

	default:
		return "Unknown reason";
	}
}

void nrf_modem_fault_handler(struct nrf_modem_fault_info *fault_info)
{
	printk("Modem crash reason: 0x%x (%s), PC: 0x%x\n",
		fault_info->reason,
		modem_crash_reason_get(fault_info->reason),
		fault_info->program_counter);

	__ASSERT(false, "Modem crash detected, halting application execution");
}

// Global variables for high-level device information
struct dect_phy_mac_nbr_info_list_item *ptr_assoc_nbr = NULL;


// Scan for scan duration seconds. Whole thread sleeps for 2 * scan duration seconds
int scan_for_ft_beacons(uint32_t scan_duration_secs)
{
    desh_print("Starting beacon scan to find FT devices...");
    
	// Temporary static settings
    struct dect_phy_mac_beacon_scan_params params = {
        .duration_secs = scan_duration_secs,
        .channel = 0,  // 0 scans all channels
        .expected_rssi_level = 0,
        .clear_nbr_cache_before_scan = 0, // 1 means clearing the neighbor list before each scan
        .suspend_scheduler = 1,
    };
    
    int ret = dect_phy_mac_ctrl_beacon_scan_start(&params);
    if (ret) {
        desh_error("Cannot start beacon scan, err %d", ret);
        return ret;
    }
    
    desh_print("Beacon scan started. Waiting for results...");
    
    return 0;
}

// Loop through neighbors and store the best candidate to association pointer.
struct dect_phy_mac_nbr_info_list_item *find_best_association(struct dect_phy_mac_nbr_info_list_item *ptr_nbrs)
{
	// Currently only picks the best RSSI. Should also sort them based on above/below threshold and hop count
	struct dect_phy_mac_nbr_info_list_item *best_assoc_nbr = NULL;

	for (int i = 0; i < DECT_PHY_MAC_MAX_NEIGBORS; i++)
	{
		if (!(ptr_nbrs+i)->reserved) continue;

		if (!best_assoc_nbr)
		{
			best_assoc_nbr = ptr_nbrs + i;
			continue;
		}

		if ((ptr_nbrs+i)->rssi_2 > best_assoc_nbr->rssi_2)
			best_assoc_nbr = ptr_nbrs+i;
	}

	return best_assoc_nbr;
}

bool associate_with_ft(struct dect_phy_mac_nbr_info_list_item *ptr_assoc_nbr, uint16_t *ft_channel) // ft_channel must be pointer pointer
{
    desh_print("Attempting to associate with FT (Long RD ID = %u)...", ptr_assoc_nbr->long_rd_id);
    
    if (!ptr_assoc_nbr) {
        desh_error("FT not found in scan results");
        return -EINVAL;
    }
	
	uint32_t target_long_rd_id = ptr_assoc_nbr->long_rd_id;
	// *ft_channel = ptr_assoc_nbr->channel; // Store the channel for this association to increment for RDs next beacon scan
	// This line is buggy
    
    desh_print("FT, for association:");
    desh_print("  Long RD ID: %u", ptr_assoc_nbr->long_rd_id);
    desh_print("  Short RD ID: %u", ptr_assoc_nbr->short_rd_id);
    desh_print("  Network ID: %u (0x%08x)", ptr_assoc_nbr->nw_id_32bit, ptr_assoc_nbr->nw_id_32bit);
    desh_print("  Channel: %u", ptr_assoc_nbr->channel);
    
    // Prepare association parameters
    struct dect_phy_mac_associate_params assoc_params = {
        .tx_power_dbm = 0,
        .mcs = 0,  // MCS 0 is most robust
        .target_long_rd_id = target_long_rd_id,
    };
    
    // Send association request
    int ret = dect_phy_mac_ctrl_associate(&assoc_params);
    if (ret) {
        desh_error("Failed to send association request, err %d", ret);
        return ret;
    }
    
    desh_print("Association request sent. Waiting for response...");
    
    // Wait for association response (timeout is handled internally)
    k_sleep(K_SECONDS(15));
    
    // Check if we're now associated
    bool is_associated = dect_phy_mac_client_associated_by_target_short_rd_id(ptr_assoc_nbr->short_rd_id);
    
    if (is_associated)
	{
        desh_print("Successfully associated with FT (long RD ID): %d", ptr_assoc_nbr->long_rd_id);
        return true;
    }
	
	desh_error("Association failed or timed out");
	return false;
    
}

int send_data_to_ft(const char *data, struct dect_phy_mac_nbr_info_list_item *ptr_assoc_ft)
{
    if (ptr_assoc_ft == NULL) {
        desh_error("Not associated with any FT. Cannot send data.");
        return -ENODEV;
    }
    
    desh_print("Sending data to parent FT (long_rd_id=%u)...", ptr_assoc_ft->long_rd_id);
    
    // Prepare RACH (Random Access Channel) transmission parameters
    struct dect_phy_mac_rach_tx_params rach_params = {
        .target_long_rd_id = ptr_assoc_ft->long_rd_id,
        .tx_power_dbm = 0,
        .mcs = 0,
        .interval_secs = 0,  // 0 = send once, >0 = continuous with interval
        .get_mdm_temp = 0,   // KEEP 0. TEMPERATURE ALREADY SET!
    };
    
    // Copy data to send (max DECT_DATA_MAX_LEN bytes)
    strncpy(rach_params.tx_data_str, data, DECT_DATA_MAX_LEN - 1);
    rach_params.tx_data_str[DECT_DATA_MAX_LEN - 1] = '\0';
    
    // Send data via random access channel
    int ret = dect_phy_mac_ctrl_rach_tx_start(&rach_params);
    if (ret) {
        desh_error("Failed to send data, err %d", ret);
        return ret;
    }
    
    desh_print("Data transmission started: \"%s\"", data);
    
    return 0;
}

void relay_pt_message(dect_phy_mac_sdu_t sdu_data_item)
{
	// Get the data message and length
	uint16_t length = sdu_data_item.message.data_sdu.data_length;

	// Copy into string
	unsigned char u_rx_data[DECT_DATA_MAX_LEN];
	memcpy(u_rx_data, sdu_data_item.message.data_sdu.data, length);
	u_rx_data[length] = '\0';

	// Cast to regular char[]
	char *rx_data = (char *)(&u_rx_data);
	rx_data[DECT_DATA_MAX_LEN-1] = '\0';

	// Make sure assoication exists before parsing and sending!
	if (ptr_assoc_nbr == NULL)
	{
		desh_error("No associated FT. Not relaying message...");
		return;
	}

	// Use JSON parser to decide if this is supposed to be uplink message
	int i;
	int r;
	jsmn_parser p;
	jsmntok_t t[100];

	jsmn_init(&p);
	r = jsmn_parse(&p, rx_data, strlen(rx_data), t, sizeof(t) / sizeof(t[0]));

	if (r < 0 || (r < 1 || t[0].type != JSMN_OBJECT))
	{
		desh_error("Failed to parse incoming JSON data");
		return;
	}

	// Check if first field is message type
	if (jsoneq(rx_data, &t[1], "msg_type") != 0)
	{
		desh_error("Failed to read first field in incoming JSON!");
		return;
	}

	char *msg_type = strndup(rx_data + t[2].start, t[2].end - t[2].start);
	char *ft_uplink_str = "ft_uplink";

	// Check if message type is ft uplink
	if (strcmp(msg_type, ft_uplink_str) != 0)
	{
		desh_error("Message is not an FT uplink. Not relaying message...");
		return;
	}
	
	// After all cehcks, send data to FT
	int err = send_data_to_ft(rx_data, ptr_assoc_nbr);
	if (err) {
		desh_error("Failed to send data, err %d", err);
		return;
	}

	desh_print("Message successfully relayed!");
}


int main(void)
{
	int err;

	/* Configuration setup */
	desh_shell = shell_backend_uart_get_ptr();

	err = nrf_modem_lib_init();
	if (err) {
		/* Modem library initialization failed. */
		printk("Could not initialize nrf_modem_lib, err %d\n", err);
		printk("Fatal error\n");
		return 0;
	}

	#if defined(CONFIG_DK_LIBRARY)
		err = dk_leds_init();
		if (err) {
			printk("Cannot initialize LEDs (err: %d)\n", err);
		}
	#endif
	
	#if defined(CONFIG_DESH_STARTUP_CMDS)
		startup_cmd_ctrl_init();
	#endif

	/* Important structs for the running device */
	struct dect_phy_mac_nbr_info_list_item *ptr_nbrs = dect_phy_mac_nbr_info(); // Reference to neighbor list
	struct dect_phy_settings current_settings; // The device settings
	// struct dect_phy_mac_nbr_info_list_item *ptr_assoc_nbr = NULL;
	bool ftpt_mode = true;
	// int hop_count_ft = -1; // Additional settings (maybe make into a struct later)

	/* Read and write current settings */
	dect_common_settings_read(&current_settings);
	uint32_t long_rd_id = 1337; // THIS DEVICE LONG RD ID
	current_settings.common.transmitter_id = long_rd_id;
	dect_common_settings_write(&current_settings);

	/* Print current settings */
	desh_print("Common settings:");
	desh_print("  network id (32bit).............................%u (0x%08x)",
		   current_settings.common.network_id, current_settings.common.network_id);
	desh_print("  transmitter id (long RD ID)....................%u (0x%08x)",
		   current_settings.common.transmitter_id, current_settings.common.transmitter_id);
	desh_print("  short RD ID....................................%u (0x%04x)",
		   current_settings.common.short_rd_id, current_settings.common.short_rd_id);
	desh_print("  band number....................................%d",
		   current_settings.common.band_nbr);
	desh_print("\n");

	/* BEACON SCAN */
	// Maybe fix this setup here
	uint32_t scan_duration_channel = 2;
	uint32_t no_scans = 2;

	for(int i = 0; i < no_scans; i++)
	{
		bool rx_ongoing = dect_phy_ctrl_rx_is_ongoing();
		while (rx_ongoing == true)
		{
			rx_ongoing = dect_phy_ctrl_rx_is_ongoing();
			desh_print("RX ongoing. Sleeping before trying again...");
			k_sleep(K_SECONDS(1)); // Sleep for 1 second before checking for free RX
		}

		err = scan_for_ft_beacons(scan_duration_channel);

		if (err) 
		{
			desh_error("Failed to scan for FT beacons, err %d", err);
			return 0;
		}
	}

	bool rx_ongoing = dect_phy_ctrl_rx_is_ongoing();
	while (rx_ongoing == true)
	{
		rx_ongoing = dect_phy_ctrl_rx_is_ongoing();
		desh_print("RX ongoing. Sleeping before trying again..."); // TODO: Fjerne denne for den flooder terminalen
		k_sleep(K_SECONDS(1)); // Sleep for 1 second before checking for free RX
	}

	// Print the neighbor status before proceeding
	desh_print("\n=== Discovered FT Devices ===");
	bool device_found = false;

	for (int i = 0; i < DECT_PHY_MAC_MAX_NEIGBORS; i++) {
		if (!(ptr_nbrs+i)->reserved)
		{
			device_found = true;
			continue;
		} 
		
		desh_print("FT #%d:", i + 1);
		desh_print("  Long RD ID: %u", (ptr_nbrs + i)->long_rd_id);
		desh_print("  Short RD ID: %u", (ptr_nbrs + i)->short_rd_id);
		desh_print("  Channel: %u", (ptr_nbrs + i)->channel);
		desh_print("  RSSI-2: %d", ptr_nbrs->rssi_2); // RSSI-2 for last beacon received
	}

	if (!device_found) desh_error("No FT devices found! Retrying...");
	desh_print("=============================\n");

	k_sleep(K_SECONDS(2));

	/* ASSOCIATION */
	desh_print("=== Finding Best Association Neighbor ===");
	ptr_assoc_nbr = find_best_association(ptr_nbrs); // assoc_nbr now points to the associated neigbor in neighbor list

	if (!ptr_assoc_nbr)
	{
		desh_print("No association neighbors found!");
		goto end_of_life; // If no neighbors, no association, so we just skip. TODO: Go back to scanning
	}

	desh_print("Found best neighbor with long RD ID: %d", ptr_assoc_nbr->long_rd_id);

	uint16_t *current_assoc_channel = NULL;
	
	desh_print("\n=== Association Process ===");
	bool association_status = associate_with_ft(ptr_assoc_nbr, current_assoc_channel);
    if (!association_status) {
        desh_error("Association failed");
        return 0;
    }

	k_sleep(K_SECONDS(2));


	/* BEACON START */
	bool beacon_started = false;
	int beacon_tries = 4;

	for(int i = 0; i < beacon_tries; i++)
	{
		if(!beacon_started) {		
			struct dect_phy_mac_beacon_start_params params =
			{
				.tx_power_dbm = 0,
				.beacon_channel = 0,
			};
			int ret = dect_phy_mac_ctrl_cluster_beacon_start(&params);
			desh_print("Beacon returned: %d", ret);
			if (ret) {
				desh_print("Cannot start beacon, err %d", ret);
			} else {
				desh_print("Beacon starting");
				beacon_started = true;
			}
		}

		desh_print("Sleeping for 30 seconds...");
		k_sleep(K_SECONDS(30));
	}

	// TODO: Hvis ikke noen association requests --> Endre ftpt_mode til false

	// Temp løsning for å gjøre forskjell på PT og FTPT
	if (current_settings.common.transmitter_id == 4567) ftpt_mode = false;

	if (ftpt_mode == true)
	{
		register_rx_callback(relay_pt_message); // Registers the local function as callback for whenever device receive pt data
	}
	else
	{
		/* TRANSMIT DATA */
		int counter = 0;
		
		while (1)
		{
			// Get temperature
			int mdm_temperature = dect_phy_ctrl_modem_temperature_get();

			// String data to send
			char message[40];
			sprintf(message, "Hello from PT! Counter: %d", counter++);

			// Actual tx message to send
			char tx_message[DECT_DATA_MAX_LEN];
			if (mdm_temperature == NRF_MODEM_DECT_PHY_TEMP_NOT_MEASURED)
			{
				sprintf(tx_message,
					"{\"msg_type\":\"ft_uplink\","
					"\"transmitter_long_id\":%d,"
					"\"msg\":\"%s\","
					"\"m_tmp\":\"N/A\"}",
					current_settings.common.transmitter_id, message);
			}
			else
			{
				sprintf(tx_message,
					"{\"msg_type\":\"ft_uplink\","
					"\"transmitter_long_id\":%d,"
					"\"msg\":\"%s\","
					"\"m_tmp\":\"%d\"}",
					current_settings.common.transmitter_id, message, mdm_temperature);
			}

			err = send_data_to_ft(tx_message, ptr_assoc_nbr);
			if (err) {
				desh_error("Failed to send data, err %d", err);
				break;
			}
			
			k_sleep(K_SECONDS(10));  // Send every 10 seconds
			
			// Optional: Print status every few iterations
			if (counter % 5 == 0) {
				dect_phy_mac_client_status_print();
			}
		}
	}

end_of_life:
	desh_print("End of RD operation.");
	while(1)
	{
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
