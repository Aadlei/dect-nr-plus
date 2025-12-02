/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <string.h>

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

/*
static void reset_reason_str_get(char *str, uint32_t reason)
{
	size_t len;

	*str = '\0';

	if (reason & NRFX_RESET_REASON_RESETPIN_MASK) {
		(void)strcat(str, "PIN reset | ");
	}
	if (reason & NRFX_RESET_REASON_DOG_MASK) {
		(void)strcat(str, "watchdog | ");
	}
	if (reason & NRFX_RESET_REASON_OFF_MASK) {
		(void)strcat(str, "wakeup from power-off | ");
	}
	if (reason & NRFX_RESET_REASON_DIF_MASK) {
		(void)strcat(str, "debug interface wakeup | ");
	}
	if (reason & NRFX_RESET_REASON_SREQ_MASK) {
		(void)strcat(str, "software | ");
	}
	if (reason & NRFX_RESET_REASON_LOCKUP_MASK) {
		(void)strcat(str, "CPU lockup | ");
	}
	if (reason & NRFX_RESET_REASON_CTRLAP_MASK) {
		(void)strcat(str, "control access port | ");
	}

	len = strlen(str);
	if (len == 0) {
		(void)strcpy(str, "power-on reset");
	} else {
		str[len - 3] = '\0';
	}
}

static void desh_print_reset_reason(void)
{
	uint32_t reset_reason;
	char reset_reason_str[128];

	// Read RESETREAS register value and clear current reset reason(s).
	reset_reason = nrfx_reset_reason_get();
	nrfx_reset_reason_clear(reset_reason);

	reset_reason_str_get(reset_reason_str, reset_reason);

	printk("\nReset reason: %s\n", reset_reason_str);
}
*/

enum band_1_channels
{
	c1657,
	c1659,
	c1661,
	c1663,
	c1665,
	c1667,
	c1669,
	c1671,
	c1673,
	c1675,
	c1677,
	no_channels
};

// Scan for scan duration seconds. Whole thread sleeps for 2 * scan duration seconds
int scan_for_ft_beacons(uint32_t scan_duration_secs)
{
    desh_print("Starting beacon scan to find FT devices...");
    
	// Temporary static settings
    struct dect_phy_mac_beacon_scan_params params = {
        .duration_secs = scan_duration_secs,
        .channel = 0,  // Or 0 to scan all channels
        .expected_rssi_level = 0,
        .clear_nbr_cache_before_scan = 1,
        .suspend_scheduler = 1,
    };
    
    int ret = dect_phy_mac_ctrl_beacon_scan_start(&params);
    if (ret) {
        desh_error("Cannot start beacon scan, err %d", ret);
        return ret;
    }
    
    desh_print("Beacon scan started. Waiting for results...");
    
    // Wait for scan to complete
    //k_sleep(K_SECONDS(params.duration_secs * 2));
    
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

bool associate_with_ft(struct dect_phy_mac_nbr_info_list_item *assoc_nbr, uint16_t *ft_channel) // ft_channel must be pointer pointer
{
    desh_print("Attempting to associate with FT (Long RD ID = %u)...", assoc_nbr->long_rd_id);
    
    if (!assoc_nbr) {
        desh_error("FT not found in scan results");
        return -EINVAL;
    }
	
	uint32_t target_long_rd_id = assoc_nbr->long_rd_id;
	// *ft_channel = assoc_nbr->channel; // Store the channel for this association to increment for RDs next beacon scan
	// This line is buggy
    
    desh_print("FT, for association:");
    desh_print("  Long RD ID: %u", assoc_nbr->long_rd_id);
    desh_print("  Short RD ID: %u", assoc_nbr->short_rd_id);
    desh_print("  Network ID: %u (0x%08x)", assoc_nbr->nw_id_32bit, assoc_nbr->nw_id_32bit);
    desh_print("  Channel: %u", assoc_nbr->channel);
    
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
    bool is_associated = dect_phy_mac_client_associated_by_target_short_rd_id(assoc_nbr->short_rd_id);
    
    if (is_associated)
	{
        desh_print("Successfully associated with FT!");
        return true;
    }
	
	desh_error("Association failed or timed out");
	return false;
    
}
/*
int send_data_to_ft(const char *data)
{
    if (!my_association.is_associated) {
        desh_error("Not associated with any FT. Cannot send data.");
        return -ENOTCONN;
    }
    
    desh_print("Sending data to FT (long_rd_id=%u)...", my_association.ft_long_rd_id);
    
    // Get fresh neighbor info (beacon might have updated)
    struct dect_phy_mac_nbr_info_list_item *ft_info = 
        dect_phy_mac_nbr_info_get_by_long_rd_id(my_association.ft_long_rd_id);
    
    if (!ft_info) {
        desh_error("FT no longer in neighbor list. Re-scan needed.");
        my_association.is_associated = false;
        return -ENODEV;
    }
    
    // Prepare RACH (Random Access Channel) transmission parameters
    struct dect_phy_mac_rach_tx_params rach_params = {
        .target_long_rd_id = my_association.ft_long_rd_id,
        .tx_power_dbm = 0,
        .mcs = 0,
        .interval_secs = 0,  // 0 = send once, >0 = continuous with interval
        .get_mdm_temp = 1,   // Set to 1 if you want to include modem temp in data
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
*/

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
	// int hop_count_ft = -1; // Additional settings (maybe make into a struct later)

	struct dect_phy_mac_nbr_info_list_item *ptr_assoc_nbr = NULL;
	bool is_ft_associated = false;

	/* Read and write current settings */
	dect_common_settings_read(&current_settings);
	uint32_t long_rd_id = 4567; // THIS DEVICE LONG RD ID
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
	uint32_t no_channels_in_band = 11;
	uint32_t scan_duration_channel = 1;
	uint32_t scan_duration_all = 
		no_channels_in_band * scan_duration_channel;
	uint32_t no_scans = 5;

	for(int i = 0; i < no_scans; i++)
	{
		err = scan_for_ft_beacons(scan_duration_channel);
		k_sleep(K_SECONDS(scan_duration_all * 2));

		if (err) 
		{
			desh_error("Failed to scan for FT beacons, err %d", err);
			return 0;
		}

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
	}

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

	// print_association_status();

	k_sleep(K_SECONDS(2));


	/* BEACON START */
	bool beacon_started = false;
	int beacon_tries = 5;

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


	/* TRANSMIT DATA */
	/*
	int counter = 0;
	char message[DECT_DATA_MAX_LEN];

	while (1) {
        // Send single message
        snprintf(message, sizeof(message), "Hello from PT! Counter: %d", counter++);
        err = send_data_to_ft(message);
        
        if (err) {
            desh_error("Failed to send data. Re-scanning...");
            
            // Try to re-associate
            scan_for_ft_beacons();
            k_sleep(K_SECONDS(2));
            associate_with_ft(target_ft_long_rd_id);
        }
        
        k_sleep(K_SECONDS(10));  // Send every 10 seconds
        
        // Optional: Print status every few iterations
        if (counter % 5 == 0) {
            print_association_status();
            dect_phy_mac_client_status_print();
        }
    }*/

end_of_life:
	desh_print("End of RD operation.");
	while(1)
	{
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
