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

/* Button definition */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS_OKAY(SW0_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

#define SW1_NODE	DT_ALIAS(sw1)
#if !DT_NODE_HAS_STATUS_OKAY(SW1_NODE)
#error "Unsupported board: sw1 devicetree alias is not defined"
#endif

#define SW2_NODE	DT_ALIAS(sw2)
#if !DT_NODE_HAS_STATUS_OKAY(SW2_NODE)
#error "Unsupported board: sw2 devicetree alias is not defined"
#endif

#define SW3_NODE	DT_ALIAS(sw3)
#if !DT_NODE_HAS_STATUS_OKAY(SW3_NODE)
#error "Unsupported board: sw3 devicetree alias is not defined"
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

struct pt_association_info {
    bool is_associated;
    uint32_t ft_long_rd_id;
    uint16_t ft_short_rd_id;
    uint32_t network_id;
    uint16_t channel;
};

static struct pt_association_info my_association = {
    .is_associated = false,
    .ft_long_rd_id = 0,
    .ft_short_rd_id = 0,
    .network_id = 0,
    .channel = 0
};

int scan_for_ft_beacons(void)
{
    desh_print("Starting beacon scan to find FT devices...");
    
	// Temporary static settings
    struct dect_phy_mac_beacon_scan_params params = {
        .duration_secs = 10,
        .channel = 1665,  // Or 0 to scan all channels
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
    k_sleep(K_SECONDS(params.duration_secs + 2));
    
    return 0;
}

int associate_with_ft(uint32_t target_ft_long_rd_id)
{
    desh_print("Attempting to associate with FT (long_rd_id=%u)...", target_ft_long_rd_id);
    
    // Get the neighbor info from scan results
    struct dect_phy_mac_nbr_info_list_item *ft_info = 
        dect_phy_mac_nbr_info_get_by_long_rd_id(target_ft_long_rd_id);
    
    if (!ft_info) {
        desh_error("FT with long_rd_id=%u not found in scan results", target_ft_long_rd_id);
        return -EINVAL;
    }
    
    // Store FT information for later use
    my_association.ft_long_rd_id = ft_info->long_rd_id;
    my_association.ft_short_rd_id = ft_info->short_rd_id;
    my_association.network_id = ft_info->nw_id_32bit;
    my_association.channel = ft_info->channel;
    
    desh_print("FT found:");
    desh_print("  Long RD ID: %u", ft_info->long_rd_id);
    desh_print("  Short RD ID: %u", ft_info->short_rd_id);
    desh_print("  Network ID: %u (0x%08x)", ft_info->nw_id_32bit, ft_info->nw_id_32bit);
    desh_print("  Channel: %u", ft_info->channel);
    
    // Prepare association parameters
    struct dect_phy_mac_associate_params assoc_params = {
        .tx_power_dbm = 0,
        .mcs = 0,  // MCS 0 is most robust
        .target_long_rd_id = target_ft_long_rd_id,
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
    bool is_associated = dect_phy_mac_client_associated_by_target_short_rd_id(
        my_association.ft_short_rd_id);
    
    if (is_associated) {
        my_association.is_associated = true;
        desh_print("Successfully associated with FT!");
        return 0;
    } else {
        desh_error("Association failed or timed out");
        my_association.is_associated = false;
        return -ETIMEDOUT;
    }
}

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

void print_association_status(void)
{
    desh_print("\n=== PT Association Status ===");
    if (my_association.is_associated) {
        desh_print("Status: ASSOCIATED");
        desh_print("FT Long RD ID: %u", my_association.ft_long_rd_id);
        desh_print("FT Short RD ID: %u", my_association.ft_short_rd_id);
        desh_print("Network ID: %u (0x%08x)", 
                   my_association.network_id, my_association.network_id);
        desh_print("Channel: %u", my_association.channel);
    } else {
        desh_print("Status: NOT ASSOCIATED");
    }
    desh_print("============================\n");
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
	struct dect_phy_mac_nbr_info_list_item *ptr_nbrs = dect_phy_mac_nbr_info(); // Neighbor list
	struct dect_phy_settings current_settings; // The device settings


	/* Read and write current settings */
	dect_common_settings_read(&current_settings);
	uint32_t long_rd_id = 9876; // Just a random value
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


	err = scan_for_ft_beacons();
	if (err) 
	{
		desh_error("Failed to scan for FT beacons, err %d", err);
		return 0;
	} 

	uint32_t target_ft_long_rd_id = 0;

	// While no FT found, keep scanning
	while(target_ft_long_rd_id == 0) {
		desh_print("\n=== Discovered FT Devices ===");
			for (int i = 0; i < DECT_PHY_MAC_MAX_NEIGBORS; i++) {
				if (!(ptr_nbrs + i)->reserved) continue; 
				
				desh_print("FT #%d:", i + 1);
				desh_print("  Long RD ID: %u", (ptr_nbrs + i)->long_rd_id);
				desh_print("  Short RD ID: %u", (ptr_nbrs + i)->short_rd_id);
				desh_print("  Channel: %u", (ptr_nbrs + i)->channel);
				
				// Use first discovered FT
				if (target_ft_long_rd_id == 0) {
					target_ft_long_rd_id = (ptr_nbrs + i)->long_rd_id;
				}
				
			}
			desh_print("=============================\n");
		if (target_ft_long_rd_id == 0) {
        	desh_error("No FT devices found! Retrying...");
			k_sleep(K_SECONDS(10));
    	}
	}

	k_sleep(K_SECONDS(2));
    
	err = associate_with_ft(target_ft_long_rd_id);
    if (err) {
        desh_error("Association failed");
        return 0;
    }

	print_association_status();

	k_sleep(K_SECONDS(2));

	/* Transmit data */

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
    }
	return 0;
}
