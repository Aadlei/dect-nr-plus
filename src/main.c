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

#include <dect_common.h>
#include <dect_phy_mac_common.h>
#include <dect_phy_mac_ctrl.h>
#include <dect_phy_mac_nbr.h>
#include "dect_app_time.h"
#include "dect_phy_mac_nbr_bg_scan.h"

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

	/* Read RESETREAS register value and clear current reset reason(s). */
	reset_reason = nrfx_reset_reason_get();
	nrfx_reset_reason_clear(reset_reason);

	reset_reason_str_get(reset_reason_str, reset_reason);

	printk("\nReset reason: %s\n", reset_reason_str);
}

int main(void)
{
	int err;

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

	struct dect_phy_mac_nbr_info_list_item *ptr_nbrs = dect_phy_mac_nbr_info();

	struct dect_phy_mac_beacon_scan_params params = {
		.duration_secs = 4,
		.channel = 1665,
		.expected_rssi_level = 0,
		.clear_nbr_cache_before_scan = 1,
		.suspend_scheduler = 1, // Force this scan above other tasks
	};

	for (int i = 0; i < 5; i++)
	{
		int ret = dect_phy_mac_ctrl_beacon_scan_start(&params);
		if (ret)
		{
			desh_error("Cannot start beacon scan, err %d", ret);
		}
		else
		{
			desh_print("Beacon scan started.");
		}

		k_sleep(K_SECONDS(10));
	}

	desh_print("Stopping beacon.");
	dect_phy_mac_ctrl_cluster_beacon_stop(DECT_PHY_MAC_CTRL_BEACON_STOP_CAUSE_USER_INITIATED);

	k_sleep(K_SECONDS(10));

	while(1)
	{
		// PRINT NEIGHBOR LIST
		uint64_t time_now = dect_app_modem_time_now();
		desh_print("Neighbor list status:");
		for (int i = 0; i < DECT_PHY_MAC_MAX_NEIGBORS; i++) {
			struct dect_phy_mac_nbr_info_list_item current_neighbor = *(ptr_nbrs + i);

			if (current_neighbor.reserved == true) {
				int64_t time_from_last_received_ms = MODEM_TICKS_TO_MS(
						time_now - current_neighbor.time_rcvd_mdm_ticks);

				desh_print("  Neighbor %d:", i + 1);
				desh_print("   network ID (24bit MSB): %u (0x%06x)", current_neighbor.nw_id_24msb,
					current_neighbor.nw_id_24msb);
				desh_print("   network ID (8bit LSB):  %u (0x%02x)", current_neighbor.nw_id_8lsb,
					current_neighbor.nw_id_8lsb);
				desh_print("   network ID (32bit):     %u (0x%06x)", current_neighbor.nw_id_32bit,
					current_neighbor.nw_id_32bit);
				desh_print("   long RD ID:             %u", current_neighbor.long_rd_id);
				desh_print("   short RD ID:            %u", current_neighbor.short_rd_id);
				desh_print("   channel:                %u", current_neighbor.channel);
				desh_print("   Last seen:              %d msecs ago",
					time_from_last_received_ms);
				dect_phy_mac_nbr_bg_scan_status_print_for_target_long_rd_id(
					current_neighbor.long_rd_id);
			}
		}
		
		k_sleep(K_SECONDS(15));
	}

	return 0;
}
