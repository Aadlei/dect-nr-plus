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

/* Button crap */
// Button 1: 
// Button 2:
// Button 3:
// Button 4:

static const struct gpio_dt_spec button_1 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static const struct gpio_dt_spec button_2 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0});
static const struct gpio_dt_spec button_3 = GPIO_DT_SPEC_GET_OR(SW2_NODE, gpios, {0});
static const struct gpio_dt_spec button_4 = GPIO_DT_SPEC_GET_OR(SW3_NODE, gpios, {0});

static struct gpio_callback button_1_cb_data;
static struct gpio_callback button_2_cb_data;
static struct gpio_callback button_3_cb_data;
static struct gpio_callback button_4_cb_data;

static const struct gpio_dt_spec *ptr_buttons[] = { &button_1, &button_2, &button_3, &button_4 };
static struct gpio_callback *ptr_buttons_cb_data[] = { &button_1_cb_data, &button_2_cb_data, &button_3_cb_data, &button_4_cb_data };

void button_1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button 1 pressed\n");
}

void button_2_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button 2 pressed\n");
}

void button_3_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button 3 pressed\n");
}

void button_4_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button 4 pressed\n");
}

static const void (*ptr_buttons_pressed[])(const struct device *dev, struct gpio_callback *cb, uint32_t pins) =
{
	&button_1_pressed, &button_2_pressed, &button_3_pressed, &button_4_pressed
};

int main(void)
{
	int err;

	/* Buttons configuration */
	int ret;

	for (int i = 0; i < 4; i++)
	{
		if (!gpio_is_ready_dt(ptr_buttons[i]))
		{
			printk("Error: button device %s is not ready\n", *ptr_buttons[i]->port->name);
			return 0;
		}

		ret = gpio_pin_configure_dt(ptr_buttons[i], GPIO_INPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure %s pin %d\n",
				ret, *ptr_buttons[i]->port->name, ptr_buttons[i]->pin);
			return 0;
		}

		ret = gpio_pin_interrupt_configure_dt(ptr_buttons[i], GPIO_INT_EDGE_TO_ACTIVE);
		if (ret != 0) {
			printk("Error %d: failed to configure interrupt on %s pin %d\n",
				ret, *ptr_buttons[i]->port->name, ptr_buttons[i]->pin);
			return 0;
		}

		gpio_init_callback(ptr_buttons_cb_data[i], ptr_buttons_pressed[i], BIT(ptr_buttons[i]->pin));
		gpio_add_callback(ptr_buttons[i]->port, ptr_buttons_cb_data[i]);
		printk("Set up button at %s pin %d\n", ptr_buttons[i]->port->name, ptr_buttons[i]->pin);
	}

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


	/* Run beacon scan for 30 seconds, then stop */
	struct dect_phy_mac_beacon_scan_params params = {
		.duration_secs = 4,
		.channel = 1665,
		.expected_rssi_level = 0,
		.clear_nbr_cache_before_scan = 1,
		.suspend_scheduler = 1, // Force this scan above other tasks
	};
	
	for (int i = 0; i < 3; i++)
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


	/* Find the neighbor and send association request */
	if (ptr_nbrs->reserved == true) // Get the first neighbor if it is registered
	{
		int ret = 0;
		
		uint32_t nbr_long_rd_id = ptr_nbrs->long_rd_id;

		struct dect_phy_mac_associate_params params;
		params.tx_power_dbm = 0;
		params.mcs = 0;
		params.target_long_rd_id = nbr_long_rd_id;

		ret = dect_phy_mac_ctrl_associate(&params); // Send association request
		if (ret)
		{
			desh_error("Cannot send association request to PT %u "
				"a random access resource, err %d",
					params.target_long_rd_id, ret);
		}
		else
		{
			desh_print("Association request TX started.");
		}
	}
	else
	{
		desh_print("No neighbors found. Do not send association request.");
	}


	while(1)
	{
		/*
		// PRINT NEIGHBOR LIST
		uint64_t time_now = dect_app_modem_time_now();
		desh_print("Neighbor list status:");
		for (int i = 0; i < DECT_PHY_MAC_MAX_NEIGBORS; i++)
		{
			if ((ptr_nbrs + i)->reserved == true) {
				int64_t time_from_last_received_ms = MODEM_TICKS_TO_MS(
						time_now - (ptr_nbrs + i)->time_rcvd_mdm_ticks);

				desh_print("  Neighbor %d:", i + 1);
				desh_print("   network ID (24bit MSB): %u (0x%06x)", (ptr_nbrs + i)->nw_id_24msb,
					(ptr_nbrs + i)->nw_id_24msb);
				desh_print("   network ID (8bit LSB):  %u (0x%02x)", (ptr_nbrs + i)->nw_id_8lsb,
					(ptr_nbrs + i)->nw_id_8lsb);
				desh_print("   network ID (32bit):     %u (0x%06x)", (ptr_nbrs + i)->nw_id_32bit,
					(ptr_nbrs + i)->nw_id_32bit);
				desh_print("   long RD ID:             %u", (ptr_nbrs + i)->long_rd_id);
				desh_print("   short RD ID:            %u", (ptr_nbrs + i)->short_rd_id);
				desh_print("   channel:                %u", (ptr_nbrs + i)->channel);
				desh_print("   Last seen:              %d msecs ago",
					time_from_last_received_ms);
				dect_phy_mac_nbr_bg_scan_status_print_for_target_long_rd_id(
					(ptr_nbrs + i)->long_rd_id);
			}
		}
		
		k_sleep(K_SECONDS(15)); */
	}

	return 0;
}
