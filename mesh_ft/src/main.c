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

#include <dect_phy_mac.h>

#include <modem/nrf_modem_lib.h>
#include <modem/nrf_modem_lib_trace.h>

#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/hwinfo.h>

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

static const struct gpio_dt_spec button_1 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec button_2 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec button_3 = GPIO_DT_SPEC_GET(SW2_NODE, gpios);
static const struct gpio_dt_spec button_4 = GPIO_DT_SPEC_GET(SW3_NODE, gpios);

static struct gpio_callback button_1_cb_data;
static struct gpio_callback button_2_cb_data;
static struct gpio_callback button_3_cb_data;
static struct gpio_callback button_4_cb_data;


static const struct gpio_dt_spec *ptr_buttons[] = { &button_1, &button_2, &button_3, &button_4 };
static struct gpio_callback *ptr_buttons_cb_data[] = { &button_1_cb_data, &button_2_cb_data, &button_3_cb_data, &button_4_cb_data };


/* Global variables */
const struct shell *desh_shell;
static uint16_t device_id;
bool beacon_started = false;

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
static struct k_work button_1_work;

static void button_1_work_handler(struct k_work *work)
{
    desh_print("dect-phy-mac status:");
    dect_phy_mac_cluster_beacon_status_print();
    dect_phy_mac_client_status_print();
    dect_phy_mac_nbr_status_print();
}

void button_1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_work_submit(&button_1_work);
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

static void data_received_handler(const uint8_t *data, uint32_t length)
{
    char received_str[DECT_DATA_MAX_LEN];
    
    // Copy to null-terminated string
    uint32_t copy_len = MIN(length, DECT_DATA_MAX_LEN - 1);
    memcpy(received_str, data, copy_len);
    received_str[copy_len] = '\0';
    
    printk("\n*** CALLBACK: Received data ***\n");
    printk("Length: %u\n", length);
    printk("Data: %s\n", received_str);
    
    // Parse JSON if needed
    char *data_start = strstr(received_str, "\"data\":\"");
    if (data_start) {
        data_start += 8; // Skip past "data":"
        char *data_end = strchr(data_start, '"');
        if (data_end) {
            *data_end = '\0';
            printk("Parsed message: %s\n", data_start);
        }
    }
    
    char *temp_start = strstr(received_str, "\"m_tmp\":\"");
    if (temp_start) {
        temp_start += 9; // Skip past "m_tmp":"
        char *temp_end = strchr(temp_start, '"');
        if (temp_end) {
            *temp_end = '\0';
            printk("Temperature: %s°C\n", temp_start);
        }
    }
    printk("*******************************\n\n");
}

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

	k_work_init(&button_1_work, button_1_work_handler);

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
	hwinfo_get_device_id((void *)&device_id, sizeof(device_id));
	printk("DECT NR+ Started. Device id: %u\n", device_id);

	
	struct dect_phy_settings current_settings; // The device settings
	dect_common_settings_read(&current_settings);
	printk("Current transmitter id (long RD ID): %u\n",
	       current_settings.common.transmitter_id);
	printk("Current band number: %d\n", current_settings.common.band_nbr);
	printk("Network id: %u\n", current_settings.common.network_id);
  
	while(1) {
		if(!beacon_started) {		
			struct dect_phy_mac_beacon_start_params params = {
				.tx_power_dbm = 0,
				.beacon_channel = 1665,
			};
			int ret = dect_phy_mac_ctrl_cluster_beacon_start(&params);
			printk("Beacon returned: %d\n", ret);
			if (ret) {
				printk("Cannot start beacon, err %d", ret);
			} else {
				printk("Beacon starting");
				beacon_started = true;
				dect_phy_mac_register_data_callback(data_received_handler);
			}
		}

		printk("Sleeping for 30 seconds...\n");
		k_sleep(K_SECONDS(30));
	}

	return 0;
}
