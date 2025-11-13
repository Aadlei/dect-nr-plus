/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef DECT_PHY_MAC_H
#define DECT_PHY_MAC_H

#include <zephyr/kernel.h>
#include "dect_common.h"
#include "dect_phy_common.h"

/******************************************************************************/

typedef void (*dect_phy_mac_data_received_cb_t)(const uint8_t *data, uint32_t length);

void dect_phy_mac_register_data_callback(dect_phy_mac_data_received_cb_t callback);

bool dect_phy_mac_handle(struct dect_phy_commmon_op_pdc_rcv_params *rcv_params);

bool dect_phy_mac_direct_pdc_handle(struct dect_phy_commmon_op_pdc_rcv_params *rcv_params);

/******************************************************************************/

#endif /* DECT_PHY_MAC_H */
