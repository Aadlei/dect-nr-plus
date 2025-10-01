#ifndef DECT_COMMON_H
#define DECT_COMMON_H
#include <zephyr/kernel.h>


/******************************************************************************/

/**
 * Physical Layer Control Field common part.
 *
 * The part of the Physical Layer Control Field that is common between all its variants.
 * See Section 6.2 of [2].
 */
struct phy_ctrl_field_common {
	uint32_t packet_length : 4;
	uint32_t packet_length_type : 1;
	uint32_t header_format : 3;
	uint32_t short_network_id : 8;
	uint32_t transmitter_id_hi : 8;
	uint32_t transmitter_id_lo : 8;
	uint32_t df_mcs : 3;
	uint32_t reserved : 1;
	uint32_t transmit_power : 4;
	uint32_t pad : 24;
};

/**
 * Device type enumeration.
 * FT, PT - The node is acting as a router, but also a client to another router.
 * PT - The node acts as a client on the network, and has no other nodes connected to it (other than the upstream node)
 * FT - The node is acting as a router and has PT nodes connected to it
 */
enum device_type {
	ft_pt = 0,  // Fixed Termination and Portable Termination
	pt = 1,		// Portable Termination
	ft = 2		// Fixed Termination
};












/******************************************************************************/

typedef struct {
	/** @brief Network ID (24 bits), ETSI TS 103 636-4 4.2.3.1 */
	uint32_t network_id;

	/** @brief Transmitter Long Radio ID, ETSI TS 103 636-4 4.2.3.2 */
	uint32_t transmitter_long_rd_id;

	/** @brief Receiver Long Radio ID, ETSI TS 103 636-4 4.2.3.2 */
	uint32_t receiver_long_rd_id;

} dect_mac_address_info_t;


/************************************************************************************************/

/* Supported DECT bands. See ETSI TS 103 636-2 v1.3.1 Table 5.4.2-1. (3rd column) */

#define DECT_PHY_SUPPORTED_CHANNEL_BAND1_MIN 1657
#define DECT_PHY_SUPPORTED_CHANNEL_BAND1_MAX 1677

#define DECT_PHY_SUPPORTED_CHANNEL_BAND2_MIN 1680
#define DECT_PHY_SUPPORTED_CHANNEL_BAND2_MAX 1700

#define DECT_PHY_SUPPORTED_CHANNEL_BAND4_MIN 525
#define DECT_PHY_SUPPORTED_CHANNEL_BAND4_MAX 551

#define DECT_PHY_SUPPORTED_CHANNEL_BAND9_MIN 1703
#define DECT_PHY_SUPPORTED_CHANNEL_BAND9_MAX 1711

#define DECT_PHY_SUPPORTED_CHANNEL_BAND22_MIN 1691
#define DECT_PHY_SUPPORTED_CHANNEL_BAND22_MAX 1711

#define DESH_DECT_PHY_SUPPORTED_BAND_COUNT 5

/************************************************************************************************/

#endif /* DECT_COMMON_H */
