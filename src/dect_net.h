#ifndef DECT_NET_H
#define DECT_NET_H

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/posix/sys/socket.h>
#include <stdint.h>
#include <stdbool.h>

/* Device identities — single source of truth for the whole project */
#define DECT_EDGE_PT_LONG_RD_ID     0xAAAAAAAAU

#define DECT_FT_LONG_RD_ID          0xBBBBBBBBU
#define DECT_PT_LONG_RD_ID          0xCCCCCCCCU

//#define DECT_FT_LONG_RD_ID          0xDDDDDDDDU
//#define DECT_PT_LONG_RD_ID          0xEEEEEEEEU

#define DECT_SINK_LONG_RD_ID        0x67FFFFFFU
#define MESH_PREFIX_STR           "fd12:3456:789a"

/* Must be called before any other dect_net function */
void dect_net_init(struct net_if *iface);

/* Settings */
void dect_net_write_ft_settings(void);
void dect_net_write_pt_settings(void);

/* Network operations */
void dect_net_start_beacon(void);
void dect_net_start_scan(void);
void dect_net_consider_ft(uint32_t long_rd_id, uint8_t route_cost);
bool dect_net_has_best_ft(void);
void dect_net_join_best(void);

/* Queries */
uint32_t             dect_net_get_current_long_rd_id(void);
const struct in6_addr *dect_net_get_mesh_prefix(void);
uint32_t             dect_net_get_parent_long_rd_id(void);
bool                 dect_net_create_ipv6(struct in6_addr *addr, uint32_t long_rd_id);

/* RSSI — update called from NET_EVENT_DECT_NEIGHBOR_INFO handler */
void   dect_net_update_rssi(uint32_t long_rd_id, int8_t rssi);
int8_t dect_net_get_rx_rssi(struct sockaddr_in6 *src_addr);

#endif /* DECT_NET_H */