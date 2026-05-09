#ifndef SYNC_H
#define SYNC_H

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <stdint.h>

#define SYNC_PORT 12346

/*
 * sync_init - store the DECT interface, mesh prefix, and sink long RD ID
 * needed to construct IPv6 addresses for SYNC peers.
 *
 * Must be called before sync_open_socket or either operation.
 */
void sync_init(struct net_if *iface, const struct in6_addr *mesh_prefix,
               uint32_t sink_long_rd_id);

/* Open/close the dedicated SYNC UDP socket (driven by IF_UP / IF_DOWN). */
int  sync_open_socket(void);
void sync_close_socket(void);

/*
 * sync_pt_operation - PT side: send T0, receive T1/T2, compute clock offset.
 *
 * @param parent_long_rd_id  Long RD ID of the parent FT to synchronise with.
 * @param offset_out         Receives the computed PT-to-FT clock offset (ms).
 *
 * Returns 0 on success, negative on failure or timeout.
 */
int sync_pt_operation(uint32_t parent_long_rd_id, int32_t *offset_out);

/*
 * sync_ft_operation - FT side: receive T0 from child PT, echo T1/T2 back.
 *
 * @param child_long_rd_id  Long RD ID of the expected child PT.
 *
 * Returns 0 on success, negative on failure or timeout.
 */
int sync_ft_operation(uint32_t child_long_rd_id);

#endif /* SYNC_H */