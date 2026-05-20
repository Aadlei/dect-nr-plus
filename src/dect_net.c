#include "dect_net.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/posix/arpa/inet.h>

#include <net/dect/dect_net_l2_mgmt.h>
#include <net/dect/dect_net_l2.h>
#include <net/dect/dect_utils.h>

#include "uart.h"   /* ROUTING_MAX_HOPS */

LOG_MODULE_REGISTER(dect_net, LOG_LEVEL_INF);

#define RSSI_CACHE_SIZE ROUTING_MAX_HOPS

static struct net_if   *dect_iface;
static struct in6_addr  mesh_prefix;
static uint32_t         current_long_rd_id;
static uint32_t         best_long_rd_id;
static uint8_t          best_route_cost;

static struct {
    uint32_t long_rd_id;
    int8_t   rssi;
} rssi_cache[RSSI_CACHE_SIZE];

/* ── Internal helpers ── */

static bool create_ipv6_internal(struct in6_addr *addr, uint32_t long_rd_id)
{
    bool ok = dect_utils_lib_net_ipv6_addr_create_from_sink_and_long_rd_id(
        mesh_prefix, DECT_SINK_LONG_RD_ID, long_rd_id, addr);
    if (!ok) LOG_ERR("Failed to create IPv6 address for 0x%08x", long_rd_id);
    return ok;
}

static void create_and_set_device_ipv6(void)
{
    net_addr_pton(AF_INET6, MESH_PREFIX_STR, &mesh_prefix);

    struct in6_addr global_addr;
    if (!create_ipv6_internal(&global_addr, current_long_rd_id)) return;

    net_if_ipv6_addr_add(dect_iface, &global_addr, NET_ADDR_MANUAL, 0);

    char addr_str[NET_IPV6_ADDR_LEN];
    net_addr_ntop(AF_INET6, &global_addr, addr_str, sizeof(addr_str));
    LOG_INF("Adding global IPv6: %s", addr_str);
}

/* ── Public API ── */

void dect_net_init(struct net_if *iface)
{
    dect_iface      = iface;
    best_long_rd_id = 0;
    best_route_cost = 0xFF;
}

void dect_net_write_ft_settings(void)
{
    struct dect_settings s = {0};
    int ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_READ, dect_iface, &s, sizeof(s));
    if (ret) {
        LOG_ERR("Failed to read settings: %d", ret);
        return;
    }

    s.device_type = DECT_DEVICE_TYPE_FT;
#if IS_ENABLED(CONFIG_DECT_RELAY_FT)
    s.identities.transmitter_long_rd_id = DECT_FT_LONG_RD_ID;
#else
    s.identities.transmitter_long_rd_id = DECT_SINK_LONG_RD_ID;
#endif

    // Disable auto-start
    s.auto_start.activate = false;

    s.nw_beacon.channel       = 1657;
    s.nw_beacon.beacon_period = DECT_NW_BEACON_PERIOD_1000MS;
    s.cmd_params.write_scope_bitmap =
        DECT_SETTINGS_WRITE_SCOPE_AUTO_START  |
        DECT_SETTINGS_WRITE_SCOPE_DEVICE_TYPE |
        DECT_SETTINGS_WRITE_SCOPE_IDENTITIES  |
        DECT_SETTINGS_WRITE_SCOPE_NW_BEACON;

    ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_WRITE, dect_iface, &s, sizeof(s));
    if (ret) {
        LOG_ERR("Failed to write settings: %d", ret);
        return;
    }

    current_long_rd_id = s.identities.transmitter_long_rd_id;
    create_and_set_device_ipv6();
    LOG_INF("FT settings written (long_rd_id=0x%08x)", current_long_rd_id);
}

void dect_net_write_pt_settings(void)
{
    struct dect_settings s = {0};
    int ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_READ, dect_iface, &s, sizeof(s));
    if (ret) {
        LOG_ERR("Failed to read settings: %d", ret);
        return; 
    }

    s.device_type = DECT_DEVICE_TYPE_PT;
#if IS_ENABLED(CONFIG_DECT_RELAY_PT)
    s.identities.transmitter_long_rd_id = DECT_PT_LONG_RD_ID;
#else
    s.identities.transmitter_long_rd_id = DECT_EDGE_PT_LONG_RD_ID;
#endif

    // Disable auto-start
    s.auto_start.activate = false;

    s.cmd_params.write_scope_bitmap =
        DECT_SETTINGS_WRITE_SCOPE_AUTO_START    |
        DECT_SETTINGS_WRITE_SCOPE_DEVICE_TYPE   |
        DECT_SETTINGS_WRITE_SCOPE_IDENTITIES;

    ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_WRITE, dect_iface, &s, sizeof(s));
    if (ret) {
        LOG_ERR("Failed to write settings: %d", ret);
        return;
    }

    current_long_rd_id = s.identities.transmitter_long_rd_id;
    create_and_set_device_ipv6();
    LOG_INF("PT settings written (long_rd_id=0x%08x)", current_long_rd_id);
}

void dect_net_start_beacon(void)
{
    struct dect_nw_beacon_start_req_params p = {
        .channel = 1657,
        .additional_ch_count = 0,
    };
    int ret = net_mgmt(NET_REQUEST_DECT_NW_BEACON_START, dect_iface, &p, sizeof(p));
    if (ret) LOG_ERR("Beacon start failed: %d", ret);
}

void dect_net_start_scan(void)
{
    best_long_rd_id = 0;
    best_route_cost = 0xFF;

    struct dect_scan_params p = {
        .band = 1,
        .channel_count = 0,
        .channel_scan_time_ms = 1200,
    };
    int ret = net_mgmt(NET_REQUEST_DECT_SCAN, dect_iface, &p, sizeof(p)); // Callback to NET_EVENT_DECT_SCAN_RESULT and NET_EVENT_DECT_SCAN_DONE
    if (ret) LOG_ERR("Scan start failed: %d", ret);
}

void dect_net_consider_ft(uint32_t long_rd_id, uint8_t route_cost)
{
    if (route_cost < best_route_cost) {
        best_route_cost = route_cost;
        best_long_rd_id = long_rd_id;
        LOG_INF("New best FT: 0x%08x (cost=%d)", best_long_rd_id, best_route_cost);
    }
}

bool dect_net_has_best_ft(void)
{
    return best_long_rd_id != 0;
}

void dect_net_join_best(void)
{
    struct dect_settings s = {0};
    int ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_READ, dect_iface, &s, sizeof(s));
    if (ret) { LOG_ERR("Failed to read settings: %d", ret); return; }

    s.network_join.target_ft_long_rd_id = best_long_rd_id;
    s.cmd_params.write_scope_bitmap |= DECT_SETTINGS_WRITE_SCOPE_NETWORK_JOIN;

    ret = net_mgmt(NET_REQUEST_DECT_SETTINGS_WRITE, dect_iface, &s, sizeof(s));
    if (ret) { LOG_ERR("Failed to write join settings: %d", ret); return; }

    ret = net_mgmt(NET_REQUEST_DECT_NETWORK_JOIN, dect_iface, NULL, 0); // Callback to NET_EVENT_DECT_NETWORK_STATUS->Joined
    if (ret) LOG_ERR("Network join failed: %d", ret);
    else     LOG_INF("Joining FT 0x%08x", best_long_rd_id);
}

uint32_t dect_net_get_current_long_rd_id(void)
{
    return current_long_rd_id;
}

const struct in6_addr *dect_net_get_mesh_prefix(void)
{
    return &mesh_prefix;
}

uint32_t dect_net_get_parent_long_rd_id(void)
{
    struct dect_status_info info = {0};
    int ret = net_mgmt(NET_REQUEST_DECT_STATUS_INFO_GET, dect_iface, &info, sizeof(info));
    if (ret) { LOG_ERR("Failed to get status info: %d", ret); return 0; }
    if (info.parent_count == 0) return 0;
    return info.parent_associations->long_rd_id;
}

bool dect_net_create_ipv6(struct in6_addr *addr, uint32_t long_rd_id)
{
    return create_ipv6_internal(addr, long_rd_id);
}

void dect_net_update_rssi(uint32_t long_rd_id, int8_t rssi)
{
    for (int i = 0; i < RSSI_CACHE_SIZE; i++) {
        if (rssi_cache[i].long_rd_id == long_rd_id || rssi_cache[i].long_rd_id == 0) {
            rssi_cache[i].long_rd_id = long_rd_id;
            rssi_cache[i].rssi       = rssi;
            return;
        }
    }
    /* Cache full - overwrite slot 0 */
    rssi_cache[0].long_rd_id = long_rd_id;
    rssi_cache[0].rssi       = rssi;
}

int8_t dect_net_get_rx_rssi(struct sockaddr_in6 *src_addr)
{
    uint32_t long_rd_id = dect_utils_lib_long_rd_id_from_ipv6_addr(&src_addr->sin6_addr);
    if (long_rd_id == 0) return 0;

    struct dect_neighbor_info_req_params p = { .long_rd_id = long_rd_id };
    int ret = net_mgmt(NET_REQUEST_DECT_NEIGHBOR_INFO, dect_iface, &p, sizeof(p));
    if (ret) LOG_WRN("Neighbor info request failed: %d", ret);

    for (int i = 0; i < RSSI_CACHE_SIZE; i++) {
        if (rssi_cache[i].long_rd_id == long_rd_id) return rssi_cache[i].rssi;
    }
    return 0;
}