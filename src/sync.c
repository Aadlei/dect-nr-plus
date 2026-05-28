#include "sync.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/arpa/inet.h>

#include <net/dect/dect_utils.h>

LOG_MODULE_REGISTER(sync, LOG_LEVEL_INF);

#define SYNC_MAGIC_SIGNATURE    0xFEFDU     // The G.O.A.T
#define SYNC_T_SIZE             4
#define SOCKET_SYNC_RX_TIMEOUT_SEC  10
#define SYNC_MAX_RX_RETRIES     5

struct sync_data {
    uint32_t magic_signature;
    uint32_t T[SYNC_T_SIZE];
};

/* Module state set by sync_init() */
static struct net_if   *sync_iface;
static struct in6_addr  sync_mesh_prefix;
static uint32_t         sync_sink_long_rd_id;

static int sync_socket = -1;

/* ── Internal helpers ── */

static bool make_peer_addr(struct sockaddr_in6 *addr, uint32_t long_rd_id)
{
    bool ok = dect_utils_lib_net_ipv6_addr_create_from_sink_and_long_rd_id(
        sync_mesh_prefix, sync_sink_long_rd_id, long_rd_id, &addr->sin6_addr);
    if (!ok) {
        LOG_ERR("Failed to construct IPv6 address for 0x%08x", long_rd_id);
    }
    return ok;
}

static int recv_sync_from_peer(struct sync_data *out, uint32_t *rx_long_rd_id_out,
                               uint32_t *timestamp_out, struct sockaddr_in6 *src_addr_out)
{
    struct sockaddr_in6 src_addr;
    socklen_t addr_len = sizeof(src_addr);
    char addr_str[NET_IPV6_ADDR_LEN];

    int ret = recvfrom(sync_socket, out, sizeof(*out), 0,
                       (struct sockaddr *)&src_addr, &addr_len);
    if (ret < 0) return -errno;

    *timestamp_out = k_uptime_get_32();

    if (out->magic_signature != SYNC_MAGIC_SIGNATURE) {
        LOG_WRN("SYNC magic mismatch (got 0x%08x)", out->magic_signature);
        return -EBADMSG;
    }

    net_addr_ntop(AF_INET6, &src_addr.sin6_addr, addr_str, sizeof(addr_str));
    *rx_long_rd_id_out = dect_utils_lib_long_rd_id_from_ipv6_addr(&src_addr.sin6_addr);
    LOG_INF("SYNC RX: %d bytes from %s (0x%08x)", ret, addr_str, *rx_long_rd_id_out);

    if (src_addr_out) *src_addr_out = src_addr;
    return 0;
}

/* ── Public API ── */

void sync_init(struct net_if *iface, const struct in6_addr *mesh_prefix,
               uint32_t sink_long_rd_id)
{
    sync_iface          = iface;
    sync_mesh_prefix    = *mesh_prefix;
    sync_sink_long_rd_id = sink_long_rd_id;
}

int sync_open_socket(void)
{
    sync_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (sync_socket < 0) {
        LOG_ERR("Failed to create SYNC socket: %d", errno);
        sync_socket = -1;
        return -errno;
    }

    struct timeval timeout = { .tv_sec = SOCKET_SYNC_RX_TIMEOUT_SEC };
    int dect_iface_idx = net_if_get_by_iface(sync_iface);
    int reuse = 1;

    setsockopt(sync_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sync_socket, SOL_SOCKET, SO_BINDTODEVICE,
               &dect_iface_idx, sizeof(dect_iface_idx));
    setsockopt(sync_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in6 rx_addr = {
        .sin6_family = AF_INET6,
        .sin6_port   = htons(SYNC_PORT),
        .sin6_addr   = in6addr_any,
    };

    int ret = bind(sync_socket, (struct sockaddr *)&rx_addr, sizeof(rx_addr));
    if (ret < 0) {
        LOG_ERR("Failed to bind SYNC socket: %d", errno);
        close(sync_socket);
        sync_socket = -1;
        return -errno;
    }

    LOG_INF("SYNC socket open");
    return 0;
}

void sync_close_socket(void)
{
    if (sync_socket >= 0) {
        close(sync_socket);
        sync_socket = -1;
    }
    LOG_INF("SYNC socket closed");
}

int sync_pt_operation(uint32_t parent_long_rd_id, int32_t *offset_out)
{
    if (sync_socket < 0) {
        LOG_WRN("SYNC socket not open");
        return -ENOTCONN;
    }

    struct sockaddr_in6 dst_addr = {
        .sin6_family = AF_INET6,
        .sin6_port   = htons(SYNC_PORT),
    };
    if (!make_peer_addr(&dst_addr, parent_long_rd_id)) {
        return -EINVAL;
    }

    for (int attempt = 0; attempt < SYNC_MAX_RX_RETRIES; attempt++) {
        /* Send T0 on every attempt so FT Relay gets it even if it wasn't ready */
        struct sync_data pkt = {
            .magic_signature = SYNC_MAGIC_SIGNATURE,
        };
        pkt.T[0] = k_uptime_get_32();

        int ret = sendto(sync_socket, &pkt, sizeof(pkt), 0,
                         (struct sockaddr *)&dst_addr, sizeof(dst_addr));
        if (ret < 0) {
            LOG_WRN("Failed to send SYNC packet: %d", errno);
            continue;
        }
        LOG_INF("SYNC T0 sent (attempt %d, T0=%u)", attempt + 1, pkt.T[0]);

        struct sync_data rx;
        uint32_t rx_long_rd_id;
        uint32_t T3;

        ret = recv_sync_from_peer(&rx, &rx_long_rd_id, &T3, NULL);
        if (ret == -EAGAIN) {
            LOG_WRN("SYNC RX timeout, retrying (%d/%d)", attempt + 1, SYNC_MAX_RX_RETRIES);
            continue;
        }
        if (ret < 0) {
            LOG_WRN("SYNC RX error: %d, retrying", ret);
            continue;
        }
        if (rx_long_rd_id != parent_long_rd_id) {
            LOG_WRN("SYNC RX from unexpected peer 0x%08x (want 0x%08x), retrying",
                    rx_long_rd_id, parent_long_rd_id);
            continue;
        }

        int32_t d1 = (int32_t)(rx.T[1] - pkt.T[0]);   /* T1 - T0 */
        int32_t d2 = (int32_t)(T3 - rx.T[2]);          /* T3 - T2 */
        *offset_out = (d1 - d2) / 2;
        LOG_INF("PTP: d1=%d d2=%d offset=%d", d1, d2, *offset_out);
                return 0;
    }

    LOG_WRN("SYNC PT: exhausted retries");
    return -ETIMEDOUT;
}
int sync_ft_operation(uint32_t child_long_rd_id)
{
    if (sync_socket < 0) {
        LOG_WRN("SYNC socket not open");
        return -ENOTCONN;
    }

    for (int attempt = 0; attempt < SYNC_MAX_RX_RETRIES; attempt++) {
        struct sync_data rx;
        uint32_t rx_long_rd_id;
        uint32_t T1;
        struct sockaddr_in6 reply_addr;

        int ret = recv_sync_from_peer(&rx, &rx_long_rd_id, &T1, &reply_addr);
        if (ret == -EAGAIN) {
            LOG_WRN("SYNC RX timeout, retrying (%d/%d)", attempt + 1, SYNC_MAX_RX_RETRIES);
            continue;
        }
        if (ret < 0) {
            LOG_WRN("SYNC RX error: %d, retrying", ret);
            continue;
        }
        if (rx_long_rd_id != child_long_rd_id) {
            LOG_WRN("SYNC RX from unexpected peer 0x%08x (want 0x%08x), retrying",
                    rx_long_rd_id, child_long_rd_id);
            continue;
        }

        reply_addr.sin6_port = htons(SYNC_PORT);

        struct sync_data reply = {
            .magic_signature = SYNC_MAGIC_SIGNATURE,
        };
        reply.T[0] = rx.T[0];
        reply.T[1] = T1;
        reply.T[2] = k_uptime_get_32();

        ret = sendto(sync_socket, &reply, sizeof(reply), 0,
                     (struct sockaddr *)&reply_addr, sizeof(reply_addr));
        if (ret < 0) {
            LOG_WRN("Failed to send SYNC reply: %d", errno);
            return -errno;
        }

        LOG_INF("SYNC reply sent (T0=%u T1=%u T2=%u)", reply.T[0], reply.T[1], reply.T[2]);
        return 0;
    }

    LOG_WRN("SYNC FT: exhausted retries");
    return -ETIMEDOUT;
}