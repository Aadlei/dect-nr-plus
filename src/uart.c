#include "uart.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_data, LOG_LEVEL_INF);

#define MAGIC_0 0xAA
#define MAGIC_1 0x55
#define MAGIC_2 0xBB
#define MAGIC_3 0x44

/* UART stream framing header (97 bytes total):
 * [MAGIC:8][total_length:4][seq_num:4][timestamp_pt:4][offset_pt_to_ft:4][num_links:1]
 * [devices_visited:4*ROUTING_MAX_HOPS]
 * [per_link_delay:4*ROUTING_MAX_HOPS]
 * [per_link_rssi:1*ROUTING_MAX_HOPS]
 */

#define STREAM_HEADER_SIZE  (8 + 4 + 4 + 4 + 4 + 1         \
                             + (4 * ROUTING_MAX_HOPS)      \
                             + (4 * ROUTING_MAX_HOPS)      \
                             + (1 * ROUTING_MAX_HOPS))

#define HANDSHAKE_MAGIC_0 0xF4
#define HANDSHAKE_MAGIC_1 0xAF
#define HANDSHAKE_REPEAT  100
#define HANDSHAKE_INTERVAL_MS 200

static const struct device *uart_dev;
static uint16_t stream_crc;
static bool stream_active;
static bool uart_ready;

static struct rx_chunk chunk_pool[CHUNK_POOL_COUNT];
static struct k_fifo free_chunks;
static struct k_fifo pending_chunks;
static uint16_t expected_total;
static uint16_t next_expected_idx;
static struct packet_metadata last_meta;
static uint32_t last_total_size;

extern uint32_t current_long_rd_id;



/* ── Init ── */
int uart_data_init(void)
{
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("uart1 not ready");
        return -ENODEV;
    }
    uart_ready = true;

    
#ifdef CONFIG_DECT_RELAY_PT
    return uart_rx_start();
#else
    return uart_tx_thread_start();
#endif
}

int uart_handshake_init(void)
{
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("uart1 not ready");
        return -ENODEV;
    }
    return 0;
}

int uart_handshake_send_id_timestamp(uint32_t long_rd_id)
{
    uint8_t *id_ptr = (uint8_t *)&long_rd_id;

    for (int r = 0; r < HANDSHAKE_REPEAT; r++) {
        uint32_t ts = k_uptime_get_32();
        uint8_t *ts_ptr = (uint8_t *)&ts;

        uart_poll_out(uart_dev, HANDSHAKE_MAGIC_0);
        uart_poll_out(uart_dev, HANDSHAKE_MAGIC_1);
        for (int i = 0; i < 4; i++) uart_poll_out(uart_dev, ts_ptr[i]);
        for (int i = 0; i < 4; i++) uart_poll_out(uart_dev, id_ptr[i]);

        k_msleep(HANDSHAKE_INTERVAL_MS);
    }

    LOG_INF("Handshake: sent FT ID 0x%08x (%d times)", long_rd_id, HANDSHAKE_REPEAT);
    return 0;
}

#if IS_ENABLED(CONFIG_DECT_RELAY_PT)

static volatile bool handshake_received;
static volatile uint32_t handshake_rx_id;
static volatile int32_t handshake_rx_offset;

static uint8_t hs_rx_buf[10]; /* magic0 + magic1 + 4 ID bytes + 4 timestamp bytes */
static K_SEM_DEFINE(hs_rx_sem, 0, 1);

static void handshake_async_cb(const struct device *dev,
                               struct uart_event *evt, void *user_data)
{
    if (evt->type == UART_RX_RDY) {
        uint8_t *d = evt->data.rx.buf + evt->data.rx.offset;
        uint32_t len = evt->data.rx.len;

        uint32_t current_pt_timestamp = k_uptime_get_32();
        uint32_t sibling_ft_timestamp;

        for (uint32_t i = 0; i <= len - 10; i++) { // TODO: Remove this magic number
            if (d[i] == HANDSHAKE_MAGIC_0 && d[i + 1] == HANDSHAKE_MAGIC_1) {
                memcpy(&sibling_ft_timestamp, &d[i + 2], 4); // Bytes 2-5: timestamp
                memcpy(&handshake_rx_id, &d[i + 6], 4); // Bytes: 6-9: long RD ID
                LOG_INF("Timestamps:");
                LOG_INF("  sibling_ft_timestamp: %u", sibling_ft_timestamp);
                LOG_INF("  current_pt_timestamp: %u", current_pt_timestamp);
                handshake_rx_offset = (int32_t)(sibling_ft_timestamp-current_pt_timestamp); // Offset from the POV of the PT
                LOG_INF("  handshake_rx_offset: %d", handshake_rx_offset);
                handshake_received = true;
                k_sem_give(&hs_rx_sem);
                return;
            }
        }
    } else if (evt->type == UART_RX_DISABLED) {
        k_sem_give(&hs_rx_sem);
    }
}

// Updates the long RD ID and calculated offset between this PT device timestamp and the sibling FT device in FTPT connection
int uart_handshake_receive_id_timestamp(uint32_t *long_rd_id, int32_t *offset, int timeout_sec)
{
    int ret;

    handshake_received = false;

    ret = uart_callback_set(uart_dev, handshake_async_cb, NULL);
    if (ret) {
        LOG_ERR("Handshake callback set failed: %d", ret);
        return ret;
    }

    ret = uart_rx_enable(uart_dev, hs_rx_buf, sizeof(hs_rx_buf), 1000);
    if (ret) {
        LOG_ERR("Handshake RX enable failed: %d", ret);
        return ret;
    }

    ret = k_sem_take(&hs_rx_sem, K_SECONDS(timeout_sec));

    uart_rx_disable(uart_dev);
    k_msleep(100);

    if (handshake_received) {
        *long_rd_id = handshake_rx_id;
        *offset = handshake_rx_offset;
        LOG_INF("Handshake: received sibling FT ID 0x%08x with offset: %dms", *long_rd_id, *offset);
        return 0;
    }

    LOG_WRN("Handshake: timed out after %d seconds", timeout_sec);
    return -ETIMEDOUT;
}

#endif


#ifndef CONFIG_DECT_RELAY_PT
#define UART_TX_STACK_SIZE 2048
static K_SEM_DEFINE(uart_tx_done_sem, 0, 1);
K_THREAD_STACK_DEFINE(uart_tx_stack, UART_TX_STACK_SIZE);
static struct k_thread uart_tx_thread_data;

/* CRC16/Modbus */
static uint16_t crc16_update(uint16_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

static void uart_out_bytes(const uint8_t *data, size_t len)
{
    int ret = uart_tx(uart_dev, data, len, SYS_FOREVER_US);
    if (ret) {
        LOG_ERR("uart_tx failed: %d", ret);
        stream_active = false;
        return;
    }
    k_sem_take(&uart_tx_done_sem, K_FOREVER);
}

int uart_send_image(const uint8_t *data, uint32_t length, const struct packet_metadata *meta)
{
    int ret = uart_stream_begin(length);
    if (ret)
        return ret;
    uart_stream_chunk(data, length);
    uart_stream_end(meta);
    return 0;
}

bool uart_is_ready(void) { return uart_ready; }

/* ── Streaming API ── */
int uart_stream_begin(uint32_t total_length)
{
    if (!uart_ready) return -ENOTCONN;
    if (stream_active) LOG_WRN("Previous stream not finished");

    uint8_t header[12] = {
        MAGIC_0, MAGIC_1, MAGIC_2, MAGIC_3,
        MAGIC_0, MAGIC_1, MAGIC_2, MAGIC_3,
        (total_length >> 0) & 0xFF,
        (total_length >> 8) & 0xFF,
        (total_length >> 16) & 0xFF,
        (total_length >> 24) & 0xFF,
    };
    uart_out_bytes(header, sizeof(header));
    stream_crc = crc16_update(0xFFFF, &header[8], 4);
    stream_active = true;
    LOG_INF("UART stream start: %u bytes", total_length);
    return 0;
}

int uart_stream_chunk(const uint8_t *data, uint16_t len)
{
    if (!stream_active)
        return -EINVAL;
    uart_out_bytes(data, len);
    stream_crc = crc16_update(stream_crc, data, len);
    return 0;
}

int uart_stream_end(const struct packet_metadata *meta)
{
    if (!stream_active) return -EINVAL;

    uint8_t trailer[STREAM_HEADER_SIZE - 12]; // metadata without magic+length
    int idx = 0;
    trailer[idx++] = (meta->seq_num >> 0) & 0xFF;
    trailer[idx++] = (meta->seq_num >> 8) & 0xFF;
    trailer[idx++] = (meta->seq_num >> 16) & 0xFF;
    trailer[idx++] = (meta->seq_num >> 24) & 0xFF;
    trailer[idx++] = (meta->timestamp_pt >> 0) & 0xFF;
    trailer[idx++] = (meta->timestamp_pt >> 8) & 0xFF;
    trailer[idx++] = (meta->timestamp_pt >> 16) & 0xFF;
    trailer[idx++] = (meta->timestamp_pt >> 24) & 0xFF;
    trailer[idx++] = (meta->offset_pt_to_ft >> 0) & 0xFF;
    trailer[idx++] = (meta->offset_pt_to_ft >> 8) & 0xFF;
    trailer[idx++] = (meta->offset_pt_to_ft >> 16) & 0xFF;
    trailer[idx++] = (meta->offset_pt_to_ft >> 24) & 0xFF;
    trailer[idx++] = meta->route_delays.num_links;
    for (int i = 0; i < ROUTING_MAX_HOPS; i++)
        for (int j = 0; j < 4; j++)
            trailer[idx++] = (meta->route_delays.devices_visited[i] >> (j*8)) & 0xFF;
    for (int i = 0; i < ROUTING_MAX_HOPS; i++)
        for (int j = 0; j < 4; j++)
            trailer[idx++] = (meta->route_delays.per_link_delay[i] >> (j*8)) & 0xFF;
    for (int i = 0; i < ROUTING_MAX_HOPS; i++)
        trailer[idx++] = (uint8_t)meta->route_delays.per_link_rssi[i];

    stream_crc = crc16_update(stream_crc, trailer, sizeof(trailer));
    uart_out_bytes(trailer, sizeof(trailer));

    uint8_t crc_bytes[] = {stream_crc & 0xFF, (stream_crc >> 8) & 0xFF};
    uart_out_bytes(crc_bytes, 2);
    LOG_INF("UART stream complete (CRC=0x%04X)", stream_crc);
    stream_active = false;
    return 0;
}
static void uart_tx_async_cb(const struct device *dev,
                              struct uart_event *evt, void *user_data)
{
    if (evt->type == UART_TX_DONE) {
        k_sem_give(&uart_tx_done_sem);
    }
}

/* ── Chunk pool + TX thread ── */

struct rx_chunk *uart_get_free_chunk(void) { return k_fifo_get(&free_chunks, K_FOREVER); }
void uart_return_free_chunk(struct rx_chunk *c) { k_fifo_put(&free_chunks, c); }
void uart_queue_chunk(struct rx_chunk *c) { k_fifo_put(&pending_chunks, c); }

// Send one in-order chunk to the UART stream
static void process_chunk(struct rx_chunk *chunk)
{
    static uint8_t payload_copy[MAX_PAYLOAD_SIZE];
    static struct packet_metadata last_meta;

    struct data_packet *pkt = (struct data_packet *)chunk->data;

    if (pkt->packet_idx == 0) {
        expected_total            = pkt->total_packets;
        next_expected_idx         = 0;
        last_meta.seq_num         = pkt->seq_num;
        last_meta.timestamp_pt    = pkt->timestamp_pt;
        last_meta.offset_pt_to_ft = pkt->offset_pt_to_ft;
        uart_stream_begin(pkt->total_data_size);
    }
    last_meta.route_delays = pkt->route_delays;

    memcpy(payload_copy, pkt->payload, pkt->payload_len);
    uint16_t payload_len = pkt->payload_len;
    k_fifo_put(&free_chunks, chunk);

    uart_stream_chunk(payload_copy, payload_len);
    next_expected_idx++;

    if (next_expected_idx >= expected_total && expected_total > 0) {
        uart_stream_end(&last_meta);
        expected_total    = 0;
        next_expected_idx = 0;
    }
}

// Abandon stream when chunks are lost on the link
static void abort_stream(void)
{
    stream_active     = false;   // truncated; bridge resyncs on next MAGIC
    expected_total    = 0;
    next_expected_idx = 0;
}

static void uart_tx_thread_fn(void *p1, void *p2, void *p3)
{
    struct rx_chunk *held = NULL;
    expected_total = 0;
    next_expected_idx = 0;

    while (1) {
        struct rx_chunk *chunk = k_fifo_get(&pending_chunks, K_FOREVER);
        uint16_t idx = ((struct data_packet *)chunk->data)->packet_idx;
        
        // Process the in-order chunk. Also process the held if in order
        if (idx == next_expected_idx) {
            process_chunk(chunk);

            if (held && ((struct data_packet *)held->data)->packet_idx == next_expected_idx) {
                process_chunk(held);
                held = NULL;
            }
            continue;
        }

        // New image. Abort current stream.
        if (idx == 0) {
            LOG_WRN("Incomplete image (%u/%u chunks), resyncing", next_expected_idx, expected_total);
            abort_stream();

            if (held) {
                k_fifo_put(&free_chunks, held);
                held = NULL;
            }
            process_chunk(chunk);
            continue;
        }

        // Chunks out-of-order. Hold the chunk.
        if (held == NULL && expected_total != 0 && idx == next_expected_idx + 1) {
            held = chunk;
            continue;
        }

        // Rest is chunk loss. Abort current stream.
        LOG_WRN("Lost chunk (expected %u, got %u), aborting image", next_expected_idx, idx);
        abort_stream();
      
        if (held) {
            k_fifo_put(&free_chunks, held);
            held = NULL;
        }
        k_fifo_put(&free_chunks, chunk);
    }
}


int uart_tx_thread_start(void)
{
    uart_callback_set(uart_dev, uart_tx_async_cb, NULL);  // add this

    k_fifo_init(&free_chunks);
    k_fifo_init(&pending_chunks);
    for (int i = 0; i < CHUNK_POOL_COUNT; i++)
        k_fifo_put(&free_chunks, &chunk_pool[i]);

    k_thread_create(&uart_tx_thread_data, uart_tx_stack,
                    K_THREAD_STACK_SIZEOF(uart_tx_stack),
                    uart_tx_thread_fn, NULL, NULL, NULL,
                    8, 0, K_NO_WAIT);
    k_thread_name_set(&uart_tx_thread_data, "uart_tx");
    LOG_INF("UART TX thread started");
    return 0;
}

#endif /* !CONFIG_DECT_RELAY_PT */

#ifdef CONFIG_DECT_RELAY_PT

#define UART_RX_BUF_SIZE     4096
#define UART_RX_BUF_COUNT    2
#define UART_RX_TIMEOUT_US   1000
#define UART_RX_MAX_PAYLOAD  (1024 * 16)
#define UART_RING_BUF_SIZE   8192

/* Double-buffered RX */
static uint8_t rx_bufs[UART_RX_BUF_COUNT][UART_RX_BUF_SIZE];
static uint8_t rx_buf_idx;

/* Ring buffer for ISR -> thread handoff */
RING_BUF_DECLARE(uart_rx_ring, UART_RING_BUF_SIZE);
static K_SEM_DEFINE(uart_rx_sem, 0, 1);

/* Processing thread */
#define UART_RX_PROC_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(uart_rx_proc_stack, UART_RX_PROC_STACK_SIZE);
static struct k_thread uart_rx_proc_thread;

/* Payload storage */
static uint8_t rx_payload_storage[UART_RX_MAX_PAYLOAD];

/* Parser state */
typedef enum {
    WAIT_MAGIC_0,
    WAIT_MAGIC_1,
    WAIT_MAGIC_2,
    WAIT_MAGIC_3,
    WAIT_MAGIC_4,
    WAIT_MAGIC_5,
    WAIT_MAGIC_6,
    WAIT_MAGIC_7,
    READ_LEN,    /* 4 bytes → total_length, stored in rx_header[0..3] */
    READ_PAYLOAD,
    READ_META,   /* 85 bytes → metadata, stored in rx_header[4..88]   */
    READ_CRC,
} rx_state_t;

static uart_rx_frame_cb_t rx_frame_cb;
static rx_state_t rx_state = WAIT_MAGIC_0;
static uint8_t    rx_header[STREAM_HEADER_SIZE - 8];
static uint8_t    rx_header_idx;
static uint8_t   *rx_payload_buf;
static uint32_t   rx_payload_len;
static uint32_t   rx_payload_received;
static uint8_t    rx_crc_bytes[2];
static uint8_t    rx_crc_idx;
static uint16_t   rx_running_crc;

BUILD_ASSERT(sizeof(rx_header) == STREAM_HEADER_SIZE - 8); // Change this based on STREAM_HEADER_SIZE

/* Work item for deferred callback (ISR -> thread context) */
struct rx_frame_work_t {
    struct k_work work;
    uint32_t payload_len;
    struct packet_metadata meta;
    uint8_t payload[UART_RX_MAX_PAYLOAD];
};
static struct rx_frame_work_t rx_frame_work;

static void rx_frame_work_handler(struct k_work *work)
{
    struct rx_frame_work_t *ctx = CONTAINER_OF(work, struct rx_frame_work_t, work);
    if (rx_frame_cb) {
        rx_frame_cb(ctx->payload, ctx->payload_len, &ctx->meta);
    }
}

static uint16_t crc16_update_rx(uint16_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

static void process_byte(uint8_t b)
{
    switch (rx_state) {
    case WAIT_MAGIC_0:
        if (b == MAGIC_0) rx_state = WAIT_MAGIC_1;
        break;
    case WAIT_MAGIC_1:
        rx_state = (b == MAGIC_1) ? WAIT_MAGIC_2 : WAIT_MAGIC_0;
        break;
    case WAIT_MAGIC_2:
        rx_state = (b == MAGIC_2) ? WAIT_MAGIC_3 : WAIT_MAGIC_0;
        break;
    case WAIT_MAGIC_3:
        rx_state = (b == MAGIC_3) ? WAIT_MAGIC_4 : WAIT_MAGIC_0;
        break;
    case WAIT_MAGIC_4:
        rx_state = (b == MAGIC_0) ? WAIT_MAGIC_5 : WAIT_MAGIC_0;
        break;
    case WAIT_MAGIC_5:
        rx_state = (b == MAGIC_1) ? WAIT_MAGIC_6 : WAIT_MAGIC_0;
        break;
    case WAIT_MAGIC_6:
        rx_state = (b == MAGIC_2) ? WAIT_MAGIC_7 : WAIT_MAGIC_0;
        break;
    case WAIT_MAGIC_7:
        if (b == MAGIC_3) {
            rx_header_idx = 0;
            rx_state = READ_LEN;
        } else {
            rx_state = WAIT_MAGIC_0;
        }
        break;

    case READ_LEN:
        rx_header[rx_header_idx++] = b;
        if (rx_header_idx == 4) {
            rx_payload_len = (uint32_t)rx_header[0]
                           | ((uint32_t)rx_header[1] << 8)
                           | ((uint32_t)rx_header[2] << 16)
                           | ((uint32_t)rx_header[3] << 24);

            if (rx_payload_len == 0 || rx_payload_len > UART_RX_MAX_PAYLOAD) {
                LOG_WRN("Invalid payload len %u, resetting", rx_payload_len);
                rx_state = WAIT_MAGIC_0;
                break;
            }

            rx_running_crc = crc16_update_rx(0xFFFF, rx_header, 4);
            rx_payload_received = 0;
            rx_payload_buf = rx_payload_storage;
            LOG_INF("RX frame: len=%u", rx_payload_len);
            rx_state = READ_PAYLOAD;
        }
        break;

    case READ_PAYLOAD:
        if (rx_payload_received < rx_payload_len) {
            rx_payload_buf[rx_payload_received++] = b;
            rx_running_crc = crc16_update_rx(rx_running_crc, &b, 1);
        }
        if (rx_payload_received >= rx_payload_len) {
            rx_header_idx = 4; /* fill metadata into rx_header[4..] */
            rx_state = READ_META;
        }
        break;

    case READ_META:
        rx_header[rx_header_idx++] = b;
        rx_running_crc = crc16_update_rx(rx_running_crc, &b, 1);
        if (rx_header_idx == sizeof(rx_header)) {
            rx_crc_idx = 0;
            rx_state = READ_CRC;
        }
        break;

    case READ_CRC:
        rx_crc_bytes[rx_crc_idx++] = b;
        if (rx_crc_idx == 2) {
            uint16_t received_crc = rx_crc_bytes[0] | (rx_crc_bytes[1] << 8);
            if (received_crc == rx_running_crc) {
                LOG_INF("RX frame OK (CRC=0x%04X)", received_crc);
 
                int parse_idx = 4; /* index after total_length in rx_header */
                uint8_t b0, b1, b2, b3;
 
                /* seq_num */
                b0 = rx_header[parse_idx++];
                b1 = rx_header[parse_idx++];
                b2 = rx_header[parse_idx++];
                b3 = rx_header[parse_idx++];
                rx_frame_work.meta.seq_num = 
                    b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
 
                /* timestamp_pt */
                b0 = rx_header[parse_idx++];
                b1 = rx_header[parse_idx++];
                b2 = rx_header[parse_idx++];
                b3 = rx_header[parse_idx++];
                rx_frame_work.meta.timestamp_pt =
                    b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
 
                /* offset_pt_to_ft */
                b0 = rx_header[parse_idx++];
                b1 = rx_header[parse_idx++];
                b2 = rx_header[parse_idx++];
                b3 = rx_header[parse_idx++];
                rx_frame_work.meta.offset_pt_to_ft =
                    b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
 
                /* route_delays.num_links */
                rx_frame_work.meta.route_delays.num_links = rx_header[parse_idx++];
 
                /* route_delays.devices_visited */
                for (int i = 0; i < ROUTING_MAX_HOPS; i++) {
                    b0 = rx_header[parse_idx++];
                    b1 = rx_header[parse_idx++];
                    b2 = rx_header[parse_idx++];
                    b3 = rx_header[parse_idx++];
                    rx_frame_work.meta.route_delays.devices_visited[i] =
                        b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
                }
 
                /* route_delays.per_link_delay */
                for (int i = 0; i < ROUTING_MAX_HOPS; i++) {
                    b0 = rx_header[parse_idx++];
                    b1 = rx_header[parse_idx++];
                    b2 = rx_header[parse_idx++];
                    b3 = rx_header[parse_idx++];
                    rx_frame_work.meta.route_delays.per_link_delay[i] =
                        b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
                }
 
                /* route_delays.per_link_rssi */
                for (int i = 0; i < ROUTING_MAX_HOPS; i++) {
                    rx_frame_work.meta.route_delays.per_link_rssi[i] =
                        (int8_t)rx_header[parse_idx++];
                }
 
                rx_frame_work.payload_len = rx_payload_len;
                memcpy(rx_frame_work.payload, rx_payload_buf, rx_payload_len);
                k_work_submit(&rx_frame_work.work);
            } else {
                LOG_ERR("CRC mismatch: got 0x%04X expected 0x%04X",
                        received_crc, rx_running_crc);
            }
            rx_state = WAIT_MAGIC_0;
        }
        break;
    }
}

static void uart_rx_proc_fn(void *p1, void *p2, void *p3) // Maybe here that incoming data of UART frame is handled
{
    uint8_t buf[64];
    while (1) {
        k_sem_take(&uart_rx_sem, K_FOREVER);
        uint32_t read;
        while ((read = ring_buf_get(&uart_rx_ring, buf, sizeof(buf))) > 0) {
            for (uint32_t i = 0; i < read; i++)
                process_byte(buf[i]);
        }
    }
}

static void uart_async_cb(const struct device *dev,
                          struct uart_event *evt,
                          void *user_data)
{
    switch (evt->type) {

    case UART_RX_RDY:
        ring_buf_put(&uart_rx_ring,
                     evt->data.rx.buf + evt->data.rx.offset, // offset is not SYNC offset
                     evt->data.rx.len);
        k_sem_give(&uart_rx_sem);
        break;

    case UART_RX_BUF_REQUEST:
        rx_buf_idx = (rx_buf_idx + 1) % UART_RX_BUF_COUNT;
        uart_rx_buf_rsp(dev, rx_bufs[rx_buf_idx], UART_RX_BUF_SIZE);
        break;

    case UART_RX_BUF_RELEASED:
        break;

    case UART_RX_DISABLED:
        LOG_WRN("UART RX disabled, re-enabling");
        uart_rx_enable(dev, rx_bufs[0], UART_RX_BUF_SIZE, UART_RX_TIMEOUT_US);
        break;

    case UART_RX_STOPPED:
        LOG_ERR("UART RX stopped: reason %d", evt->data.rx_stop.reason);
        break;

    default:
        break;
    }
}

int uart_rx_start(void)
{
    int ret;

    k_work_init(&rx_frame_work.work, rx_frame_work_handler);

    ret = uart_callback_set(uart_dev, uart_async_cb, NULL);
    if (ret) {
        LOG_ERR("uart_callback_set failed: %d", ret);
        return ret;
    }

    rx_buf_idx = 0;
    ret = uart_rx_enable(uart_dev, rx_bufs[0], UART_RX_BUF_SIZE, UART_RX_TIMEOUT_US);
    if (ret) {
        LOG_ERR("uart_rx_enable failed: %d", ret);
        return ret;
    }

    k_thread_create(&uart_rx_proc_thread, uart_rx_proc_stack,
                    K_THREAD_STACK_SIZEOF(uart_rx_proc_stack),
                    uart_rx_proc_fn, NULL, NULL, NULL,
                    8, 0, K_NO_WAIT);
    k_thread_name_set(&uart_rx_proc_thread, "uart_rx_proc");

    LOG_INF("UART async RX started");
    return 0;
}

void uart_rx_set_frame_callback(uart_rx_frame_cb_t cb)
{
    rx_frame_cb = cb;
}

#endif /* CONFIG_DECT_RELAY_PT */