#include "uart.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_data, LOG_LEVEL_INF);

// STREAM HEADER:
// MAGIC_0, MAGIC_1, MAGIC_0, MAGIC_1
// struct packet_metadata
// total packet size
#define STREAM_HEADER_SIZE (4 + sizeof(size_t) + (4 + 4 + 4 + (1 + 4 * ROUTING_MAX_HOPS + 4 * ROUTING_MAX_HOPS))) // 4 magic bytes + data size + packet_metadata to bytes

#define MAGIC_0 0xAA
#define MAGIC_1 0x55

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

extern uint32_t current_long_rd_id;
extern uint32_t message_counter;

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

int uart_handshake_send_id_timestamp(uint32_t long_rd_id, uint32_t timestamp)
{
    uint8_t *id = (uint8_t *)&long_rd_id;
    uint8_t *ts = (uint8_t *)&timestamp;

    for (int r = 0; r < HANDSHAKE_REPEAT; r++) {
        uart_poll_out(uart_dev, HANDSHAKE_MAGIC_0);
        uart_poll_out(uart_dev, HANDSHAKE_MAGIC_1);
        for (int i = 0; i < 4; i++) { // For timestamp
            uart_poll_out(uart_dev, ts[i]);
        }
        for (int i = 0; i < 4; i++) { // For long RD ID // TODO: Remove this magic number
            uart_poll_out(uart_dev, id[i]);
        }
        k_msleep(HANDSHAKE_INTERVAL_MS);
    }

    LOG_INF("Handshake: sent FT ID 0x%08x (%d times)", long_rd_id, HANDSHAKE_REPEAT); // TODO: Print offset here
    return 0;
}

#if IS_ENABLED(CONFIG_DECT_RELAY_PT)

static volatile bool handshake_received;
static volatile uint32_t handshake_rx_id;
static volatile uint32_t handshake_rx_offset;

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
                handshake_rx_offset = sibling_ft_timestamp - current_pt_timestamp; // Offset from the POV of the FT
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
int uart_handshake_receive_id_timestamp(uint32_t *long_rd_id, uint32_t *offset, int timeout_sec)
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
        LOG_INF("Handshake: received sibling FT ID 0x%08x", *long_rd_id);
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
        return;
    }
    k_sem_take(&uart_tx_done_sem, K_FOREVER);
}

int uart_send_image(const uint8_t *data, uint32_t length, const struct packet_metadata *meta)
{
    int ret = uart_stream_begin(length, meta);
    if (ret)
        return ret;

    uart_stream_chunk(data, length);
    uart_stream_end();
    return 0;
}

bool uart_is_ready(void) { return uart_ready; }

/* ── Streaming API ── */

int uart_stream_begin(size_t total_length, const struct packet_metadata *meta)
{
    if (!uart_ready)
        return -ENOTCONN;
    if (stream_active)
        LOG_WRN("Previous stream not finished");

    // Magic + non-array elements of metadata struct
    uint8_t header[STREAM_HEADER_SIZE] = {
        MAGIC_0,
        MAGIC_1,
        MAGIC_0,
        MAGIC_1,
        (total_length >> 0) & 0xFF,
        (total_length >> 8) & 0xFF,
        (total_length >> 16) & 0xFF,
        (total_length >> 24) & 0xFF,
        (meta->seq_num >> 0) & 0xFF,
        (meta->seq_num >> 8) & 0xFF,
        (meta->seq_num >> 16) & 0xFF,
        (meta->seq_num >> 24) & 0xFF,
        (meta->timestamp_pt >> 0) & 0xFF,
        (meta->timestamp_pt >> 8) & 0xFF,
        (meta->timestamp_pt >> 16) & 0xFF,
        (meta->timestamp_pt >> 24) & 0xFF,
        (meta->offset_pt_to_ft >> 0) & 0xFF,
        (meta->offset_pt_to_ft >> 8) & 0xFF,
        (meta->offset_pt_to_ft >> 16) & 0xFF,
        (meta->offset_pt_to_ft >> 24) & 0xFF,
        meta->route_delays.num_links,
        // devices_visited
        // per_link_delay
        // total_length
    };

    int header_idx = 21; // Count number of elements in above array

    // devices_visited array of hop_delays in metadata
    for (int i = 0; i < ROUTING_MAX_HOPS; i++) {
        for (int j = 0; j < sizeof(meta->route_delays.devices_visited[0]); j++) {
            header[header_idx++] = (meta->route_delays.devices_visited[i] >> (j * 8)) & 0xFF;
        }
    }

    // per_link_delay array of hop delays in metadata
    for (int i = 0; i < ROUTING_MAX_HOPS; i++) {
        for (int j = 0; j < sizeof(meta->route_delays.per_link_delay[0]); j++) {
            header[header_idx++] = (meta->route_delays.per_link_delay[i] >> (j * 8)) & 0xFF;
        }
    }

    uart_out_bytes(header, sizeof(header));

    /* CRC seeded over metadata bytes (header[4] and out) */
    stream_crc = crc16_update(0xFFFF, &header[4], sizeof(header) - 4);
    stream_active = true;

    LOG_INF("UART stream start: %u bytes",
            total_length);
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

int uart_stream_end(void)
{
    if (!stream_active)
        return -EINVAL;
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

static void uart_tx_thread_fn(void *p1, void *p2, void *p3)
{
    static uint8_t payload_copy[CHUNK_BUF_SIZE];
    uint16_t expected_total = 0;
    uint16_t received_count = 0;

    while (1)
    {
        struct rx_chunk *chunk = k_fifo_get(&pending_chunks, K_FOREVER);
        struct data_packet *pkt = (struct data_packet *)chunk->data;

        // Copy everything we need out of the chunk
        uint16_t packet_idx   = pkt->packet_idx;
        uint16_t total_packets = pkt->total_packets;
        uint16_t payload_len  = pkt->payload_len;
        
        uint32_t total_size   = pkt->total_data_size;
        memcpy(payload_copy, pkt->payload, payload_len);

        // Return chunk immediately before slow UART TX
        k_fifo_put(&free_chunks, chunk);

        if (packet_idx == 0)
        {
            expected_total = total_packets;
            received_count = 0;
            struct packet_metadata meta = {
                .seq_num = message_counter++,
                .timestamp_pt = pkt->timestamp_pt,
                .offset_pt_to_ft = pkt->offset_pt_to_ft,
                .route_delays = pkt->route_delays,
            };
            uart_stream_begin(total_size, &meta);
        }

        uart_stream_chunk(payload_copy, payload_len);
        received_count++;

        if (received_count >= expected_total && expected_total > 0)
        {
            uart_stream_end();
            expected_total = 0;
            received_count = 0;
        }
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
#define UART_RING_BUF_SIZE   32768

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
    READ_HEADER,
    READ_PAYLOAD,
    READ_CRC,
} rx_state_t;

static uart_rx_frame_cb_t rx_frame_cb;
static rx_state_t rx_state = WAIT_MAGIC_0;
static uint8_t    rx_header[STREAM_HEADER_SIZE];
static uint8_t    rx_header_idx;
static uint8_t   *rx_payload_buf;
static uint32_t   rx_payload_len;
static uint32_t   rx_payload_received;
static uint8_t    rx_crc_bytes[2];
static uint8_t    rx_crc_idx;
static uint16_t   rx_running_crc;

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
        rx_state = (b == MAGIC_0) ? WAIT_MAGIC_3 : WAIT_MAGIC_0;
        break;
    case WAIT_MAGIC_3:
        if (b == MAGIC_1) {
            rx_header_idx = 0;
            rx_state = READ_HEADER;
        } else {
            rx_state = WAIT_MAGIC_0;
        }
        break;

    case READ_HEADER:
        rx_header[rx_header_idx++] = b;
        if (rx_header_idx == sizeof(rx_header)) {
            rx_payload_len = rx_header[4]
                           | (rx_header[5]  << 8)
                           | (rx_header[6]  << 16)
                           | (rx_header[7] << 24);

            if (rx_payload_len == 0 || rx_payload_len > UART_RX_MAX_PAYLOAD) {
                LOG_WRN("Invalid payload len %u, resetting", rx_payload_len);
                rx_state = WAIT_MAGIC_0;
                break;
            }

            rx_running_crc = crc16_update_rx(0xFFFF, rx_header, sizeof(rx_header));
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
                
                int parse_idx = 4; // Index after magic bytes
                uint8_t b0, b1, b2, b3;

                // seq_num
                b0 = rx_header[parse_idx++]; 
                b1 = rx_header[parse_idx++]; 
                b2 = rx_header[parse_idx++]; 
                b3 = rx_header[parse_idx++];
                rx_frame_work.meta.seq_num = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);

                // timestamp_ftpt_child
                b0 = rx_header[parse_idx++]; 
                b1 = rx_header[parse_idx++]; 
                b2 = rx_header[parse_idx++]; 
                b3 = rx_header[parse_idx++];
                rx_frame_work.meta.timestamp_pt = 
                    b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);

                // offset_pt_to_ftpt
                b0 = rx_header[parse_idx++]; 
                b1 = rx_header[parse_idx++]; 
                b2 = rx_header[parse_idx++]; 
                b3 = rx_header[parse_idx++];
                rx_frame_work.meta.offset_pt_to_ft = 
                    b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);

                // route_delays.num_devices
                rx_frame_work.meta.route_delays.num_links = rx_header[parse_idx++];

                // route_delays.devices_visited
                for (int i = 0; i < ROUTING_MAX_HOPS; i++) {
                    b0 = rx_header[parse_idx++];
                    b1 = rx_header[parse_idx++]; 
                    b2 = rx_header[parse_idx++]; 
                    b3 = rx_header[parse_idx++];
                    rx_frame_work.meta.route_delays.devices_visited[i] = 
                        b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
                }

                // route_delays.per_link_delay
                for (int i = 0; i < ROUTING_MAX_HOPS; i++) {
                    b0 = rx_header[parse_idx++];
                    b1 = rx_header[parse_idx++]; 
                    b2 = rx_header[parse_idx++]; 
                    b3 = rx_header[parse_idx++];
                    rx_frame_work.meta.route_delays.per_link_delay[i] = 
                        b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
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
                     evt->data.rx.buf + evt->data.rx.offset,
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

// TODO: Consider redesigning to buffer of DECT packets and adding timeout