#include "uart.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_data, LOG_LEVEL_INF);

#define MAGIC_0 0xAA
#define MAGIC_1 0x55

#define UART_TX_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(uart_tx_stack, UART_TX_STACK_SIZE);
static struct k_thread uart_tx_thread_data;

static const struct device *uart_dev;
static uint16_t stream_crc;
static bool stream_active;
static bool uart_ready;

static struct rx_chunk chunk_pool[CHUNK_POOL_COUNT];
static struct k_fifo free_chunks;
static struct k_fifo pending_chunks;

extern uint32_t current_long_rd_id;
extern uint32_t message_counter;

/* CRC16/Modbus */
static uint16_t crc16_update(uint16_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

static void uart_out_bytes(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++)
        uart_poll_out(uart_dev, data[i]);
}

/* ── Init ── */

int uart_data_init(void)
{
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("uart1 not ready");
        return -ENODEV;
    }
    uart_ready = true;
    LOG_INF("uart1 ready (%s)", uart_dev->name);
    return 0;
}

int uart_send_image(const uint8_t *data, uint32_t length, const struct image_metadata *meta)
{
    int ret = uart_stream_begin(length, meta);
    if (ret) return ret;

    uart_stream_chunk(data, length);
    uart_stream_end();
    return 0;
}

bool uart_is_ready(void) { return uart_ready; }

/* ── Streaming API ── */

int uart_stream_begin(size_t total_length, const struct image_metadata *meta)
{
    if (!uart_ready) return -ENOTCONN;
    if (stream_active) LOG_WRN("Previous stream not finished");

    uint8_t header[] = {
        MAGIC_0, MAGIC_1, MAGIC_0, MAGIC_1,
        meta->tx_id & 0xFF, (meta->tx_id >> 8) & 0xFF,
        meta->hop_count,
        (meta->seq_num >>  0) & 0xFF, (meta->seq_num >>  8) & 0xFF,
        (meta->seq_num >> 16) & 0xFF, (meta->seq_num >> 24) & 0xFF,
        (total_length >>  0) & 0xFF, (total_length >>  8) & 0xFF,
        (total_length >> 16) & 0xFF, (total_length >> 24) & 0xFF,
    };
    uart_out_bytes(header, sizeof(header));

    /* CRC seeded over metadata bytes (header[4..10]) */
    stream_crc = crc16_update(0xFFFF, &header[4], 7);
    stream_active = true;

    LOG_INF("UART stream start: %u bytes (tx=%u, hops=%u, seq=%u)",
            total_length, meta->tx_id, meta->hop_count, meta->seq_num);
    return 0;
}

int uart_stream_chunk(const uint8_t *data, uint16_t len)
{
    if (!stream_active) return -EINVAL;
    uart_out_bytes(data, len);
    stream_crc = crc16_update(stream_crc, data, len);
    return 0;
}

int uart_stream_end(void)
{
    if (!stream_active) return -EINVAL;
    uint8_t crc_bytes[] = { stream_crc & 0xFF, (stream_crc >> 8) & 0xFF };
    uart_out_bytes(crc_bytes, 2);
    LOG_INF("UART stream complete (CRC=0x%04X)", stream_crc);
    stream_active = false;
    return 0;
}

/* ── Chunk pool + TX thread ── */

struct rx_chunk *uart_get_free_chunk(void)    { return k_fifo_get(&free_chunks, K_FOREVER); }
void uart_return_free_chunk(struct rx_chunk *c) { k_fifo_put(&free_chunks, c); }
void uart_queue_chunk(struct rx_chunk *c)       { k_fifo_put(&pending_chunks, c); }

static void uart_tx_thread_fn(void *p1, void *p2, void *p3)
{
    uint16_t expected_total = 0;
    uint16_t received_count = 0;

    while (1) {
        struct rx_chunk *chunk = k_fifo_get(&pending_chunks, K_FOREVER);
        struct data_packet *pkt = (struct data_packet *)chunk->data;

        if (pkt->packet_idx == 0) {
            expected_total = pkt->total_packets;
            received_count = 0;
            struct image_metadata meta = {
                .tx_id = current_long_rd_id,
                .hop_count = 1,
                .seq_num = message_counter++,
            };
            uart_stream_begin(pkt->total_data_size, &meta);
        }

        uart_stream_chunk(pkt->payload, pkt->payload_len);
        received_count++;
        k_fifo_put(&free_chunks, chunk);

        if (received_count >= expected_total && expected_total > 0) {
            uart_stream_end();
            expected_total = 0;
            received_count = 0;
        }
    }
}

int uart_tx_thread_start(void)
{
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