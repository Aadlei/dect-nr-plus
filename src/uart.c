#include "uart.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(uart_data, LOG_LEVEL_INF);

/*
 * Image data transfer over uart1 (VCOM0) at 1 Mbaud.
 * During image transfer, logging is temporarily paused to prevent
 * log output from interleaving with binary image data.
 *
 * Frame format:
 *   [MAGIC: AA 55 AA 55] [LENGTH: 4B LE] [DATA: N bytes] [CRC16: 2B LE]
 */
#define IMAGE_FRAME_MAGIC_0  0xAA
#define IMAGE_FRAME_MAGIC_1  0x55
#define IMAGE_FRAME_MAGIC_2  0xAA
#define IMAGE_FRAME_MAGIC_3  0x55

static const struct device *uart_dev;
static bool uart_ready = false;

static uint16_t crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int uart_data_init(void)
{
    LOG_INF("Initializing uart1 for image data");

    uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("uart1 device not ready.");
        return -ENODEV;
    }

    uart_ready = true;
    LOG_INF("uart1 ready for image transfer (%s)", uart_dev->name);
    return 0;
}

bool uart_is_ready(void)
{
    return uart_ready;
}

// Send frame format: [MAGIC: 4B] [TX_ID: 2B] [HOP_COUNT: 1B] [SEQ: 4B] [LENGTH: 4B] [DATA: N bytes] [CRC16: 2B]
int uart_send_image(const uint8_t *data, uint32_t length, const struct image_metadata *meta)
{
    if (!uart_ready) {
        LOG_ERR("UART not initialized");
        return -ENOTCONN;
    }

    if (data == NULL || length == 0) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }

    LOG_INF("Sending %u bytes via uart1 (tx_id=%u, hops=%u, seq=%u)",
            length, meta->tx_id, meta->hop_count, meta->seq_num);


    /* Send magic header */
    uart_poll_out(uart_dev, IMAGE_FRAME_MAGIC_0);
    uart_poll_out(uart_dev, IMAGE_FRAME_MAGIC_1);
    uart_poll_out(uart_dev, IMAGE_FRAME_MAGIC_2);
    uart_poll_out(uart_dev, IMAGE_FRAME_MAGIC_3);

    /* Send metadata (7 bytes) */
    uart_poll_out(uart_dev, (meta->tx_id >> 0) & 0xFF);
    uart_poll_out(uart_dev, (meta->tx_id >> 8) & 0xFF);
    uart_poll_out(uart_dev, meta->hop_count);
    uart_poll_out(uart_dev, (meta->seq_num >> 0) & 0xFF);
    uart_poll_out(uart_dev, (meta->seq_num >> 8) & 0xFF);
    uart_poll_out(uart_dev, (meta->seq_num >> 16) & 0xFF);
    uart_poll_out(uart_dev, (meta->seq_num >> 24) & 0xFF);

    /* Send length (4 bytes, little-endian) */
    uart_poll_out(uart_dev, (length >> 0) & 0xFF);
    uart_poll_out(uart_dev, (length >> 8) & 0xFF);
    uart_poll_out(uart_dev, (length >> 16) & 0xFF);
    uart_poll_out(uart_dev, (length >> 24) & 0xFF);

    /* Send image data */
    for (uint32_t i = 0; i < length; i++) {
        uart_poll_out(uart_dev, data[i]);
    }

    /* CRC16 over metadata + image data */
    uint8_t meta_bytes[] = {
        (meta->tx_id >> 0) & 0xFF, (meta->tx_id >> 8) & 0xFF,
        meta->hop_count,
        (meta->seq_num >> 0) & 0xFF, (meta->seq_num >> 8) & 0xFF,
        (meta->seq_num >> 16) & 0xFF, (meta->seq_num >> 24) & 0xFF,
    };
    uint16_t crc = crc16(meta_bytes, sizeof(meta_bytes));
    /* Continue CRC over image data */
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }

    uart_poll_out(uart_dev, (crc >> 0) & 0xFF);
    uart_poll_out(uart_dev, (crc >> 8) & 0xFF);

    LOG_INF("Transfer complete (%u bytes, CRC=0x%04X)", length, crc);

    return 0;
}