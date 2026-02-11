#include "uart.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_data, LOG_LEVEL_INF);

static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(nordic_app_uart));
static bool uart_ready = false;

int uart_data_init(void) 
{
    LOG_INF("Initializing UART for data transfer...");
    
    // Check if device tree node exists
    if (!DEVICE_DT_GET(DT_CHOSEN(nordic_app_uart))) {
        LOG_ERR("nordic,app-uart not defined in device tree");
        return -ENODEV;
    }
    
    LOG_INF("UART device pointer: %p", uart_dev);
    
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready: %s", uart_dev->name);
        return -ENODEV;
    }
    
    LOG_INF("UART device %s is ready", uart_dev->name);
    uart_ready = true;
    
    // Send test message to verify VCOM1 connection
    const char *test_msg = "\r\n=== UART1/VCOM1 TEST MESSAGE ===\r\n";
    for (int i = 0; test_msg[i] != '\0'; i++) {
        uart_poll_out(uart_dev, test_msg[i]);
    }
    LOG_INF("Test message sent to UART1");
    
    LOG_INF("UART initialized successfully");
    return 0;
}

bool uart_is_ready(void) 
{
    return uart_ready;
}

int uart_send_image(const uint8_t *data, uint32_t length) 
{
    if (!uart_ready) {
        LOG_ERR("UART not initialized");
        return -ENOTCONN;
    }
    
    if (data == NULL || length == 0) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }
    
    uint32_t last_log = 0;
    LOG_INF("Sending %u bytes via UART", length);
    
    // Send length header (4 bytes, little-endian)
    uart_poll_out(uart_dev, (length >> 0) & 0xFF);
    uart_poll_out(uart_dev, (length >> 8) & 0xFF);
    uart_poll_out(uart_dev, (length >> 16) & 0xFF);
    uart_poll_out(uart_dev, (length >> 24) & 0xFF);
    
    // Send all data
    for (uint32_t i = 0; i < length; i++) {
        uart_poll_out(uart_dev, data[i]);
        
        if (i - last_log >= 10240) {
            LOG_INF("Progress: %u/%u bytes (%.1f%%)", 
                    i, length, (float)i * 100.0f / length);
            last_log = i;
        }
    }
    
    LOG_INF("UART transfer complete");
    
    return 0;
}