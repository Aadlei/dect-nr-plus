#ifndef UART_DATA_H
#define UART_DATA_H

#include <zephyr/kernel.h>
#include <stdint.h>

struct image_metadata {
    uint16_t tx_id;
    uint8_t hop_count;
    uint32_t seq_num;
};

int uart_data_init(void);
int uart_send_image(const uint8_t *data, uint32_t length, const struct image_metadata *meta);
bool uart_is_ready(void);

#endif /* UART_DATA_H */