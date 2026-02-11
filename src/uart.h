#ifndef UART1_DATA_H
#define UART1_DATA_H

#include <zephyr/kernel.h>
#include <stdint.h>

int uart_data_init(void);
int uart_send_image(const uint8_t *data, uint32_t length);
bool uart_is_ready(void);

#endif /* UART1_DATA_H */