#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


int spi_slave_init(void);
void spi_slave_receive_thread(void *p1, void *p2, void *p3);
uint8_t* spi_slave_get_image_buffer(void);
size_t spi_slave_get_image_size(void);
bool spi_slave_is_new_image_available(void);
void spi_slave_clear_image_flag(void);
int spi_slave_start_thread(void);

#endif // SPI_SLAVE_H