#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_slave, LOG_LEVEL_INF);

// SPI Device configuration, from the overlay file.
#define SPI_NODE DT_NODELABEL(spi0)

#define TX_BUFFER_SIZE 1024
#define RX_BUFFER_SIZE 1024

#pragma region SPI Configuration
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint8_t rx_buffer[RX_BUFFER_SIZE];


static struct spi_config spi_cfg = {
    .frequency = 1000000, // 1 MHz
    .operation = SPI_OP_MODE_SLAVE | 
                 SPI_TRANSFER_MSB  | 
                 SPI_WORD_SET(8)   |
                 SPI_MODE_CPOL     |
                 SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL
};

static struct spi_buf tx_buf = {
    .buf = tx_buffer,
    .len = TX_BUFFER_SIZE
};

static struct spi_buf_set tx_buf_set = {
    .buffers = &tx_buf,
    .count = 1
};

static struct spi_buf rx_buf = {
    .buf = rx_buffer,
    .len = RX_BUFFER_SIZE
};

static struct spi_buf_set rx_buf_set = {
    .buffers = &rx_buf,
    .count = 1
};

#pragma endregion


// SPI slave thread function to receive data from the master device.
void spi_slave_receive_thread(void) 
{
    const struct device *spi_dev;
    int ret;

    spi_dev = DEVICE_DT_GET(SPI_NODE);
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
        return;
    }

    LOG_INF("SPI slave device ready, waiting for data from master...");
        while (1) 
        {
            memset(tx_buffer, 0, TX_BUFFER_SIZE);
            memset(rx_buffer, 0, RX_BUFFER_SIZE);
            
            ret = spi_transceive(spi_dev, NULL, &tx_buf_set, &rx_buf_set);
            
            if (ret < 0) {
                LOG_ERR("SPI transceive failed: %d", ret);
                k_sleep(K_MSEC(100));
                continue;
            } else {
                LOG_INF("Received %d bytes from master", rx_buf.len);
            }

        }
    }
}

K_THREAD_DEFINE(spi_slave_thread_id, 2048,
                spi_slave_receive_thread, NULL, NULL, NULL,
                7, 0, 0);