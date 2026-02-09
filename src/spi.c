#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "spi.h"

LOG_MODULE_REGISTER(spi_slave, LOG_LEVEL_INF);

#define SPI_NODE DT_NODELABEL(spi3)
#define TX_BUFFER_SIZE 60000
#define RX_BUFFER_SIZE 60000

K_THREAD_STACK_DEFINE(spi_slave_stack, 2048);
static struct k_thread spi_slave_thread_data;
static k_tid_t spi_slave_thread_id;

static uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t image_buffer[TX_BUFFER_SIZE];
static size_t image_size = 0;
static bool new_image_available = false;

K_MUTEX_DEFINE(image_mutex);

static struct spi_config spi_cfg = {
    .frequency = 1000000,
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

int spi_slave_init(void) 
{
    const struct device *spi_dev;
    
    LOG_INF("Initializing SPI slave...");
    spi_dev = DEVICE_DT_GET(SPI_NODE);
    
    if (spi_dev == NULL) {
        LOG_ERR("Device pointer is NULL!");
        return -ENODEV;
    }
    
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }
    
    LOG_INF("SPI slave initialized: %s", spi_dev->name);
    return 0;
}

void spi_slave_receive_thread(void *p1, void *p2, void *p3)
{
    const struct device *spi_dev;
    int ret;
    
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("SPI slave thread starting...");
    
    spi_dev = DEVICE_DT_GET(SPI_NODE);
    
    if (spi_dev == NULL) {
        LOG_ERR("Device is NULL in thread!");
        return;
    }
    
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("Device not ready in thread");
        return;
    }
    
    LOG_INF("SPI slave ready, waiting for master...");
    
    while (1) 
    {
        memset(tx_buffer, 0, TX_BUFFER_SIZE);
        memset(rx_buffer, 0, RX_BUFFER_SIZE);
        
        ret = spi_transceive(spi_dev, &spi_cfg, &tx_buf_set, &rx_buf_set);
        
        if (ret < 0) {
            LOG_INF("SPI transceive failed: %d", ret);
            k_sleep(K_MSEC(1000));
            continue;
        }
        
        if (ret > 0) {
            LOG_INF("Received %d bytes", ret);
            LOG_HEXDUMP_INF(rx_buffer, ret, "RX:");
            
            k_mutex_lock(&image_mutex, K_FOREVER);
            if (ret <= TX_BUFFER_SIZE) {
                memcpy(image_buffer, rx_buffer, ret);
                image_size = ret;
                new_image_available = true;
            }
            k_mutex_unlock(&image_mutex);
        }
    }
}

uint8_t* spi_slave_get_image_buffer(void) 
{
    return image_buffer;
}

size_t spi_slave_get_image_size(void) 
{
    size_t size;
    k_mutex_lock(&image_mutex, K_FOREVER);
    size = image_size;
    k_mutex_unlock(&image_mutex);
    return size;
}

bool spi_slave_is_new_image_available(void) 
{
    bool available;
    k_mutex_lock(&image_mutex, K_FOREVER);
    available = new_image_available;
    k_mutex_unlock(&image_mutex);
    return available;
}

void spi_slave_clear_image_flag(void)
{
    k_mutex_lock(&image_mutex, K_FOREVER);
    new_image_available = false;
    k_mutex_unlock(&image_mutex);
}

int spi_slave_start_thread(void)
{
    spi_slave_thread_id = k_thread_create(&spi_slave_thread_data,
                                           spi_slave_stack,
                                           K_THREAD_STACK_SIZEOF(spi_slave_stack),
                                           spi_slave_receive_thread,
                                           NULL, NULL, NULL,
                                           7, 0, K_NO_WAIT);
    
    if (spi_slave_thread_id == NULL) {
        LOG_ERR("Failed to create SPI slave thread");
        return -1;
    }
    
    k_thread_name_set(spi_slave_thread_id, "spi_slave");
    LOG_INF("SPI slave thread started");
    return 0;
}