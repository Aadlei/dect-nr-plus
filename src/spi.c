#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "spi.h"

LOG_MODULE_REGISTER(spi_slave, LOG_LEVEL_INF);

#define READY_PIN_NODE DT_NODELABEL(gpio0)
#define READY_PIN 6

static const struct device *gpio_dev;

#define SPI_NODE DT_NODELABEL(spi2)
#define TX_BUFFER_SIZE 1024
#define CHUNK_SIZE 4096
#define MAX_IMAGE_SIZE 32768

K_THREAD_STACK_DEFINE(spi_slave_stack, 6144);
static struct k_thread spi_slave_thread_data;
static k_tid_t spi_slave_thread_id;

static uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint8_t rx_buffer[CHUNK_SIZE];
static uint8_t image_buffer[MAX_IMAGE_SIZE]; // Full image accumulator

static size_t total_received = 0;
static size_t image_size = 0;
static bool new_image_available = false;
static bool receiving_image = false;
static bool ready_for_image = false;  // Tracks whether we should accept new images
K_MUTEX_DEFINE(image_mutex);

static struct spi_config spi_cfg = {
    .frequency = 8000000,
    .operation = SPI_OP_MODE_SLAVE |
                 SPI_TRANSFER_MSB |
                 SPI_WORD_SET(8) |
                 SPI_MODE_CPOL |
                 SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL};

static struct spi_buf tx_buf = {
    .buf = tx_buffer,
    .len = TX_BUFFER_SIZE};

static struct spi_buf_set tx_buf_set = {
    .buffers = &tx_buf,
    .count = 1};

static struct spi_buf rx_buf = {
    .buf = rx_buffer,
    .len = CHUNK_SIZE};

static struct spi_buf_set rx_buf_set = {
    .buffers = &rx_buf,
    .count = 1};
static bool find_jpeg_end(uint8_t *buffer, size_t len, size_t *end_pos)
{
    for (size_t i = 0; i < len - 1; i++)
    {
        if (buffer[i] == 0xFF && buffer[i + 1] == 0xD9)
        {
            *end_pos = i + 2; // Position after EOI marker
            return true;
        }
    }
    return false;
}

int spi_slave_init(void)
{
    const struct device *spi_dev;

    LOG_INF("Initializing SPI slave...");
    spi_dev = DEVICE_DT_GET(SPI_NODE);

    if (spi_dev == NULL)
    {
        LOG_ERR("Device pointer is NULL!");
        return -ENODEV;
    }

    if (!device_is_ready(spi_dev))
    {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }
    gpio_dev = DEVICE_DT_GET(READY_PIN_NODE);
    if (!device_is_ready(gpio_dev))
    {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }
    
    gpio_pin_configure(gpio_dev, READY_PIN, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set(gpio_dev, READY_PIN, 0);

    LOG_INF("Ready pin (P0.06) initialized");

    LOG_INF("SPI slave initialized: %s", spi_dev->name);
    return 0;
}

void spi_slave_receive_thread(void *p1, void *p2, void *p3)
{
    const struct device *spi_dev;
    int ret;
    size_t eoi_pos;

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("SPI slave thread starting...");

    spi_dev = DEVICE_DT_GET(SPI_NODE);

    if (!device_is_ready(spi_dev))
    {
        LOG_ERR("SPI device not ready");
        return;
    }

    LOG_INF("SPI slave ready, waiting for master...");
    gpio_pin_set(gpio_dev, READY_PIN, 1); 
    while (1)
    {
        memset(rx_buffer, 0, CHUNK_SIZE);

        ret = spi_transceive(spi_dev, &spi_cfg, &tx_buf_set, &rx_buf_set);

        if (ret < 0)
        {
            LOG_INF("SPI transceive failed: %d", ret);
            k_sleep(K_MSEC(1000));
            continue;
        }

        if (ret > 0)
        {
            // Check for JPEG header (start of new image)
            if (rx_buffer[0] == 0xFF && rx_buffer[1] == 0xD8)
            {
                // Wait until previous image has been consumed
                bool still_pending = true;
                while (still_pending)
                {
                    k_mutex_lock(&image_mutex, K_FOREVER);
                    still_pending = new_image_available;
                    k_mutex_unlock(&image_mutex);
                    if (still_pending)
                    {
                        k_sleep(K_MSEC(10));
                    }
                }

                LOG_INF("=== New image started ===");
                total_received = 0;
                receiving_image = true;
                memset(image_buffer, 0, MAX_IMAGE_SIZE);
            }

            if (receiving_image)
            {
                // Accumulate chunk into image buffer
                if (total_received + ret <= MAX_IMAGE_SIZE)
                {
                    memcpy(image_buffer + total_received, rx_buffer, ret);
                    total_received += ret;

                    LOG_INF("Chunk %d bytes (total: %zu/%d)",
                            ret, total_received, MAX_IMAGE_SIZE);

                    // Check if this chunk contains JPEG EOI marker
                    if (find_jpeg_end(rx_buffer, ret, &eoi_pos))
                    {
                        size_t actual_size = (total_received - ret) + eoi_pos;
                        receiving_image = false;
                        
                        // Signal NOT ready immediately
                        gpio_pin_set(gpio_dev, READY_PIN, 0);
                        
                        k_mutex_lock(&image_mutex, K_FOREVER);
                        image_size = actual_size;
                        new_image_available = true;
                        k_mutex_unlock(&image_mutex);
                        
                        LOG_INF("=== Image complete! ===");
                        LOG_INF("Total size: %zu bytes", actual_size);

                        // Print entire image as hex dump (first 128 bytes)
                        LOG_INF("First 128 bytes:");
                        LOG_HEXDUMP_INF(image_buffer,
                                        (actual_size > 128) ? 128 : actual_size,
                                        "Image data:");

                        // Print last 32 bytes (should contain FF D9)
                        if (actual_size > 32)
                        {
                            LOG_INF("Last 32 bytes:");
                            LOG_HEXDUMP_INF(image_buffer + actual_size - 32, 32,
                                            "Image end:");
                        }
                    }
                }
                else
                {
                    LOG_ERR("Image too large! Discarding.");
                    receiving_image = false;
                    total_received = 0;
                }
            }
        }
    }
}

uint8_t *spi_slave_get_image_buffer(void)
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
    
    // Signal ready for next image
    gpio_pin_set(gpio_dev, READY_PIN, 1);
    LOG_INF("Ready for next image");
}

int spi_slave_start_thread(void)
{
    spi_slave_thread_id = k_thread_create(&spi_slave_thread_data,
                                          spi_slave_stack,
                                          K_THREAD_STACK_SIZEOF(spi_slave_stack),
                                          spi_slave_receive_thread,
                                          NULL, NULL, NULL,
                                          7, 0, K_NO_WAIT);

    if (spi_slave_thread_id == NULL)
    {
        LOG_ERR("Failed to create SPI slave thread");
        return -1;
    }

    k_thread_name_set(spi_slave_thread_id, "spi_slave");
    LOG_INF("SPI slave thread started");
    return 0;
}