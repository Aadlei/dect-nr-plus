#ifndef UART_DATA_H
#define UART_DATA_H

#include <zephyr/kernel.h>
#include <stdint.h>

#define MAX_PAYLOAD_SIZE    1024
#define CHUNK_BUF_SIZE      (12 + MAX_PAYLOAD_SIZE)  /* data_packet header + payload */
#define CHUNK_POOL_COUNT    16
#define ROUTING_MAX_HOPS	8

struct rx_chunk {
    void *fifo_reserved;
    uint16_t data_len;
    uint8_t  data[CHUNK_BUF_SIZE];
};

struct image_metadata {
    uint32_t tx_id;
    uint8_t hop_count;
    uint32_t seq_num;
};

struct hop_delays {
    uint8_t num_devs; // Number of devices so far
    uint32_t per_device_delay[ROUTING_MAX_HOPS]; // Time delays for each link
};

struct data_packet
{
    uint16_t packet_idx;
    uint16_t total_packets;
    size_t total_data_size;
    uint32_t timestamp_pt;
    uint32_t offset_pt_to_ft;
    struct hop_delays route_delays; // cml = cumulative
    uint16_t payload_len;
    uint8_t payload[];
} __attribute__((packed));

int uart_data_init(void);
int uart_send_image(const uint8_t *data, uint32_t length, const struct image_metadata *meta);
int uart_stream_begin(uint32_t total_length, const struct image_metadata *meta);
int uart_stream_chunk(const uint8_t *data, uint16_t len);
int uart_stream_end(void);
int uart_tx_thread_start(void);
struct rx_chunk *uart_get_free_chunk(void);
void uart_return_free_chunk(struct rx_chunk *chunk);
void uart_queue_chunk(struct rx_chunk *chunk);
bool uart_is_ready(void);
typedef void (*uart_rx_frame_cb_t)(const uint8_t *data, uint32_t len, 
                                    const struct image_metadata *meta);
void uart_rx_set_frame_callback(uart_rx_frame_cb_t cb);

int uart_handshake_init(void);
int uart_handshake_send_id(uint32_t long_rd_id);
int uart_handshake_receive_id(uint32_t *long_rd_id, int timeout_sec);

#endif /* UART_DATA_H */