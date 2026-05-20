#ifndef UART_DATA_H
#define UART_DATA_H

#include <zephyr/kernel.h>
#include <stdint.h>

#define ROUTING_MAX_HOPS    8
#define MAX_PAYLOAD_SIZE    1024
#define CHUNK_POOL_COUNT    16

struct hop_delays {
    uint8_t  num_links;
    uint32_t devices_visited[ROUTING_MAX_HOPS];
    int32_t per_link_delay[ROUTING_MAX_HOPS];
    int8_t   per_link_rssi[ROUTING_MAX_HOPS];
};

struct data_packet {
    uint16_t packet_idx;
    uint16_t total_packets;
    uint32_t total_data_size;
    uint16_t seq_num;
    uint32_t timestamp_pt;
    int32_t  offset_pt_to_ft;
    struct hop_delays route_delays;
    uint16_t payload_len;
    uint8_t  payload[];
} __attribute__((packed));

/* Single source of truth - can never drift from the actual struct layout */
#define CHUNK_BUF_SIZE  (sizeof(struct data_packet) + MAX_PAYLOAD_SIZE)

struct rx_chunk {
    void    *fifo_reserved;
    uint16_t data_len;
    uint8_t  data[CHUNK_BUF_SIZE];
};

struct packet_metadata {
    uint32_t seq_num;
    uint32_t timestamp_pt;
    int32_t  offset_pt_to_ft;
    struct hop_delays route_delays;
};

int uart_data_init(void);
int uart_send_image(const uint8_t *data, uint32_t length, const struct packet_metadata *meta);
int uart_stream_begin(uint32_t total_length);
int uart_stream_chunk(const uint8_t *data, uint16_t len);
int uart_stream_end(const struct packet_metadata *meta);
int uart_rx_start(void);
int uart_tx_thread_start(void);
struct rx_chunk *uart_get_free_chunk(void);
void uart_return_free_chunk(struct rx_chunk *chunk);
void uart_queue_chunk(struct rx_chunk *chunk);
bool uart_is_ready(void);

typedef void (*uart_rx_frame_cb_t)(const uint8_t *data, uint32_t len,
                                    const struct packet_metadata *meta);
void uart_rx_set_frame_callback(uart_rx_frame_cb_t cb);

int uart_handshake_init(void);
int uart_handshake_send_id_timestamp(uint32_t long_rd_id);
int uart_handshake_receive_id_timestamp(uint32_t *long_rd_id, int32_t *offset, int timeout_sec);

#endif