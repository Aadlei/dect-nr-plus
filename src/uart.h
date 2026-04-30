#ifndef UART_DATA_H
#define UART_DATA_H

#include <zephyr/kernel.h>
#include <stdint.h>

#define ROUTING_MAX_HOPS	8
#define PACKET_HEADER_SIZE  (18 + 4 * ROUTING_MAX_HOPS + 4 * ROUTING_MAX_HOPS) // data_packet fields + hop_delays arrays
#define MAX_PAYLOAD_SIZE    1024
#define CHUNK_BUF_SIZE      (PACKET_HEADER_SIZE + MAX_PAYLOAD_SIZE) // data_packet header + payload 
#define CHUNK_POOL_COUNT    16

struct rx_chunk {
    void *fifo_reserved;
    uint16_t data_len;
    uint8_t  data[CHUNK_BUF_SIZE];
};

// NOTE: Consider removing this, its troublesome with arrays over UART
struct hop_delays {
    uint8_t num_devices_visited; // Keeps track of array indexing
    uint32_t devices_visited[ROUTING_MAX_HOPS]; // Device visited along path
    uint32_t per_link_delay[ROUTING_MAX_HOPS]; // Time delays for each link
};

// TODO: Rename UART handshake to something else
// TODO: Include delay and hop information from regular packet included here
struct packet_metadata {
    uint32_t seq_num;
    uint32_t timestamp_pt; // Same as timestamp_pt in data_packet
    uint32_t offset_pt_to_ft; // Same as offset_pt_to_ft in data_packet
    struct hop_delays route_delays;
};

// TODO: Structure this better so we can distinguish what is only necessary for PT->FT, and what needs to be present through the whole path
struct data_packet {
    uint16_t packet_idx;
    uint16_t total_packets;
    size_t total_data_size;
    uint32_t timestamp_pt; // NOTE: Timestamp for the start of TX of first chunk
    uint32_t offset_pt_to_ft; // Offset between PT and FT (DECT)
    struct hop_delays route_delays;
    uint16_t payload_len;
    uint8_t payload[];
} __attribute__((packed));

int uart_data_init(void);
int uart_send_image(const uint8_t *data, uint32_t length, const struct packet_metadata *meta);
int uart_stream_begin(uint32_t total_length, const struct packet_metadata *meta);
int uart_stream_chunk(const uint8_t *data, uint16_t len);
int uart_stream_end(void);
int uart_tx_thread_start(void);
struct rx_chunk *uart_get_free_chunk(void);
void uart_return_free_chunk(struct rx_chunk *chunk);
void uart_queue_chunk(struct rx_chunk *chunk);
bool uart_is_ready(void);
typedef void (*uart_rx_frame_cb_t)(const uint8_t *data, uint32_t len, 
                                    const struct packet_metadata *meta);
void uart_rx_set_frame_callback(uart_rx_frame_cb_t cb);

int uart_handshake_init(void);
int uart_handshake_send_id_timestamp(uint32_t long_rd_id, uint32_t timestamp);
int uart_handshake_receive_id_timestamp(uint32_t *long_rd_id, uint32_t *offset, int timeout_sec);

#endif /* UART_DATA_H */