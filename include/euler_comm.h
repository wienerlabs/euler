#ifndef EULER_COMM_H
#define EULER_COMM_H

#include "euler_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EULER_BROADCAST_PORT 14550
#define EULER_RING_BUFFER_SIZE 64

typedef enum {
    COMM_OK = 0,
    COMM_ERR_SOCKET,
    COMM_ERR_BIND,
    COMM_ERR_SEND,
    COMM_ERR_RECV,
    COMM_ERR_SERIALIZE,
    COMM_ERR_CRC,
    COMM_ERR_FULL
} CommResult;

typedef struct {
    DroneState buffer[EULER_RING_BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} StateRingBuffer;

typedef struct {
    int socket_fd;
    uint16_t port;
    uint32_t broadcast_addr;
    StateRingBuffer rx_buffer;
    bool running;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t crc_errors;
} CommContext;

CommResult euler_comm_init(CommContext *ctx, uint16_t port);
void euler_comm_shutdown(CommContext *ctx);
CommResult euler_comm_broadcast(CommContext *ctx, const DroneState *state);
CommResult euler_comm_receive(CommContext *ctx, DroneState *state);
bool euler_comm_has_pending(const CommContext *ctx);

uint16_t euler_crc16(const void *data, size_t len);
CommResult euler_serialize_state(const DroneState *state, uint8_t *buf, size_t len);
CommResult euler_deserialize_state(const uint8_t *buf, size_t len, DroneState *state);

void euler_ring_init(StateRingBuffer *rb);
bool euler_ring_push(StateRingBuffer *rb, const DroneState *state);
bool euler_ring_pop(StateRingBuffer *rb, DroneState *state);
bool euler_ring_empty(const StateRingBuffer *rb);
bool euler_ring_full(const StateRingBuffer *rb);

void euler_neighbor_update(SwarmContext *swarm, const DroneState *state, uint32_t now_ms);
void euler_neighbor_prune(SwarmContext *swarm, uint32_t now_ms, uint32_t timeout_ms);
NeighborEntry* euler_neighbor_find(SwarmContext *swarm, uint8_t drone_id);
Vec3 euler_neighbor_estimate_velocity(const NeighborEntry *neighbor);

#ifdef __cplusplus
}
#endif

#endif /* EULER_COMM_H */

