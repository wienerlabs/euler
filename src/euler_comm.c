#include "euler_comm.h"
#include <string.h>
#include <stddef.h>

#ifdef EULER_SIMULATION
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#endif

static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t euler_crc16(const void *data, size_t len) {
    const uint8_t *bytes = (const uint8_t *)data;
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc = (uint16_t)((crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ bytes[i]]);
    }
    return crc;
}

void euler_ring_init(StateRingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
}

bool euler_ring_empty(const StateRingBuffer *rb) {
    return rb->head == rb->tail;
}

bool euler_ring_full(const StateRingBuffer *rb) {
    return ((rb->head + 1) % EULER_RING_BUFFER_SIZE) == rb->tail;
}

bool euler_ring_push(StateRingBuffer *rb, const DroneState *state) {
    if (euler_ring_full(rb)) return false;
    rb->buffer[rb->head] = *state;
    rb->head = (rb->head + 1) % EULER_RING_BUFFER_SIZE;
    return true;
}

bool euler_ring_pop(StateRingBuffer *rb, DroneState *state) {
    if (euler_ring_empty(rb)) return false;
    *state = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % EULER_RING_BUFFER_SIZE;
    return true;
}

CommResult euler_serialize_state(const DroneState *state, uint8_t *buf, size_t len) {
    if (len < EULER_PACKET_SIZE) return COMM_ERR_SERIALIZE;
    memcpy(buf, state, EULER_PACKET_SIZE - 2);
    uint16_t crc = euler_crc16(buf, EULER_PACKET_SIZE - 2);
    buf[EULER_PACKET_SIZE - 2] = (uint8_t)(crc & 0xFF);
    buf[EULER_PACKET_SIZE - 1] = (uint8_t)(crc >> 8);
    return COMM_OK;
}

CommResult euler_deserialize_state(const uint8_t *buf, size_t len, DroneState *state) {
    if (len < EULER_PACKET_SIZE) return COMM_ERR_SERIALIZE;
    uint16_t received_crc = (uint16_t)(buf[EULER_PACKET_SIZE - 2] |
                                        (buf[EULER_PACKET_SIZE - 1] << 8));
    uint16_t calc_crc = euler_crc16(buf, EULER_PACKET_SIZE - 2);
    if (received_crc != calc_crc) return COMM_ERR_CRC;
    memcpy(state, buf, EULER_PACKET_SIZE);
    state->crc = received_crc;
    return COMM_OK;
}

void euler_neighbor_update(SwarmContext *swarm, const DroneState *state, uint32_t now_ms) {
    if (state->drone_id == swarm->drone_id) return;

    NeighborEntry *entry = euler_neighbor_find(swarm, state->drone_id);
    if (!entry) {
        for (uint8_t i = 0; i < EULER_MAX_DRONES; i++) {
            if (!swarm->neighbors[i].active) {
                entry = &swarm->neighbors[i];
                entry->drone_id = state->drone_id;
                entry->active = true;
                entry->history.head = 0;
                entry->history.count = 0;
                swarm->neighbor_count++;
                break;
            }
        }
    }
    if (!entry) return;

    entry->last_state = *state;
    entry->last_seen_ms = now_ms;

    PositionHistory *h = &entry->history;
    h->positions[h->head] = (Vec3){state->pos_x, state->pos_y, state->pos_z};
    h->timestamps[h->head] = now_ms;
    h->head = (uint8_t)((h->head + 1) % EULER_POSITION_HISTORY);
    if (h->count < EULER_POSITION_HISTORY) h->count++;
}

NeighborEntry* euler_neighbor_find(SwarmContext *swarm, uint8_t drone_id) {
    for (uint8_t i = 0; i < EULER_MAX_DRONES; i++) {
        if (swarm->neighbors[i].active && swarm->neighbors[i].drone_id == drone_id) {
            return &swarm->neighbors[i];
        }
    }
    return NULL;
}

void euler_neighbor_prune(SwarmContext *swarm, uint32_t now_ms, uint32_t timeout_ms) {
    for (uint8_t i = 0; i < EULER_MAX_DRONES; i++) {
        if (swarm->neighbors[i].active) {
            if ((now_ms - swarm->neighbors[i].last_seen_ms) > timeout_ms) {
                swarm->neighbors[i].active = false;
                swarm->neighbor_count--;
            }
        }
    }
}

#ifdef EULER_SIMULATION

CommResult euler_comm_init(CommContext *ctx, uint16_t port) {
    memset(ctx, 0, sizeof(CommContext));
    ctx->port = port;
    euler_ring_init(&ctx->rx_buffer);

    ctx->socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (ctx->socket_fd < 0) return COMM_ERR_SOCKET;

    int broadcast = 1;
    if (setsockopt(ctx->socket_fd, SOL_SOCKET, SO_BROADCAST,
                   &broadcast, sizeof(broadcast)) < 0) {
        close(ctx->socket_fd);
        return COMM_ERR_SOCKET;
    }

    int reuse = 1;
    setsockopt(ctx->socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
    setsockopt(ctx->socket_fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(ctx->socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(ctx->socket_fd);
        return COMM_ERR_BIND;
    }

    int flags = fcntl(ctx->socket_fd, F_GETFL, 0);
    fcntl(ctx->socket_fd, F_SETFL, flags | O_NONBLOCK);

    ctx->broadcast_addr = inet_addr("127.0.0.1");
    ctx->running = true;
    ctx->tx_count = 0;
    ctx->rx_count = 0;
    ctx->crc_errors = 0;
    return COMM_OK;
}

void euler_comm_shutdown(CommContext *ctx) {
    ctx->running = false;
    if (ctx->socket_fd >= 0) {
        close(ctx->socket_fd);
        ctx->socket_fd = -1;
    }
}

CommResult euler_comm_broadcast(CommContext *ctx, const DroneState *state) {
    if (!ctx->running || ctx->socket_fd < 0) return COMM_ERR_SOCKET;

    uint8_t buf[EULER_PACKET_SIZE];
    CommResult res = euler_serialize_state(state, buf, sizeof(buf));
    if (res != COMM_OK) return res;

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(ctx->port);
    dest.sin_addr.s_addr = ctx->broadcast_addr;

    ssize_t sent = sendto(ctx->socket_fd, buf, EULER_PACKET_SIZE, 0,
                          (struct sockaddr *)&dest, sizeof(dest));
    if (sent != EULER_PACKET_SIZE) {
        return COMM_ERR_SEND;
    }

    ctx->tx_count = ctx->tx_count + 1;
    return COMM_OK;
}

CommResult euler_comm_receive(CommContext *ctx, DroneState *state) {
    if (!ctx->running || ctx->socket_fd < 0) return COMM_ERR_SOCKET;

    uint8_t buf[EULER_PACKET_SIZE];
    struct sockaddr_in src;
    socklen_t src_len = sizeof(src);

    ssize_t received = recvfrom(ctx->socket_fd, buf, sizeof(buf), 0,
                                 (struct sockaddr *)&src, &src_len);

    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return COMM_ERR_RECV;
        }
        return COMM_ERR_RECV;
    }

    if (received != EULER_PACKET_SIZE) return COMM_ERR_RECV;

    CommResult res = euler_deserialize_state(buf, (size_t)received, state);
    if (res == COMM_OK) {
        ctx->rx_count++;
    } else if (res == COMM_ERR_CRC) {
        ctx->crc_errors++;
    }
    return res;
}

bool euler_comm_has_pending(const CommContext *ctx) {
    return !euler_ring_empty(&ctx->rx_buffer);
}

static void* receiver_thread_func(void *arg) {
    CommContext *ctx = (CommContext *)arg;

    while (ctx->running) {
        DroneState state;
        if (euler_comm_receive(ctx, &state) == COMM_OK) {
            euler_ring_push(&ctx->rx_buffer, &state);
        }
        usleep(1000);
    }
    return NULL;
}

CommResult euler_comm_start_receiver(CommContext *ctx) {
    if (ctx->thread_active) return COMM_OK;

    ctx->running = true;
    if (pthread_create(&ctx->rx_thread, NULL, receiver_thread_func, ctx) != 0) {
        return COMM_ERR_SOCKET;
    }
    ctx->thread_active = true;
    return COMM_OK;
}

void euler_comm_stop_receiver(CommContext *ctx) {
    if (!ctx->thread_active) return;

    ctx->running = false;
    pthread_join(ctx->rx_thread, NULL);
    ctx->thread_active = false;
}

bool euler_comm_pop_neighbor(CommContext *ctx, DroneState *state) {
    return euler_ring_pop(&ctx->rx_buffer, state);
}

#else

CommResult euler_comm_init(CommContext *ctx, uint16_t port) {
    (void)ctx; (void)port;
    return COMM_OK;
}

void euler_comm_shutdown(CommContext *ctx) {
    (void)ctx;
}

CommResult euler_comm_broadcast(CommContext *ctx, const DroneState *state) {
    (void)ctx; (void)state;
    return COMM_OK;
}

CommResult euler_comm_receive(CommContext *ctx, DroneState *state) {
    (void)ctx; (void)state;
    return COMM_ERR_RECV;
}

bool euler_comm_has_pending(const CommContext *ctx) {
    (void)ctx;
    return false;
}

CommResult euler_comm_start_receiver(CommContext *ctx) {
    (void)ctx;
    return COMM_OK;
}

void euler_comm_stop_receiver(CommContext *ctx) {
    (void)ctx;
}

bool euler_comm_pop_neighbor(CommContext *ctx, DroneState *state) {
    (void)ctx; (void)state;
    return false;
}

#endif
