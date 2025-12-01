#ifndef EULER_MAVLINK_H
#define EULER_MAVLINK_H

#include "euler_types.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAVLINK_BAUD_RATE 57600
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 1

typedef enum {
    MAV_OK = 0,
    MAV_ERR_OPEN,
    MAV_ERR_CONFIG,
    MAV_ERR_WRITE,
    MAV_ERR_READ,
    MAV_ERR_TIMEOUT,
    MAV_ERR_PARSE,
    MAV_ERR_CRC
} MavResult;

typedef enum {
    MAV_MSG_HEARTBEAT = 0,
    MAV_MSG_LOCAL_POSITION = 32,
    MAV_MSG_SET_POSITION_TARGET = 84,
    MAV_MSG_BATTERY_STATUS = 147,
    MAV_MSG_COMMAND_LONG = 76,
    MAV_MSG_COMMAND_ACK = 77
} MavMessageType;

typedef struct {
    uint8_t  system_id;
    uint8_t  component_id;
    uint8_t  type;
    uint8_t  autopilot;
    uint8_t  base_mode;
    uint32_t custom_mode;
    uint8_t  state;
} MavHeartbeat;

typedef struct {
    uint32_t time_boot_ms;
    float x, y, z;
    float vx, vy, vz;
} MavLocalPosition;

typedef struct {
    uint32_t time_boot_ms;
    uint16_t type_mask;
    float x, y, z;
    float vx, vy, vz;
    float afx, afy, afz;
    float yaw;
    float yaw_rate;
} MavPositionTarget;

typedef struct {
    int fd;
    uint8_t system_id;
    uint8_t component_id;
    uint8_t rx_buffer[280];
    uint16_t rx_len;
    uint8_t tx_buffer[280];
    uint32_t msg_received;
    uint32_t msg_sent;
    uint32_t parse_errors;
    bool connected;
} MavContext;

MavResult euler_mav_init(MavContext *ctx, const char *device, uint32_t baud);
void euler_mav_shutdown(MavContext *ctx);
MavResult euler_mav_send_heartbeat(MavContext *ctx);
MavResult euler_mav_send_position_target(MavContext *ctx, const Vec3 *pos, const Vec3 *vel);
MavResult euler_mav_receive(MavContext *ctx, MavMessageType *type, void *msg, size_t msg_size);
MavResult euler_mav_arm(MavContext *ctx, bool arm);
MavResult euler_mav_set_mode(MavContext *ctx, uint32_t mode);
MavResult euler_mav_takeoff(MavContext *ctx, float altitude);
MavResult euler_mav_land(MavContext *ctx);
bool euler_mav_parse_position(const uint8_t *data, size_t len, MavLocalPosition *pos);
bool euler_mav_parse_heartbeat(const uint8_t *data, size_t len, MavHeartbeat *hb);
void euler_mav_update_state(SwarmContext *swarm, const MavLocalPosition *pos);

#ifdef __cplusplus
}
#endif

#endif /* EULER_MAVLINK_H */

