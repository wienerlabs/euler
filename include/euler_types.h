#ifndef EULER_TYPES_H
#define EULER_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EULER_MAX_DRONES          50
#define EULER_MAX_WAYPOINTS       32
#define EULER_POSITION_HISTORY    10
#define EULER_PACKET_SIZE         32
#define EULER_BROADCAST_HZ        10
#define EULER_CONTROL_HZ          100
#define EULER_MAVLINK_HZ          50

#define EULER_SAFETY_RADIUS_M     10.0f
#define EULER_MIN_SEPARATION_M    3.0f
#define EULER_NOMINAL_SPACING_M   5.0f
#define EULER_WAYPOINT_RADIUS_M   2.0f
#define EULER_COMM_TIMEOUT_MS     5000

#define EULER_REPULSION_K         5.0f
#define EULER_AVOIDANCE_WEIGHT    0.7f
#define EULER_NAVIGATION_WEIGHT   0.3f

#define EULER_PID_KP              1.2f
#define EULER_PID_KI              0.05f
#define EULER_PID_KD              0.3f

typedef struct {
    float x;
    float y;
    float z;
} Vec3;

typedef struct __attribute__((packed)) {
    uint8_t  drone_id;
    uint8_t  status;
    uint8_t  battery_pct;
    uint8_t  mission_id;
    float    pos_x;
    float    pos_y;
    float    pos_z;
    float    vel_x;
    float    vel_y;
    float    vel_z;
    uint16_t sequence;
    uint16_t crc;
} DroneState;

_Static_assert(sizeof(DroneState) == EULER_PACKET_SIZE, "DroneState must be 32 bytes");

typedef enum {
    STATUS_DISARMED   = 0x00,
    STATUS_ARMED      = 0x01,
    STATUS_FLYING     = 0x02,
    STATUS_EMERGENCY  = 0x04,
    STATUS_LOW_BATT   = 0x08,
    STATUS_COMM_LOST  = 0x10
} DroneStatus;

typedef enum {
    MISSION_INIT = 0,
    MISSION_ARMED,
    MISSION_TAKEOFF,
    MISSION_CRUISE,
    MISSION_EXECUTE,
    MISSION_RTL,
    MISSION_LANDING,
    MISSION_COMPLETE
} MissionPhase;

typedef struct {
    Vec3     positions[EULER_POSITION_HISTORY];
    uint32_t timestamps[EULER_POSITION_HISTORY];
    uint8_t  head;
    uint8_t  count;
} PositionHistory;

typedef struct {
    uint8_t          drone_id;
    DroneState       last_state;
    PositionHistory  history;
    Vec3             estimated_velocity;
    uint32_t         last_seen_ms;
    bool             active;
} NeighborEntry;

typedef struct {
    Vec3     position;
    float    radius;
    uint8_t  action;
} Waypoint;

typedef struct {
    Waypoint waypoints[EULER_MAX_WAYPOINTS];
    uint8_t  count;
    uint8_t  current;
} WaypointQueue;

typedef struct {
    float integral;
    float prev_error;
    float kp;
    float ki;
    float kd;
    float integral_limit;
} PIDState;

typedef struct {
    uint8_t       drone_id;
    MissionPhase  phase;
    DroneState    self_state;
    NeighborEntry neighbors[EULER_MAX_DRONES];
    uint8_t       neighbor_count;
    WaypointQueue mission;
    Vec3          formation_offset;
    uint8_t       leader_id;
    PIDState      pid_x;
    PIDState      pid_y;
    PIDState      pid_z;
    uint32_t      uptime_ms;
} SwarmContext;

#ifdef __cplusplus
}
#endif

#endif /* EULER_TYPES_H */

