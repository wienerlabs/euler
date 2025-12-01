#ifndef EULER_MISSION_H
#define EULER_MISSION_H

#include "euler_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MISSION_OK = 0,
    MISSION_ERR_INVALID,
    MISSION_ERR_FULL,
    MISSION_ERR_EMPTY,
    MISSION_ERR_STATE
} MissionResult;

typedef void (*MissionCallback)(SwarmContext *swarm, MissionPhase old_phase, MissionPhase new_phase);

typedef struct {
    MissionPhase phase;
    uint32_t phase_start_ms;
    uint32_t phase_timeout_ms;
    float takeoff_altitude;
    float cruise_speed;
    bool failsafe_triggered;
    MissionCallback on_transition;
} MissionState;

void euler_mission_init(MissionState *state, MissionCallback callback);
MissionResult euler_mission_start(SwarmContext *swarm, MissionState *state);
MissionResult euler_mission_abort(SwarmContext *swarm, MissionState *state);
MissionResult euler_mission_update(SwarmContext *swarm, MissionState *state, uint32_t now_ms);
const char* euler_mission_phase_str(MissionPhase phase);

MissionResult euler_waypoint_add(WaypointQueue *queue, Vec3 position, float radius, uint8_t action);
MissionResult euler_waypoint_clear(WaypointQueue *queue);
Waypoint* euler_waypoint_current(WaypointQueue *queue);
MissionResult euler_waypoint_advance(WaypointQueue *queue);
bool euler_waypoint_reached(const WaypointQueue *queue, Vec3 current_pos);

bool euler_should_transition(const SwarmContext *swarm, const MissionState *state);
MissionPhase euler_next_phase(MissionPhase current);
void euler_transition_to(SwarmContext *swarm, MissionState *state, MissionPhase next);
void euler_check_failsafe(SwarmContext *swarm, MissionState *state, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* EULER_MISSION_H */

