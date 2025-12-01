#include "euler_mission.h"
#include "euler_control.h"
#include <stddef.h>

static const char* PHASE_NAMES[] = {
    "INIT", "ARMED", "TAKEOFF", "CRUISE", "EXECUTE", "RTL", "LANDING", "COMPLETE"
};

const char* euler_mission_phase_str(MissionPhase phase) {
    if (phase > MISSION_COMPLETE) return "UNKNOWN";
    return PHASE_NAMES[phase];
}

void euler_mission_init(MissionState *state, MissionCallback callback) {
    state->phase = MISSION_INIT;
    state->phase_start_ms = 0;
    state->phase_timeout_ms = 30000;
    state->takeoff_altitude = 10.0f;
    state->cruise_speed = 5.0f;
    state->failsafe_triggered = false;
    state->on_transition = callback;
}

MissionResult euler_mission_start(SwarmContext *swarm, MissionState *state) {
    if (state->phase != MISSION_INIT) return MISSION_ERR_STATE;
    euler_transition_to(swarm, state, MISSION_ARMED);
    return MISSION_OK;
}

MissionResult euler_mission_abort(SwarmContext *swarm, MissionState *state) {
    euler_transition_to(swarm, state, MISSION_RTL);
    return MISSION_OK;
}

MissionPhase euler_next_phase(MissionPhase current) {
    switch (current) {
        case MISSION_INIT:    return MISSION_ARMED;
        case MISSION_ARMED:   return MISSION_TAKEOFF;
        case MISSION_TAKEOFF: return MISSION_CRUISE;
        case MISSION_CRUISE:  return MISSION_EXECUTE;
        case MISSION_EXECUTE: return MISSION_RTL;
        case MISSION_RTL:     return MISSION_LANDING;
        case MISSION_LANDING: return MISSION_COMPLETE;
        case MISSION_COMPLETE: return MISSION_COMPLETE;
        default: return MISSION_INIT;
    }
}

void euler_transition_to(SwarmContext *swarm, MissionState *state, MissionPhase next) {
    MissionPhase old = state->phase;
    state->phase = next;
    state->phase_start_ms = swarm->uptime_ms;
    swarm->phase = next;
    
    if (state->on_transition) {
        state->on_transition(swarm, old, next);
    }
}

bool euler_should_transition(const SwarmContext *swarm, const MissionState *state) {
    Vec3 self_pos = {swarm->self_state.pos_x, swarm->self_state.pos_y, swarm->self_state.pos_z};
    
    switch (state->phase) {
        case MISSION_ARMED:
            return (swarm->self_state.status & STATUS_ARMED) != 0;
            
        case MISSION_TAKEOFF:
            return self_pos.z >= state->takeoff_altitude * 0.95f;
            
        case MISSION_CRUISE:
            return swarm->mission.count > 0;
            
        case MISSION_EXECUTE:
            return swarm->mission.current >= swarm->mission.count;
            
        case MISSION_RTL: {
            Vec3 home = {0, 0, state->takeoff_altitude};
            return euler_vec3_distance(self_pos, home) < EULER_WAYPOINT_RADIUS_M;
        }
            
        case MISSION_LANDING:
            return self_pos.z < 0.5f;
            
        default:
            return false;
    }
}

MissionResult euler_mission_update(SwarmContext *swarm, MissionState *state, uint32_t now_ms) {
    swarm->uptime_ms = now_ms;
    euler_check_failsafe(swarm, state, now_ms);
    
    if (state->failsafe_triggered) return MISSION_OK;
    if (state->phase == MISSION_COMPLETE) return MISSION_OK;
    
    if (euler_should_transition(swarm, state)) {
        euler_transition_to(swarm, state, euler_next_phase(state->phase));
    }
    
    return MISSION_OK;
}

void euler_check_failsafe(SwarmContext *swarm, MissionState *state, uint32_t now_ms) {
    if (state->failsafe_triggered) return;
    
    if ((swarm->self_state.status & STATUS_EMERGENCY) ||
        (swarm->self_state.status & STATUS_LOW_BATT) ||
        (swarm->self_state.status & STATUS_COMM_LOST)) {
        state->failsafe_triggered = true;
        euler_transition_to(swarm, state, MISSION_RTL);
    }
    
    uint32_t elapsed = now_ms - state->phase_start_ms;
    if (elapsed > state->phase_timeout_ms && state->phase != MISSION_COMPLETE) {
        state->failsafe_triggered = true;
        euler_transition_to(swarm, state, MISSION_RTL);
    }
}

MissionResult euler_waypoint_add(WaypointQueue *queue, Vec3 position, float radius, uint8_t action) {
    if (queue->count >= EULER_MAX_WAYPOINTS) return MISSION_ERR_FULL;
    queue->waypoints[queue->count].position = position;
    queue->waypoints[queue->count].radius = radius;
    queue->waypoints[queue->count].action = action;
    queue->count++;
    return MISSION_OK;
}

MissionResult euler_waypoint_clear(WaypointQueue *queue) {
    queue->count = 0;
    queue->current = 0;
    return MISSION_OK;
}

Waypoint* euler_waypoint_current(WaypointQueue *queue) {
    if (queue->current >= queue->count) return NULL;
    return &queue->waypoints[queue->current];
}

MissionResult euler_waypoint_advance(WaypointQueue *queue) {
    if (queue->current >= queue->count) return MISSION_ERR_EMPTY;
    queue->current++;
    return MISSION_OK;
}

bool euler_waypoint_reached(const WaypointQueue *queue, Vec3 current_pos) {
    if (queue->current >= queue->count) return false;
    const Waypoint *wp = &queue->waypoints[queue->current];
    return euler_vec3_distance(current_pos, wp->position) < wp->radius;
}

