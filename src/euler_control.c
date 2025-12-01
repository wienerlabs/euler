#include "euler_control.h"
#include "euler_mission.h"
#include <math.h>
#include <stddef.h>

Vec3 euler_vec3_add(Vec3 a, Vec3 b) {
    return (Vec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 euler_vec3_sub(Vec3 a, Vec3 b) {
    return (Vec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 euler_vec3_scale(Vec3 v, float s) {
    return (Vec3){v.x * s, v.y * s, v.z * s};
}

float euler_vec3_magnitude(Vec3 v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 euler_vec3_normalize(Vec3 v) {
    float mag = euler_vec3_magnitude(v);
    if (mag < 1e-6f) return (Vec3){0.0f, 0.0f, 0.0f};
    return euler_vec3_scale(v, 1.0f / mag);
}

float euler_vec3_distance(Vec3 a, Vec3 b) {
    Vec3 diff = euler_vec3_sub(a, b);
    return euler_vec3_magnitude(diff);
}

float euler_vec3_dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 euler_compute_repulsion(Vec3 self_pos, Vec3 neighbor_pos, float k, float safety_radius) {
    Vec3 diff = euler_vec3_sub(self_pos, neighbor_pos);
    float dist = euler_vec3_magnitude(diff);
    
    if (dist >= safety_radius || dist < 1e-6f) {
        return (Vec3){0.0f, 0.0f, 0.0f};
    }
    
    float force_mag = k / (dist * dist);
    Vec3 direction = euler_vec3_normalize(diff);
    return euler_vec3_scale(direction, force_mag);
}

Vec3 euler_compute_total_avoidance(const SwarmContext *swarm) {
    Vec3 total = {0.0f, 0.0f, 0.0f};
    Vec3 self_pos = {swarm->self_state.pos_x, swarm->self_state.pos_y, swarm->self_state.pos_z};
    
    for (uint8_t i = 0; i < swarm->neighbor_count; i++) {
        if (!swarm->neighbors[i].active) continue;
        
        Vec3 neighbor_pos = {
            swarm->neighbors[i].last_state.pos_x,
            swarm->neighbors[i].last_state.pos_y,
            swarm->neighbors[i].last_state.pos_z
        };
        
        Vec3 repulsion = euler_compute_repulsion(
            self_pos, neighbor_pos, EULER_REPULSION_K, EULER_SAFETY_RADIUS_M
        );
        total = euler_vec3_add(total, repulsion);
    }
    return total;
}

Vec3 euler_compute_waypoint_attraction(Vec3 self_pos, Vec3 waypoint, float weight) {
    Vec3 to_waypoint = euler_vec3_sub(waypoint, self_pos);
    Vec3 direction = euler_vec3_normalize(to_waypoint);
    return euler_vec3_scale(direction, weight);
}

ControlResult euler_collision_check(const SwarmContext *swarm, float *min_distance) {
    Vec3 self_pos = {swarm->self_state.pos_x, swarm->self_state.pos_y, swarm->self_state.pos_z};
    *min_distance = EULER_SAFETY_RADIUS_M * 2.0f;
    
    for (uint8_t i = 0; i < swarm->neighbor_count; i++) {
        if (!swarm->neighbors[i].active) continue;
        
        Vec3 neighbor_pos = {
            swarm->neighbors[i].last_state.pos_x,
            swarm->neighbors[i].last_state.pos_y,
            swarm->neighbors[i].last_state.pos_z
        };
        
        float dist = euler_vec3_distance(self_pos, neighbor_pos);
        if (dist < *min_distance) *min_distance = dist;
        if (dist < EULER_MIN_SEPARATION_M) return CTRL_ERR_COLLISION_IMMINENT;
    }
    return CTRL_OK;
}

void euler_pid_init(PIDState *pid, float kp, float ki, float kd, float integral_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = integral_limit;
}

void euler_pid_reset(PIDState *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float euler_pid_update(PIDState *pid, float error, float dt) {
    if (dt <= 0.0f) return 0.0f;
    
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

void euler_clamp_velocity(Vec3 *velocity, float max_speed) {
    float mag = euler_vec3_magnitude(*velocity);
    if (mag > max_speed) {
        *velocity = euler_vec3_scale(euler_vec3_normalize(*velocity), max_speed);
    }
}

void euler_apply_deadband(Vec3 *velocity, float threshold) {
    if (fabsf(velocity->x) < threshold) velocity->x = 0.0f;
    if (fabsf(velocity->y) < threshold) velocity->y = 0.0f;
    if (fabsf(velocity->z) < threshold) velocity->z = 0.0f;
}

Vec3 euler_formation_target(const SwarmContext *swarm) {
    Vec3 leader_pos = {0.0f, 0.0f, 0.0f};
    bool found = false;

    for (uint8_t i = 0; i < swarm->neighbor_count; i++) {
        if (!swarm->neighbors[i].active) continue;
        if (swarm->neighbors[i].drone_id == swarm->leader_id) {
            leader_pos.x = swarm->neighbors[i].last_state.pos_x;
            leader_pos.y = swarm->neighbors[i].last_state.pos_y;
            leader_pos.z = swarm->neighbors[i].last_state.pos_z;
            found = true;
            break;
        }
    }

    if (!found) {
        return (Vec3){
            swarm->self_state.pos_x + swarm->formation_offset.x,
            swarm->self_state.pos_y + swarm->formation_offset.y,
            swarm->self_state.pos_z + swarm->formation_offset.z
        };
    }

    return euler_vec3_add(leader_pos, swarm->formation_offset);
}

ControlResult euler_formation_control(SwarmContext *swarm, float dt, Vec3 *velocity_cmd) {
    Vec3 self_pos = {swarm->self_state.pos_x, swarm->self_state.pos_y, swarm->self_state.pos_z};
    Vec3 target = euler_formation_target(swarm);

    float error_x = target.x - self_pos.x;
    float error_y = target.y - self_pos.y;
    float error_z = target.z - self_pos.z;

    velocity_cmd->x = euler_pid_update(&swarm->pid_x, error_x, dt);
    velocity_cmd->y = euler_pid_update(&swarm->pid_y, error_y, dt);
    velocity_cmd->z = euler_pid_update(&swarm->pid_z, error_z, dt);

    euler_clamp_velocity(velocity_cmd, EULER_MAX_SPEED_MS);

    return CTRL_OK;
}

ControlResult euler_compute_velocity(SwarmContext *swarm, float dt, ControlOutput *output) {
    Vec3 self_pos = {swarm->self_state.pos_x, swarm->self_state.pos_y, swarm->self_state.pos_z};

    ControlResult res = euler_collision_check(swarm, &output->min_neighbor_dist);
    output->collision_warning = (res == CTRL_ERR_COLLISION_IMMINENT);

    Vec3 avoidance = euler_compute_total_avoidance(swarm);
    output->avoidance_force = euler_vec3_magnitude(avoidance);

    Vec3 navigation = {0.0f, 0.0f, 0.0f};
    Waypoint *wp = euler_waypoint_current(&swarm->mission);
    if (wp != NULL) {
        navigation = euler_compute_waypoint_attraction(self_pos, wp->position, EULER_NAV_WEIGHT);
    }
    output->navigation_force = euler_vec3_magnitude(navigation);

    Vec3 formation = {0.0f, 0.0f, 0.0f};
    if (swarm->leader_id != swarm->drone_id) {
        euler_formation_control(swarm, dt, &formation);
    }

    output->velocity_cmd = euler_vec3_add(avoidance, navigation);
    output->velocity_cmd = euler_vec3_add(output->velocity_cmd, formation);

    output->velocity_cmd = euler_vec3_scale(output->velocity_cmd, EULER_BLEND_RATIO);

    euler_clamp_velocity(&output->velocity_cmd, EULER_MAX_SPEED_MS);
    euler_apply_deadband(&output->velocity_cmd, 0.1f);

    return res;
}

