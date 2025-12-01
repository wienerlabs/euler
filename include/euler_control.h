#ifndef EULER_CONTROL_H
#define EULER_CONTROL_H

#include "euler_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CTRL_OK = 0,
    CTRL_ERR_INVALID,
    CTRL_ERR_COLLISION_IMMINENT,
    CTRL_ERR_NO_WAYPOINT
} ControlResult;

typedef struct {
    Vec3  velocity_cmd;
    float avoidance_force;
    float navigation_force;
    bool  collision_warning;
    float min_neighbor_dist;
} ControlOutput;

Vec3 euler_vec3_add(Vec3 a, Vec3 b);
Vec3 euler_vec3_sub(Vec3 a, Vec3 b);
Vec3 euler_vec3_scale(Vec3 v, float s);
float euler_vec3_magnitude(Vec3 v);
Vec3 euler_vec3_normalize(Vec3 v);
float euler_vec3_distance(Vec3 a, Vec3 b);
float euler_vec3_dot(Vec3 a, Vec3 b);

Vec3 euler_compute_repulsion(Vec3 self_pos, Vec3 neighbor_pos, float k, float safety_radius);
Vec3 euler_compute_total_avoidance(const SwarmContext *swarm);
Vec3 euler_compute_waypoint_attraction(Vec3 self_pos, Vec3 waypoint, float weight);
ControlResult euler_collision_check(const SwarmContext *swarm, float *min_distance);

void euler_pid_init(PIDState *pid, float kp, float ki, float kd, float integral_limit);
void euler_pid_reset(PIDState *pid);
float euler_pid_update(PIDState *pid, float error, float dt);

Vec3 euler_formation_target(const SwarmContext *swarm);
ControlResult euler_formation_control(SwarmContext *swarm, float dt, Vec3 *velocity_cmd);

ControlResult euler_compute_velocity(SwarmContext *swarm, float dt, ControlOutput *output);
void euler_clamp_velocity(Vec3 *velocity, float max_speed);
void euler_apply_deadband(Vec3 *velocity, float threshold);

#ifdef __cplusplus
}
#endif

#endif /* EULER_CONTROL_H */

