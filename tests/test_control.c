#include "euler_control.h"
#include "euler_mission.h"
#include <stdio.h>
#include <math.h>

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        return 1; \
    } \
} while(0)

#define FLOAT_EQ(a, b) (fabsf((a) - (b)) < 0.0001f)

int test_collision_avoidance(void) {
    Vec3 self = {0.0f, 0.0f, 0.0f};
    Vec3 neighbor = {5.0f, 0.0f, 0.0f};
    
    Vec3 repulsion = euler_compute_repulsion(self, neighbor, EULER_REPULSION_K, EULER_SAFETY_RADIUS_M);
    TEST_ASSERT(repulsion.x < 0, "Repulsion should push away from neighbor (negative x)");
    TEST_ASSERT(FLOAT_EQ(repulsion.y, 0.0f), "Repulsion Y should be zero");
    TEST_ASSERT(FLOAT_EQ(repulsion.z, 0.0f), "Repulsion Z should be zero");
    
    Vec3 far_neighbor = {20.0f, 0.0f, 0.0f};
    Vec3 no_repulsion = euler_compute_repulsion(self, far_neighbor, EULER_REPULSION_K, EULER_SAFETY_RADIUS_M);
    TEST_ASSERT(FLOAT_EQ(no_repulsion.x, 0.0f), "No repulsion outside safety radius");
    
    return 0;
}

int test_pid_controller(void) {
    PIDState pid;
    euler_pid_init(&pid, EULER_PID_KP, EULER_PID_KI, EULER_PID_KD, 10.0f);
    
    TEST_ASSERT(FLOAT_EQ(pid.kp, EULER_PID_KP), "Kp should be set");
    TEST_ASSERT(FLOAT_EQ(pid.ki, EULER_PID_KI), "Ki should be set");
    TEST_ASSERT(FLOAT_EQ(pid.kd, EULER_PID_KD), "Kd should be set");
    TEST_ASSERT(FLOAT_EQ(pid.integral, 0.0f), "Integral should start at zero");
    
    float output = euler_pid_update(&pid, 1.0f, 0.1f);
    TEST_ASSERT(output > 0, "Positive error should produce positive output");
    
    euler_pid_reset(&pid);
    TEST_ASSERT(FLOAT_EQ(pid.integral, 0.0f), "Reset should clear integral");
    TEST_ASSERT(FLOAT_EQ(pid.prev_error, 0.0f), "Reset should clear previous error");
    
    return 0;
}

int test_waypoint_queue(void) {
    WaypointQueue queue = {0};
    
    Vec3 wp1 = {10.0f, 20.0f, 5.0f};
    MissionResult res = euler_waypoint_add(&queue, wp1, 2.0f, 0);
    TEST_ASSERT(res == MISSION_OK, "Adding waypoint should succeed");
    TEST_ASSERT(queue.count == 1, "Queue should have one waypoint");
    
    Vec3 wp2 = {30.0f, 40.0f, 5.0f};
    euler_waypoint_add(&queue, wp2, 2.0f, 0);
    TEST_ASSERT(queue.count == 2, "Queue should have two waypoints");
    
    Waypoint *current = euler_waypoint_current(&queue);
    TEST_ASSERT(current != NULL, "Current waypoint should exist");
    TEST_ASSERT(FLOAT_EQ(current->position.x, 10.0f), "First waypoint X");
    
    euler_waypoint_advance(&queue);
    current = euler_waypoint_current(&queue);
    TEST_ASSERT(FLOAT_EQ(current->position.x, 30.0f), "Second waypoint X after advance");
    
    Vec3 near_pos = {10.5f, 20.5f, 5.0f};
    queue.current = 0;
    TEST_ASSERT(euler_waypoint_reached(&queue, near_pos), "Should detect waypoint reached");
    
    Vec3 far_pos = {100.0f, 100.0f, 5.0f};
    TEST_ASSERT(!euler_waypoint_reached(&queue, far_pos), "Should detect waypoint not reached");
    
    euler_waypoint_clear(&queue);
    TEST_ASSERT(queue.count == 0, "Clear should empty queue");
    
    return 0;
}

int test_mission_fsm(void) {
    SwarmContext swarm = {0};
    swarm.drone_id = 1;
    swarm.uptime_ms = 0;

    MissionState state;
    euler_mission_init(&state, NULL);

    TEST_ASSERT(state.phase == MISSION_INIT, "Initial phase should be INIT");

    MissionResult res = euler_mission_start(&swarm, &state);
    TEST_ASSERT(res == MISSION_OK, "Start should succeed");
    TEST_ASSERT(state.phase == MISSION_ARMED, "Phase should be ARMED after start");

    TEST_ASSERT(euler_next_phase(MISSION_INIT) == MISSION_ARMED, "Next after INIT is ARMED");
    TEST_ASSERT(euler_next_phase(MISSION_ARMED) == MISSION_TAKEOFF, "Next after ARMED is TAKEOFF");
    TEST_ASSERT(euler_next_phase(MISSION_RTL) == MISSION_LANDING, "Next after RTL is LANDING");

    const char *name = euler_mission_phase_str(MISSION_CRUISE);
    TEST_ASSERT(name != NULL, "Phase name should exist");

    return 0;
}

int test_formation_control(void) {
    SwarmContext swarm = {0};
    swarm.drone_id = 1;
    swarm.leader_id = 0;
    swarm.self_state.pos_x = 5.0f;
    swarm.self_state.pos_y = 5.0f;
    swarm.self_state.pos_z = 10.0f;
    swarm.formation_offset = (Vec3){-5.0f, 0.0f, 0.0f};

    euler_pid_init(&swarm.pid_x, EULER_PID_KP, EULER_PID_KI, EULER_PID_KD, 10.0f);
    euler_pid_init(&swarm.pid_y, EULER_PID_KP, EULER_PID_KI, EULER_PID_KD, 10.0f);
    euler_pid_init(&swarm.pid_z, EULER_PID_KP, EULER_PID_KI, EULER_PID_KD, 10.0f);

    swarm.neighbors[0].active = true;
    swarm.neighbors[0].drone_id = 0;
    swarm.neighbors[0].last_state.pos_x = 10.0f;
    swarm.neighbors[0].last_state.pos_y = 5.0f;
    swarm.neighbors[0].last_state.pos_z = 10.0f;
    swarm.neighbor_count = 1;

    Vec3 target = euler_formation_target(&swarm);
    TEST_ASSERT(FLOAT_EQ(target.x, 5.0f), "Target X should be leader(10) + offset(-5) = 5");
    TEST_ASSERT(FLOAT_EQ(target.y, 5.0f), "Target Y should be leader(5) + offset(0) = 5");

    Vec3 velocity_cmd;
    euler_formation_control(&swarm, 0.1f, &velocity_cmd);
    float speed = euler_vec3_magnitude(velocity_cmd);
    TEST_ASSERT(speed <= EULER_MAX_SPEED_MS + 0.01f, "Velocity should not exceed max speed");

    return 0;
}

int test_compute_velocity(void) {
    SwarmContext swarm = {0};
    swarm.drone_id = 0;
    swarm.leader_id = 0;
    swarm.self_state.pos_x = 0.0f;
    swarm.self_state.pos_y = 0.0f;
    swarm.self_state.pos_z = 10.0f;

    euler_pid_init(&swarm.pid_x, EULER_PID_KP, EULER_PID_KI, EULER_PID_KD, 10.0f);
    euler_pid_init(&swarm.pid_y, EULER_PID_KP, EULER_PID_KI, EULER_PID_KD, 10.0f);
    euler_pid_init(&swarm.pid_z, EULER_PID_KP, EULER_PID_KI, EULER_PID_KD, 10.0f);

    euler_waypoint_add(&swarm.mission, (Vec3){10.0f, 0.0f, 10.0f}, 2.0f, 0);

    ControlOutput output;
    ControlResult res = euler_compute_velocity(&swarm, 0.1f, &output);

    TEST_ASSERT(res == CTRL_OK, "Compute velocity should succeed");
    TEST_ASSERT(output.velocity_cmd.x > 0.0f, "Should move towards waypoint (positive x)");

    float speed = euler_vec3_magnitude(output.velocity_cmd);
    TEST_ASSERT(speed <= EULER_MAX_SPEED_MS + 0.01f, "Velocity should not exceed max speed");
    TEST_ASSERT(output.navigation_force > 0.0f, "Navigation force should be non-zero");

    return 0;
}

