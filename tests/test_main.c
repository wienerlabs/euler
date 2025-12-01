#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        tests_failed++; \
        return 1; \
    } else { \
        tests_passed++; \
    } \
} while(0)

#define RUN_TEST(fn) do { \
    printf("Running %s...\n", #fn); \
    if (fn() == 0) { \
        printf("  PASS\n"); \
    } \
} while(0)

extern int test_vec3_operations(void);
extern int test_drone_state_size(void);
extern int test_ring_buffer(void);
extern int test_crc16(void);
extern int test_serialization(void);
extern int test_collision_avoidance(void);
extern int test_pid_controller(void);
extern int test_waypoint_queue(void);
extern int test_mission_fsm(void);
extern int test_formation_control(void);
extern int test_compute_velocity(void);
extern int test_mavlink_state_update(void);

int main(void) {
    printf("\n=== EULER Unit Tests ===\n\n");

    RUN_TEST(test_drone_state_size);
    RUN_TEST(test_vec3_operations);
    RUN_TEST(test_ring_buffer);
    RUN_TEST(test_crc16);
    RUN_TEST(test_serialization);
    RUN_TEST(test_collision_avoidance);
    RUN_TEST(test_pid_controller);
    RUN_TEST(test_waypoint_queue);
    RUN_TEST(test_mission_fsm);
    RUN_TEST(test_formation_control);
    RUN_TEST(test_compute_velocity);
    RUN_TEST(test_mavlink_state_update);
    
    printf("\n=== Results ===\n");
    printf("Tests run: %d\n", tests_run);
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    
    return tests_failed > 0 ? 1 : 0;
}

