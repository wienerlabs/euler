#include "euler_types.h"
#include <stdio.h>
#include <string.h>

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        return 1; \
    } \
} while(0)

int test_drone_state_size(void) {
    TEST_ASSERT(sizeof(DroneState) == 32, "DroneState must be exactly 32 bytes");
    TEST_ASSERT(sizeof(Vec3) == 12, "Vec3 must be 12 bytes (3 floats)");
    return 0;
}

int test_vec3_operations(void) {
    Vec3 a = {1.0f, 2.0f, 3.0f};
    Vec3 b = {4.0f, 5.0f, 6.0f};
    
    TEST_ASSERT(a.x == 1.0f, "Vec3 x component");
    TEST_ASSERT(a.y == 2.0f, "Vec3 y component");
    TEST_ASSERT(a.z == 3.0f, "Vec3 z component");
    
    (void)b;
    return 0;
}

