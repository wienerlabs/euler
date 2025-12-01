#include "euler_comm.h"
#include <stdio.h>
#include <string.h>

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        return 1; \
    } \
} while(0)

int test_ring_buffer(void) {
    StateRingBuffer rb;
    euler_ring_init(&rb);
    
    TEST_ASSERT(euler_ring_empty(&rb), "Ring buffer should be empty initially");
    TEST_ASSERT(!euler_ring_full(&rb), "Ring buffer should not be full initially");
    
    DroneState state = {0};
    state.drone_id = 1;
    state.pos_x = 10.0f;
    
    TEST_ASSERT(euler_ring_push(&rb, &state), "Push should succeed");
    TEST_ASSERT(!euler_ring_empty(&rb), "Ring buffer should not be empty after push");
    
    DroneState out = {0};
    TEST_ASSERT(euler_ring_pop(&rb, &out), "Pop should succeed");
    TEST_ASSERT(out.drone_id == 1, "Drone ID should match");
    TEST_ASSERT(out.pos_x == 10.0f, "Position should match");
    TEST_ASSERT(euler_ring_empty(&rb), "Ring buffer should be empty after pop");
    
    return 0;
}

int test_crc16(void) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc1 = euler_crc16(data, sizeof(data));
    uint16_t crc2 = euler_crc16(data, sizeof(data));
    
    TEST_ASSERT(crc1 == crc2, "CRC should be deterministic");
    TEST_ASSERT(crc1 != 0, "CRC should not be zero for non-zero data");
    
    data[0] = 0x02;
    uint16_t crc3 = euler_crc16(data, sizeof(data));
    TEST_ASSERT(crc1 != crc3, "Different data should produce different CRC");
    
    return 0;
}

int test_serialization(void) {
    DroneState state = {0};
    state.drone_id = 42;
    state.status = STATUS_ARMED | STATUS_FLYING;
    state.battery_pct = 85;
    state.pos_x = 100.5f;
    state.pos_y = -50.25f;
    state.pos_z = 25.0f;
    state.vel_x = 1.5f;
    state.vel_y = -0.5f;
    state.vel_z = 0.1f;
    state.sequence = 1234;
    
    uint8_t buf[EULER_PACKET_SIZE];
    CommResult res = euler_serialize_state(&state, buf, sizeof(buf));
    TEST_ASSERT(res == COMM_OK, "Serialization should succeed");
    
    DroneState out = {0};
    res = euler_deserialize_state(buf, sizeof(buf), &out);
    TEST_ASSERT(res == COMM_OK, "Deserialization should succeed");
    
    TEST_ASSERT(out.drone_id == 42, "Drone ID should match");
    TEST_ASSERT(out.battery_pct == 85, "Battery should match");
    TEST_ASSERT(out.pos_x == 100.5f, "Position X should match");
    TEST_ASSERT(out.pos_y == -50.25f, "Position Y should match");
    
    buf[5] ^= 0xFF;
    res = euler_deserialize_state(buf, sizeof(buf), &out);
    TEST_ASSERT(res == COMM_ERR_CRC, "Corrupted data should fail CRC");
    
    return 0;
}

