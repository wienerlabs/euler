#include "euler_comm.h"
#include "euler_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define TEST_PORT 15553

static int test_basic_udp(void) {
    printf("=== Test 1: Basic UDP Send/Receive ===\n");

    CommContext tx_ctx, rx_ctx;

    if (euler_comm_init(&rx_ctx, TEST_PORT) != COMM_OK) {
        printf("FAIL: RX init\n");
        return 1;
    }
    if (euler_comm_init(&tx_ctx, TEST_PORT) != COMM_OK) {
        euler_comm_shutdown(&rx_ctx);
        printf("FAIL: TX init\n");
        return 1;
    }

    DroneState tx_state = {0};
    tx_state.drone_id = 42;
    tx_state.status = STATUS_ARMED;
    tx_state.pos_x = 100.0f;
    tx_state.pos_y = 200.0f;
    tx_state.pos_z = 50.0f;

    euler_comm_broadcast(&tx_ctx, &tx_state);
    usleep(10000);

    DroneState rx_state;
    CommResult res = euler_comm_receive(&rx_ctx, &rx_state);

    euler_comm_shutdown(&tx_ctx);
    euler_comm_shutdown(&rx_ctx);

    if (res != COMM_OK) {
        printf("FAIL: receive returned %d\n", res);
        return 1;
    }
    if (rx_state.drone_id != 42 || rx_state.pos_x != 100.0f) {
        printf("FAIL: data mismatch\n");
        return 1;
    }

    printf("PASS\n\n");
    return 0;
}

static int test_threaded_receiver(void) {
    printf("=== Test 2: Threaded Receiver ===\n");

    CommContext tx_ctx, rx_ctx;

    if (euler_comm_init(&rx_ctx, TEST_PORT + 1) != COMM_OK) {
        printf("FAIL: RX init\n");
        return 1;
    }
    if (euler_comm_init(&tx_ctx, TEST_PORT + 1) != COMM_OK) {
        euler_comm_shutdown(&rx_ctx);
        printf("FAIL: TX init\n");
        return 1;
    }

    if (euler_comm_start_receiver(&rx_ctx) != COMM_OK) {
        printf("FAIL: start receiver\n");
        euler_comm_shutdown(&tx_ctx);
        euler_comm_shutdown(&rx_ctx);
        return 1;
    }

    usleep(10000);

    DroneState tx_state = {0};
    for (int i = 0; i < 5; i++) {
        tx_state.drone_id = (uint8_t)(i + 1);
        tx_state.sequence = (uint16_t)i;
        tx_state.pos_x = (float)i * 10.0f;
        euler_comm_broadcast(&tx_ctx, &tx_state);
        usleep(20000);
    }

    usleep(100000);
    euler_comm_stop_receiver(&rx_ctx);

    int count = 0;
    DroneState rx_state;
    while (euler_comm_pop_neighbor(&rx_ctx, &rx_state)) {
        printf("  Popped: drone_id=%d seq=%d\n", rx_state.drone_id, rx_state.sequence);
        count++;
    }

    euler_comm_shutdown(&tx_ctx);
    euler_comm_shutdown(&rx_ctx);

    if (count >= 3) {
        printf("PASS (received %d packets)\n\n", count);
        return 0;
    } else {
        printf("FAIL: only %d packets received\n", count);
        return 1;
    }
}

int main(void) {
    printf("\n=== EULER UDP Communication Tests ===\n\n");

    int failures = 0;
    failures += test_basic_udp();
    failures += test_threaded_receiver();

    printf("=== Summary: %d failures ===\n", failures);
    return failures > 0 ? 1 : 0;
}

