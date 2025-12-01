#include "euler_comm.h"
#include "euler_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#define TEST_PORT 15552

static CommContext g_tx_ctx;
static CommContext g_rx_ctx;
static volatile int g_received = 0;
static volatile int g_running = 1;

static void* receiver_thread(void *arg) {
    (void)arg;
    while (g_running) {
        DroneState state;
        if (euler_comm_receive(&g_rx_ctx, &state) == COMM_OK) {
            printf("  RX: drone_id=%d seq=%d pos=(%.1f,%.1f,%.1f)\n",
                   state.drone_id, state.sequence,
                   (double)state.pos_x, (double)state.pos_y, (double)state.pos_z);
            g_received++;
        }
        usleep(1000);
    }
    return NULL;
}

int main(void) {
    printf("=== UDP Two-Socket Test ===\n\n");

    if (euler_comm_init(&g_rx_ctx, TEST_PORT) != COMM_OK) {
        fprintf(stderr, "Failed to init RX\n");
        return 1;
    }
    printf("RX socket on port %d\n", TEST_PORT);

    if (euler_comm_init(&g_tx_ctx, TEST_PORT) != COMM_OK) {
        fprintf(stderr, "Failed to init TX\n");
        euler_comm_shutdown(&g_rx_ctx);
        return 1;
    }
    printf("TX socket on port %d\n", TEST_PORT);

    pthread_t rx_thread;
    pthread_create(&rx_thread, NULL, receiver_thread, NULL);

    usleep(50000);

    DroneState state = {0};
    state.drone_id = 1;
    state.status = STATUS_ARMED;
    state.battery_pct = 90;
    state.pos_x = 10.0f;
    state.pos_y = 20.0f;
    state.pos_z = 30.0f;

    printf("\nSending 5 packets...\n");
    for (int i = 0; i < 5; i++) {
        state.sequence = (uint16_t)i;
        state.pos_x += 1.0f;

        CommResult res = euler_comm_broadcast(&g_tx_ctx, &state);
        printf("  TX: seq=%d result=%d tx_count=%u\n", i, res, g_tx_ctx.tx_count);
        usleep(50000);
    }

    usleep(200000);
    g_running = 0;
    pthread_join(rx_thread, NULL);

    printf("\nResults: TX(tx=%u) RX(rx=%u crc_err=%u)\n",
           g_tx_ctx.tx_count, g_rx_ctx.rx_count, g_rx_ctx.crc_errors);

    euler_comm_shutdown(&g_tx_ctx);
    euler_comm_shutdown(&g_rx_ctx);

    if (g_received > 0) {
        printf("\n*** UDP TEST PASSED: %d packets received ***\n", g_received);
        return 0;
    } else {
        printf("\n*** UDP TEST FAILED: No packets received ***\n");
        return 1;
    }
}

