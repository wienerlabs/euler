/**
 * Euler Swarm Simulation - Standalone C version
 * Simulates multiple drones with collision avoidance and formation control.
 * No external dependencies (Gazebo not required).
 */

#include "euler_comm.h"
#include "euler_control.h"
#include "euler_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>
#include <time.h>

#define MAX_DRONES 50
#define SIM_PORT 15570
#define DT 0.1f

typedef struct {
    uint8_t id;
    Vec3 pos;
    Vec3 vel;
    CommContext comm;
    SwarmContext swarm;
    pthread_t thread;
    volatile bool running;
} SimDrone;

static SimDrone g_drones[MAX_DRONES];
static int g_num_drones = 0;
static volatile bool g_running = true;

static uint32_t get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void* drone_thread(void *arg) {
    SimDrone *drone = (SimDrone *)arg;
    uint16_t seq = 0;
    
    while (drone->running && g_running) {
        uint32_t now = get_time_ms();
        
        DroneState state = {0};
        state.drone_id = drone->id;
        state.status = STATUS_ARMED | STATUS_FLYING;
        state.battery_pct = 90;
        state.pos_x = drone->pos.x;
        state.pos_y = drone->pos.y;
        state.pos_z = drone->pos.z;
        state.vel_x = drone->vel.x;
        state.vel_y = drone->vel.y;
        state.vel_z = drone->vel.z;
        state.sequence = seq++;
        
        euler_comm_broadcast(&drone->comm, &state);
        
        DroneState neighbor;
        while (euler_comm_pop_neighbor(&drone->comm, &neighbor)) {
            if (neighbor.drone_id != drone->id) {
                euler_neighbor_update(&drone->swarm, &neighbor, now);
            }
        }
        
        drone->swarm.self_state.pos_x = drone->pos.x;
        drone->swarm.self_state.pos_y = drone->pos.y;
        drone->swarm.self_state.pos_z = drone->pos.z;

        ControlOutput output;
        euler_compute_velocity(&drone->swarm, DT, &output);

        drone->vel.x += output.velocity_cmd.x * DT;
        drone->vel.y += output.velocity_cmd.y * DT;
        drone->vel.z += output.velocity_cmd.z * DT;
        
        float speed = sqrtf(drone->vel.x*drone->vel.x + 
                           drone->vel.y*drone->vel.y + 
                           drone->vel.z*drone->vel.z);
        if (speed > 5.0f) {
            float scale = 5.0f / speed;
            drone->vel.x *= scale;
            drone->vel.y *= scale;
            drone->vel.z *= scale;
        }
        
        drone->pos.x += drone->vel.x * DT;
        drone->pos.y += drone->vel.y * DT;
        drone->pos.z += drone->vel.z * DT;
        
        if (drone->pos.z < 1.0f) drone->pos.z = 1.0f;
        
        usleep(100000);
    }
    return NULL;
}

static void signal_handler(int sig) {
    (void)sig;
    g_running = false;
}

static float min_distance(void) {
    float min_dist = 1e9f;
    for (int i = 0; i < g_num_drones; i++) {
        for (int j = i + 1; j < g_num_drones; j++) {
            float dx = g_drones[i].pos.x - g_drones[j].pos.x;
            float dy = g_drones[i].pos.y - g_drones[j].pos.y;
            float dz = g_drones[i].pos.z - g_drones[j].pos.z;
            float d = sqrtf(dx*dx + dy*dy + dz*dz);
            if (d < min_dist) min_dist = d;
        }
    }
    return min_dist;
}

int main(int argc, char *argv[]) {
    g_num_drones = (argc > 1) ? atoi(argv[1]) : 10;
    if (g_num_drones > MAX_DRONES) g_num_drones = MAX_DRONES;
    
    printf("=== Euler Swarm Simulation ===\n");
    printf("Drones: %d\n\n", g_num_drones);
    
    signal(SIGINT, signal_handler);
    
    for (int i = 0; i < g_num_drones; i++) {
        g_drones[i].id = (uint8_t)i;
        g_drones[i].pos.x = (float)(i % 10) * 5.0f;
        g_drones[i].pos.y = (float)(i / 10) * 5.0f;
        g_drones[i].pos.z = 10.0f;
        g_drones[i].vel = (Vec3){0, 0, 0};
        g_drones[i].running = true;
        
        euler_comm_init(&g_drones[i].comm, SIM_PORT);
        euler_comm_start_receiver(&g_drones[i].comm);
        memset(&g_drones[i].swarm, 0, sizeof(SwarmContext));
        g_drones[i].swarm.drone_id = (uint8_t)i;
        g_drones[i].swarm.leader_id = 0;
        g_drones[i].swarm.formation_offset = (Vec3){(float)(i % 5) * 3.0f, (float)(i / 5) * 3.0f, 0.0f};

        euler_pid_init(&g_drones[i].swarm.pid_x, EULER_PID_KP, EULER_PID_KI, EULER_PID_KD, 10.0f);
        euler_pid_init(&g_drones[i].swarm.pid_y, EULER_PID_KP, EULER_PID_KI, EULER_PID_KD, 10.0f);
        euler_pid_init(&g_drones[i].swarm.pid_z, EULER_PID_KP, EULER_PID_KI, EULER_PID_KD, 10.0f);
        
        pthread_create(&g_drones[i].thread, NULL, drone_thread, &g_drones[i]);
    }
    
    printf("Running... (Ctrl+C to stop)\n\n");
    
    int step = 0;
    while (g_running) {
        sleep(1);
        step++;
        
        float md = min_distance();
        printf("[%3d] Min distance: %.2f m", step, (double)md);
        if (md < 2.0f) printf(" *** COLLISION RISK ***");
        printf("\n");
        
        if (step % 5 == 0) {
            printf("  Positions: ");
            for (int i = 0; i < (g_num_drones > 5 ? 5 : g_num_drones); i++) {
                printf("D%d(%.1f,%.1f) ", i, 
                       (double)g_drones[i].pos.x, (double)g_drones[i].pos.y);
            }
            printf("...\n");
        }
    }
    
    printf("\nShutting down...\n");
    for (int i = 0; i < g_num_drones; i++) {
        g_drones[i].running = false;
        pthread_join(g_drones[i].thread, NULL);
        euler_comm_stop_receiver(&g_drones[i].comm);
        euler_comm_shutdown(&g_drones[i].comm);
    }
    
    printf("Done.\n");
    return 0;
}

