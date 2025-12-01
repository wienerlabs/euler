#include "euler_mavlink.h"
#include <string.h>

#ifdef EULER_SIMULATION
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

MavResult euler_mav_init(MavContext *ctx, const char *device, uint32_t baud) {
    memset(ctx, 0, sizeof(MavContext));
    ctx->system_id = MAVLINK_SYSTEM_ID;
    ctx->component_id = MAVLINK_COMPONENT_ID;
    
#ifdef EULER_SIMULATION
    ctx->fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (ctx->fd < 0) return MAV_ERR_OPEN;
    
    struct termios tty;
    if (tcgetattr(ctx->fd, &tty) != 0) {
        close(ctx->fd);
        return MAV_ERR_CONFIG;
    }
    
    cfsetospeed(&tty, (speed_t)baud);
    cfsetispeed(&tty, (speed_t)baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= (tcflag_t)~PARENB;
    tty.c_cflag &= (tcflag_t)~CSTOPB;
    tty.c_cflag &= (tcflag_t)~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag &= (tcflag_t)~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= (tcflag_t)~(IXON | IXOFF | IXANY);
    tty.c_oflag &= (tcflag_t)~OPOST;
    
    if (tcsetattr(ctx->fd, TCSANOW, &tty) != 0) {
        close(ctx->fd);
        return MAV_ERR_CONFIG;
    }
#else
    (void)device;
    (void)baud;
    ctx->fd = -1;
#endif
    
    ctx->connected = true;
    return MAV_OK;
}

void euler_mav_shutdown(MavContext *ctx) {
#ifdef EULER_SIMULATION
    if (ctx->fd >= 0) {
        close(ctx->fd);
        ctx->fd = -1;
    }
#endif
    ctx->connected = false;
}

MavResult euler_mav_send_heartbeat(MavContext *ctx) {
    if (!ctx->connected) return MAV_ERR_WRITE;
    
    uint8_t msg[17] = {0xFD, 9, 0, 0, 0, ctx->system_id, ctx->component_id, 0, 0, 0};
    msg[9] = 0;  // type: MAV_TYPE_GENERIC
    msg[10] = 0; // autopilot: MAV_AUTOPILOT_GENERIC
    msg[11] = 0; // base_mode
    msg[12] = 0; // custom_mode (4 bytes)
    msg[13] = 0;
    msg[14] = 0;
    msg[15] = 0;
    msg[16] = 0; // system_status
    
#ifdef EULER_SIMULATION
    ssize_t written = write(ctx->fd, msg, sizeof(msg));
    if (written != sizeof(msg)) return MAV_ERR_WRITE;
#endif
    
    ctx->msg_sent++;
    return MAV_OK;
}

MavResult euler_mav_send_position_target(MavContext *ctx, const Vec3 *pos, const Vec3 *vel) {
    if (!ctx->connected) return MAV_ERR_WRITE;
    
    MavPositionTarget target = {0};
    target.type_mask = 0x0FC7;  // ignore accel, yaw, yaw_rate
    target.x = pos->x;
    target.y = pos->y;
    target.z = pos->z;
    target.vx = vel->x;
    target.vy = vel->y;
    target.vz = vel->z;
    
    (void)target;  // TODO: serialize and send
    ctx->msg_sent++;
    return MAV_OK;
}

bool euler_mav_parse_position(const uint8_t *data, size_t len, MavLocalPosition *pos) {
    if (len < 28) return false;
    
    memcpy(&pos->time_boot_ms, data, 4);
    memcpy(&pos->x, data + 4, 4);
    memcpy(&pos->y, data + 8, 4);
    memcpy(&pos->z, data + 12, 4);
    memcpy(&pos->vx, data + 16, 4);
    memcpy(&pos->vy, data + 20, 4);
    memcpy(&pos->vz, data + 24, 4);
    return true;
}

bool euler_mav_parse_heartbeat(const uint8_t *data, size_t len, MavHeartbeat *hb) {
    if (len < 9) return false;
    
    hb->type = data[4];
    hb->autopilot = data[5];
    hb->base_mode = data[6];
    memcpy(&hb->custom_mode, data, 4);
    hb->state = data[7];
    return true;
}

void euler_mav_update_state(SwarmContext *swarm, const MavLocalPosition *pos) {
    swarm->self_state.pos_x = pos->x;
    swarm->self_state.pos_y = pos->y;
    swarm->self_state.pos_z = pos->z;
    swarm->self_state.vel_x = pos->vx;
    swarm->self_state.vel_y = pos->vy;
    swarm->self_state.vel_z = pos->vz;
}

MavResult euler_mav_receive(MavContext *ctx, MavMessageType *type, void *msg, size_t msg_size) {
    if (!ctx->connected) return MAV_ERR_READ;

#ifdef EULER_SIMULATION
    uint8_t buf[280];
    ssize_t n = read(ctx->fd, buf, sizeof(buf));
    if (n <= 0) return MAV_ERR_READ;

    if (buf[0] != 0xFD) return MAV_ERR_PARSE;

    uint8_t payload_len = buf[1];
    if (n < payload_len + 12) return MAV_ERR_PARSE;

    uint8_t msg_id_low = buf[7];
    *type = (MavMessageType)msg_id_low;

    size_t copy_len = (size_t)payload_len < msg_size ? (size_t)payload_len : msg_size;
    memcpy(msg, &buf[10], copy_len);

    ctx->msg_received++;
#else
    (void)type;
    (void)msg;
    (void)msg_size;
#endif

    return MAV_OK;
}

static MavResult send_command_long(MavContext *ctx, uint16_t cmd, float p1, float p2, float p3, float p4, float p5, float p6, float p7) {
    if (!ctx->connected) return MAV_ERR_WRITE;

    uint8_t msg[45] = {0xFD, 33, 0, 0, 0, ctx->system_id, ctx->component_id, 76, 0, 0};

    memcpy(&msg[10], &p1, 4);
    memcpy(&msg[14], &p2, 4);
    memcpy(&msg[18], &p3, 4);
    memcpy(&msg[22], &p4, 4);
    memcpy(&msg[26], &p5, 4);
    memcpy(&msg[30], &p6, 4);
    memcpy(&msg[34], &p7, 4);
    memcpy(&msg[38], &cmd, 2);
    msg[40] = 1;  // target_system
    msg[41] = 1;  // target_component
    msg[42] = 0;  // confirmation

#ifdef EULER_SIMULATION
    ssize_t written = write(ctx->fd, msg, sizeof(msg));
    if (written != sizeof(msg)) return MAV_ERR_WRITE;
#endif

    ctx->msg_sent++;
    return MAV_OK;
}

MavResult euler_mav_arm(MavContext *ctx, bool arm) {
    return send_command_long(ctx, 400, arm ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0);
}

MavResult euler_mav_set_mode(MavContext *ctx, uint32_t mode) {
    float mode_f;
    memcpy(&mode_f, &mode, sizeof(float));
    return send_command_long(ctx, 176, 1.0f, mode_f, 0, 0, 0, 0, 0);
}

MavResult euler_mav_takeoff(MavContext *ctx, float altitude) {
    return send_command_long(ctx, 22, 0, 0, 0, 0, 0, 0, altitude);
}

MavResult euler_mav_land(MavContext *ctx) {
    return send_command_long(ctx, 21, 0, 0, 0, 0, 0, 0, 0);
}

