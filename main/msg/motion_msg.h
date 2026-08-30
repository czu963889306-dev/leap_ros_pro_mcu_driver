#pragma once

#include <stdint.h>

enum MotionCmdSource : uint8_t {
    MOTION_SRC_NONE = 0,
    MOTION_SRC_SCRIPT = 1,
    MOTION_SRC_UDP = 2,
    MOTION_SRC_ESPNOW = 3,
    MOTION_SRC_UART = 4,
    MOTION_SRC_SYSTEM = 5,
    MOTION_SRC_MICROROS = 6,
};

struct MotionMsg {
    uint8_t source = MOTION_SRC_NONE;
    uint8_t control_mode = 0;

    // 当前状态
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    float wx = 0.0f, wy = 0.0f, wz = 0.0f;
    float x = 0.0f, y = 0.0f, yaw = 0.0f;
    float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

    // 目标指令 (速度)
    float target_vx = 0.0f, target_vy = 0.0f, target_vz = 0.0f;
    float target_wx = 0.0f, target_wy = 0.0f, target_wz = 0.0f;

    int motor_id = 0;          // 0:FL, 1:FR, 2:BL, 3:BR
    float target_motor_v = 0;  // 目标电机速度 (mm/s)
    int16_t target_motor_pwm = 0;
    int16_t target_pwm_fl = 0;
    int16_t target_pwm_fr = 0;
    int16_t target_pwm_bl = 0;
    int16_t target_pwm_br = 0;

    float vel_fl = 0.0f;
    float vel_fr = 0.0f;
    float vel_bl = 0.0f;
    float vel_br = 0.0f;
};
