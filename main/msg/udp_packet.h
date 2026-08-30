#pragma once
#include <stdint.h>
#include "msg/imu_msg.h"
#include "msg/motion_msg.h"
#include "msg/ultrasonic_msg.h"
#include "msg/rgb_msg.h"
#include "msg/servo_msg.h"
#include "msg/lidar_msg.h"
#include "msg/pid_msg.h"

#pragma pack(push, 1)

// 接收包：目标指令 (上位机 -> 小车)
struct UdpCommandPacket {
    uint8_t control_mode;
    // 底盘控制
    float target_vx;
    float target_vy;
    float target_wz;
    float target_speed_p;
    float target_speed_i;
    float target_speed_d;
    // 舵机与灯光控制
    float target_servo_angle;
    uint8_t target_rgb_r;
    uint8_t target_rgb_g;
    uint8_t target_rgb_b;
};

// 发送包：机器人全量状态遥测 (小车 -> 上位机)
struct UdpTelemetryPacket {
    ImuMsg imu;
    MotionMsg motion;
    UltrasonicMsg ultrasonic;
    RgbMsg rgb;
    ServoMsg servo;
    LidarMsg lidar;      // 【新增】雷达点云数据 (720字节)
    PidMsg speed;
    PidMsg potion;
};

#pragma pack(pop)
