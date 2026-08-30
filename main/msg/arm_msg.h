#pragma once

#include <stdint.h>

enum ArmCommandMode : uint8_t {
    ARM_CMD_CARTESIAN = 0,
    ARM_CMD_JOINT_SERVO_DEG = 1,
};

#pragma pack(push, 1)
struct ArmMsg {
    uint8_t mode = ARM_CMD_CARTESIAN;
    bool valid = false;
    float x = 0.16f;
    float y = 0.0f;
    float z = 0.12f;
    float pitch_deg = 0.0f;
    float servo_deg[4] = {90.0f, 140.0f, 180.0f, 0.0f};
};
#pragma pack(pop)
