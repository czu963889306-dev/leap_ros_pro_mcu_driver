#pragma once

#include <stddef.h>
#include <stdint.h>

namespace leap_config {
namespace arm {

static constexpr float kDegToRad = 0.017453292519943295f;
static constexpr float kRadToDeg = 57.29577951308232f;
static constexpr uint32_t kArmMoveDurationMs = 1000;
static constexpr float kAngleEpsilonDeg = 0.01f;
static constexpr size_t kArmServoCount = 4;
static constexpr size_t kServoCount = 5;
static constexpr uint32_t kServoStartupDelayMs = 2000;

static constexpr float kServoStartupAngles[kServoCount] = {
    90.0f,
    0.0f,
    180.0f,
    180.0f,
    180.0f,
};

static constexpr bool kServoReversed[kServoCount] = {
    false,
    true,
    true,
    true,
    false,
};

static constexpr float kLinkA1M = 0.200f;
static constexpr float kLinkA2M = 0.105f;
static constexpr float kLinkA3M = 0.105f;
static constexpr float kLinkA4M = 0.180f;
static constexpr float kBaseOffsetM = 0.080f;

static constexpr float kServoZeroDeg[kArmServoCount] = {
    0.0f,
    90.0f,
    90.0f,
    90.0f,
};

static constexpr float kServoDir[kArmServoCount] = {
    1.0f,
    1.0f,
    1.0f,
    1.0f,
};

static constexpr float kJointMinDeg[kArmServoCount] = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
};

static constexpr float kJointMaxDeg[kArmServoCount] = {
    180.0f,
    90.0f,
    90.0f,
    90.0f,
};

}  // namespace arm
}  // namespace leap_config
