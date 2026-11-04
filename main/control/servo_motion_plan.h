#pragma once

#include <math.h>

#include "arm_config.h"
#include "freertos/FreeRTOS.h"

namespace servo_motion {
namespace arm_cfg = leap_config::arm;
static constexpr TickType_t kArmMoveDurationTicks = pdMS_TO_TICKS(arm_cfg::kArmMoveDurationMs);

struct ServoMotionPlan {
    bool active = false;
    TickType_t start_tick = 0;
    TickType_t duration_ticks = kArmMoveDurationTicks;
    float start_deg[arm_cfg::kServoCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float target_deg[arm_cfg::kServoCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

inline void copy_servo_angles(float dst[arm_cfg::kServoCount], const float src[arm_cfg::kServoCount]) {
    for (size_t i = 0; i < arm_cfg::kServoCount; ++i) {
        dst[i] = src[i];
    }
}

inline bool servo_angles_close(const float lhs[arm_cfg::kServoCount], const float rhs[arm_cfg::kServoCount]) {
    for (size_t i = 0; i < arm_cfg::kServoCount; ++i) {
        if (fabsf(lhs[i] - rhs[i]) > arm_cfg::kAngleEpsilonDeg) {
            return false;
        }
    }
    return true;
}

inline void start_motion_plan(
    ServoMotionPlan *plan,
    const float current_deg[arm_cfg::kServoCount],
    const float target_deg[arm_cfg::kServoCount],
    TickType_t now_tick) {
    if (plan == nullptr) {
        return;
    }

    copy_servo_angles(plan->start_deg, current_deg);
    copy_servo_angles(plan->target_deg, target_deg);
    plan->start_tick = now_tick;
    plan->duration_ticks = kArmMoveDurationTicks;
    plan->active = true;
}

inline bool update_motion_plan(
    ServoMotionPlan *plan,
    TickType_t now_tick,
    float current_deg[arm_cfg::kServoCount]) {
    if (plan == nullptr || !plan->active) {
        return false;
    }

    const TickType_t elapsed_ticks = now_tick - plan->start_tick;
    if (elapsed_ticks >= plan->duration_ticks) {
        copy_servo_angles(current_deg, plan->target_deg);
        plan->active = false;
        return true;
    }

    const float progress = static_cast<float>(elapsed_ticks) /
                           static_cast<float>(plan->duration_ticks);
    for (size_t i = 0; i < arm_cfg::kServoCount; ++i) {
        current_deg[i] = plan->start_deg[i] +
                         (plan->target_deg[i] - plan->start_deg[i]) * progress;
    }
    return true;
}

}  // namespace servo_motion
