#include <assert.h>
#include <initializer_list>
#include <limits>

#include "msg/servos_msg.h"
#include "servo_motion_plan.h"

using namespace servo_motion;

static void assert_angles(const float *actual, const float *expected) {
    for (size_t i = 0; i < arm_cfg::kServoCount; ++i) {
        assert(fabsf(actual[i] - expected[i]) < 0.0001f);
    }
}

int main() {
    const float target[] = {0.0f, 45.5f, 90.0f, 135.0f, 180.0f};
    ServosMsg cmd = {};
    assert(make_servos_msg(target, 5, &cmd));
    assert_angles(cmd.angles, target);
    assert(!make_servos_msg(nullptr, 5, &cmd));
    assert(!make_servos_msg(target, 5, nullptr));
    for (size_t count : {0u, 1u, 4u, 6u}) {
        assert(!make_servos_msg(target, count, &cmd));
        assert_angles(cmd.angles, target);
    }
    // Reject the entire batch, even when the invalid angle is the last one.
    for (float invalid : {-0.1f, 180.1f, std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::quiet_NaN()}) {
        for (size_t index = 0; index < 5; ++index) {
            float bad[] = {10, 20, 30, 40, 50};
            bad[index] = invalid;
            assert(!make_servos_msg(bad, 5, &cmd));
            assert_angles(cmd.angles, target);
        }
    }

    float current[] = {90, 90, 90, 90, 90};
    const float midpoint[] = {45, 67.75f, 90, 112.5f, 135};
    ServoMotionPlan plan = {};
    assert(!update_motion_plan(&plan, 0, current));
    start_motion_plan(&plan, current, cmd.angles, 100);
    assert(update_motion_plan(&plan, 600, current));
    assert_angles(current, midpoint); // Includes channel 5 at half duration.
    assert(plan.active);
    assert(update_motion_plan(&plan, 1100, current));
    assert_angles(current, target);
    assert(!plan.active);

    // New targets start from the current interpolated pose without jumping.
    const float next[] = {180, 180, 180, 180, 0};
    start_motion_plan(&plan, current, next, 1200);
    assert(update_motion_plan(&plan, 1700, current));
    float before[5];
    copy_servo_angles(before, current);
    start_motion_plan(&plan, current, target, 1700);
    assert(update_motion_plan(&plan, 1700, current));
    assert_angles(current, before);
    assert(update_motion_plan(&plan, 2700, current));
    assert_angles(current, target);

    // FreeRTOS tick wrap must not delay or skip the five-channel movement.
    start_motion_plan(&plan, current, next, UINT32_MAX - 499);
    assert(update_motion_plan(&plan, 500, current));
    assert_angles(current, next);
    assert(!plan.active);
}
