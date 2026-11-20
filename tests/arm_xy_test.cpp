#include <cassert>
#include <cmath>

#include "arm_config.h"
#include "arm_kinematics.h"

static void check_pose(float base_deg, float x, float y) {
    namespace cfg = leap_config::arm;
    ArmModel model;
    for (size_t i = 0; i < cfg::kArmServoCount; ++i) {
        model.servo_dir[i] = cfg::kServoDir[i];
        model.servo_zero_deg[i] = cfg::kServoZeroDeg[i];
    }
    const ArmKinematics arm(model);
    // Shoulder 30°, elbow 30°, wrist 60° give 0.299317 m reach at 0.253433 m high.
    const float servo_deg[] = {base_deg, 120.0f, 120.0f, 150.0f};
    ArmPose feedback;
    assert(arm.ForwardFromServoDegrees(servo_deg, &feedback));
    assert(std::fabs(feedback.x - x) < 1e-5f);
    assert(std::fabs(feedback.y - y) < 1e-5f);
    assert(std::fabs(feedback.z - 0.253433f) < 1e-5f);

    ArmPose target;
    target.x = x;
    target.y = y;
    target.z = 0.253433f;
    target.alpha_rad = 120.0f * cfg::kDegToRad;
    ArmJoints solution;
    assert(arm.Inverse(target, &solution));
    assert(std::fabs(solution.base_rad * cfg::kRadToDeg - base_deg) < 1e-3f);
    assert(arm.Forward(solution, &feedback));
    assert(std::fabs(feedback.x - x) < 1e-5f);
    assert(std::fabs(feedback.y - y) < 1e-5f);
    assert(std::fabs(feedback.z - target.z) < 1e-5f);
}

int main() {
    check_pose(0.0f, -0.080f, 0.299317f);
    check_pose(90.0f, 0.219317f, 0.0f);
    check_pose(180.0f, -0.080f, -0.299317f);
}
