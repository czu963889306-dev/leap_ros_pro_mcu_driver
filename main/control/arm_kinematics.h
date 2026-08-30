#pragma once

#include <stddef.h>

struct ArmModel {
    float a1_m = 0.200f;
    float a2_m = 0.105f;
    float a3_m = 0.105f;
    float a4_m = 0.180f;
    float p_m = 0.080f;
    float servo_zero_deg[4] = {0.0f, 90.0f, 90.0f, 90.0f};
    float servo_dir[4] = {1.0f, -1.0f, 1.0f, -1.0f};
    float servo_min_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float servo_max_deg[4] = {180.0f, 180.0f, 180.0f, 180.0f};
    float joint_min_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float joint_max_deg[4] = {180.0f, 90.0f, 90.0f, 90.0f};
};

struct ArmPose {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float alpha_rad = 0.0f;
    float pitch_rad = 0.0f;
};

struct ArmJoints {
    float base_rad = 0.0f;
    float shoulder_rad = 0.0f;
    float elbow_rad = 0.0f;
    float wrist_rad = 0.0f;
};

class ArmKinematics {
public:
    explicit ArmKinematics(const ArmModel &model);

    bool Inverse(const ArmPose &pose, ArmJoints *joints) const;
    bool Forward(const ArmJoints &joints, ArmPose *pose) const;
    bool JointsToServoDegrees(const ArmJoints &joints, float servo_deg[4]) const;
    bool ServoDegreesToJoints(const float servo_deg[4], ArmJoints *joints) const;
    bool ForwardFromServoDegrees(const float servo_deg[4], ArmPose *pose) const;

private:
    bool ServoInRange(size_t index, float angle_deg) const;
    bool JointInRange(size_t index, float angle_deg) const;

    ArmModel model_;
};
