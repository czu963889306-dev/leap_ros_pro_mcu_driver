#include "arm_kinematics.h"

#include <math.h>

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kRadToDeg = 180.0f / kPi;
static constexpr float kDegToRad = kPi / 180.0f;
static constexpr float kReachEpsilon = 1.0e-5f;
static constexpr float kDefaultAlphaDeg = 180.0f;
static constexpr int kMinAlphaDeg = 90;

static float clamp_cos(float value) {
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

ArmKinematics::ArmKinematics(const ArmModel &model) : model_(model) {}

bool ArmKinematics::Inverse(const ArmPose &pose, ArmJoints *joints) const {
    if (joints == nullptr ||
        !isfinite(pose.x) || !isfinite(pose.y) ||
        !isfinite(pose.z) || (!isfinite(pose.alpha_rad) && !isfinite(pose.pitch_rad))) {
        return false;
    }

    const float a1 = model_.a1_m;
    const float a2 = model_.a2_m;
    const float a3 = model_.a3_m;
    const float a4 = model_.a4_m;
    const float p = model_.p_m;
    if (a1 <= 0.0f || a2 <= 0.0f || a3 <= 0.0f || a4 <= 0.0f) {
        return false;
    }

    const float base_rad = atan2f(pose.y + p, pose.x);
    const float length = sqrtf((pose.y + p) * (pose.y + p) + pose.x * pose.x);
    const float height = pose.z;

    float requested_alpha_deg = (isfinite(pose.alpha_rad) ? pose.alpha_rad : pose.pitch_rad) * kRadToDeg;
    if (requested_alpha_deg < static_cast<float>(kMinAlphaDeg) ||
        requested_alpha_deg > kDefaultAlphaDeg) {
        requested_alpha_deg = kDefaultAlphaDeg;
    }

    for (int alpha_deg = static_cast<int>(requested_alpha_deg);
         alpha_deg >= kMinAlphaDeg;
         --alpha_deg) {
        const float alpha = static_cast<float>(alpha_deg) * kDegToRad;
        const float L = length - a4 * sinf(alpha);
        const float H = height - a4 * cosf(alpha) - a1;
        const float dist_sq = L * L + H * H;
        const float dist = sqrtf(dist_sq);

        if (dist > (a2 + a3 + kReachEpsilon) ||
            dist < (fabsf(a2 - a3) - kReachEpsilon)) {
            continue;
        }

        const float raw_cos3 = (dist_sq - a2 * a2 - a3 * a3) / (2.0f * a2 * a3);
        if (raw_cos3 < -1.0f || raw_cos3 > 1.0f) {
            continue;
        }
        const float cos3 = clamp_cos(raw_cos3);
        const float sin3 = sqrtf(fmaxf(0.0f, 1.0f - cos3 * cos3));
        const float j3_rad = atan2f(sin3, cos3);
        const float k1 = a2 + a3 * cosf(j3_rad);
        const float k2 = a3 * sinf(j3_rad);
        const float w = atan2f(k2, k1);
        const float j2_rad = atan2f(L, H) - w;
        const float j4_rad = alpha - j2_rad - j3_rad;

        const float joint_deg[4] = {
            base_rad * kRadToDeg,
            j2_rad * kRadToDeg,
            j3_rad * kRadToDeg,
            j4_rad * kRadToDeg,
        };
        bool joint_valid = true;
        for (size_t i = 0; i < 4; ++i) {
            if (!JointInRange(i, joint_deg[i])) {
                joint_valid = false;
                break;
            }
        }
        if (!joint_valid) {
            continue;
        }

        const float servo_deg[4] = {
            model_.servo_zero_deg[0] + model_.servo_dir[0] * base_rad * kRadToDeg,
            model_.servo_zero_deg[1] + model_.servo_dir[1] * j2_rad * kRadToDeg,
            model_.servo_zero_deg[2] + model_.servo_dir[2] * j3_rad * kRadToDeg,
            model_.servo_zero_deg[3] + model_.servo_dir[3] * j4_rad * kRadToDeg,
        };
        bool servo_valid = true;
        for (size_t i = 0; i < 4; ++i) {
            if (!ServoInRange(i, servo_deg[i])) {
                servo_valid = false;
                break;
            }
        }
        if (!servo_valid) {
            continue;
        }

        joints->base_rad = base_rad;
        joints->shoulder_rad = j2_rad;
        joints->elbow_rad = j3_rad;
        joints->wrist_rad = j4_rad;
        return true;
    }

    return false;
}

bool ArmKinematics::Forward(const ArmJoints &joints, ArmPose *pose) const {
    if (pose == nullptr ||
        !isfinite(joints.base_rad) || !isfinite(joints.shoulder_rad) ||
        !isfinite(joints.elbow_rad) || !isfinite(joints.wrist_rad)) {
        return false;
    }

    const float a1 = model_.a1_m;
    const float a2 = model_.a2_m;
    const float a3 = model_.a3_m;
    const float a4 = model_.a4_m;
    const float p = model_.p_m;

    const float length = a2 * sinf(joints.shoulder_rad) +
                         a3 * sinf(joints.shoulder_rad + joints.elbow_rad) +
                         a4 * sinf(joints.shoulder_rad + joints.elbow_rad + joints.wrist_rad);
    const float height = a1 + a2 * cosf(joints.shoulder_rad) +
                         a3 * cosf(joints.shoulder_rad + joints.elbow_rad) +
                         a4 * cosf(joints.shoulder_rad + joints.elbow_rad + joints.wrist_rad);
    const float alpha = joints.shoulder_rad + joints.elbow_rad + joints.wrist_rad;

    pose->x = length * cosf(joints.base_rad);
    pose->y = length * sinf(joints.base_rad) - p;
    pose->z = height;
    pose->alpha_rad = alpha;
    pose->pitch_rad = alpha;
    return true;
}

bool ArmKinematics::JointsToServoDegrees(const ArmJoints &joints, float servo_deg[4]) const {
    if (servo_deg == nullptr) {
        return false;
    }

    const float joint_rad[4] = {
        joints.base_rad,
        joints.shoulder_rad,
        joints.elbow_rad,
        joints.wrist_rad,
    };

    for (size_t i = 0; i < 4; ++i) {
        if (!isfinite(joint_rad[i]) || model_.servo_dir[i] == 0.0f) {
            return false;
        }
        if (!JointInRange(i, joint_rad[i] * kRadToDeg)) {
            return false;
        }
        servo_deg[i] = model_.servo_zero_deg[i] + model_.servo_dir[i] * joint_rad[i] * kRadToDeg;
        if (!ServoInRange(i, servo_deg[i])) {
            return false;
        }
    }
    return true;
}

bool ArmKinematics::ServoDegreesToJoints(const float servo_deg[4], ArmJoints *joints) const {
    if (servo_deg == nullptr || joints == nullptr) {
        return false;
    }

    float joint_rad[4] = {};
    for (size_t i = 0; i < 4; ++i) {
        if (!isfinite(servo_deg[i]) || model_.servo_dir[i] == 0.0f ||
            !ServoInRange(i, servo_deg[i])) {
            return false;
        }
        joint_rad[i] = (servo_deg[i] - model_.servo_zero_deg[i]) / model_.servo_dir[i] * kDegToRad;
        if (!JointInRange(i, joint_rad[i] * kRadToDeg)) {
            return false;
        }
    }

    joints->base_rad = joint_rad[0];
    joints->shoulder_rad = joint_rad[1];
    joints->elbow_rad = joint_rad[2];
    joints->wrist_rad = joint_rad[3];
    return true;
}

bool ArmKinematics::ForwardFromServoDegrees(const float servo_deg[4], ArmPose *pose) const {
    ArmJoints joints = {};
    return ServoDegreesToJoints(servo_deg, &joints) && Forward(joints, pose);
}

bool ArmKinematics::ServoInRange(size_t index, float angle_deg) const {
    if (index >= 4 || !isfinite(angle_deg)) {
        return false;
    }
    return angle_deg >= model_.servo_min_deg[index] &&
           angle_deg <= model_.servo_max_deg[index];
}

bool ArmKinematics::JointInRange(size_t index, float angle_deg) const {
    if (index >= 4 || !isfinite(angle_deg)) {
        return false;
    }
    return angle_deg >= model_.joint_min_deg[index] &&
           angle_deg <= model_.joint_max_deg[index];
}
