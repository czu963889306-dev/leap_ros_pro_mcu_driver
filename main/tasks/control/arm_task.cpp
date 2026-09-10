#include "system_globals.h"

#include <math.h>

#include "arm_kinematics.h"
#include "board.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "msg/arm_msg.h"
#include "msg/servo_msg.h"

static const char *TAG = "ARM_TASK";
static constexpr float kDegToRad = 0.017453292519943295f;
static constexpr float kRadToDeg = 57.29577951308232f;
static constexpr TickType_t kArmMoveDurationTicks = pdMS_TO_TICKS(1000);
static constexpr float kAngleEpsilonDeg = 0.01f;
static constexpr size_t kArmServoCount = 4;
static constexpr size_t kServoCount = 5;

static ServoDriver *get_servo_by_index(size_t index) {
    switch (index) {
    case 0:
        return &servo_1;
    case 1:
        return &servo_2;
    case 2:
        return &servo_3;
    case 3:
        return &servo_4;
    case 4:
        return &servo_5;
    default:
        return nullptr;
    }
}

static bool set_servo_angle(size_t index, float angle_deg) {
    ServoDriver *servo = get_servo_by_index(index);
    if (servo == nullptr || !isfinite(angle_deg)) {
        return false;
    }
    servo->SetAngle(angle_deg);
    return true;
}

static void set_all_servo_angles(const float servo_deg[kServoCount]) {
    for (size_t i = 0; i < kServoCount; ++i) {
        set_servo_angle(i, servo_deg[i]);
    }
}

struct ServoMotionPlan {
    bool active = false;
    TickType_t start_tick = 0;
    TickType_t duration_ticks = kArmMoveDurationTicks;
    float start_deg[kServoCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float target_deg[kServoCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

static void copy_servo_angles(float dst[kServoCount], const float src[kServoCount]) {
    for (size_t i = 0; i < kServoCount; ++i) {
        dst[i] = src[i];
    }
}

static bool servo_angles_close(const float lhs[kServoCount], const float rhs[kServoCount]) {
    for (size_t i = 0; i < kServoCount; ++i) {
        if (fabsf(lhs[i] - rhs[i]) > kAngleEpsilonDeg) {
            return false;
        }
    }
    return true;
}

static void start_motion_plan(
    ServoMotionPlan *plan,
    const float current_deg[kServoCount],
    const float target_deg[kServoCount],
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

static bool update_motion_plan(
    ServoMotionPlan *plan,
    TickType_t now_tick,
    float current_deg[kServoCount]) {
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
    for (size_t i = 0; i < 4; ++i) {
        current_deg[i] = plan->start_deg[i] +
                         (plan->target_deg[i] - plan->start_deg[i]) * progress;
    }
    return true;
}

static void publish_arm_state(
    const ArmKinematics &kinematics,
    const float servo_deg[kServoCount],
    bool valid) {
    if (q_arm_state == nullptr) {
        return;
    }

    ArmMsg state = {};
    state.mode = ARM_CMD_JOINT_SERVO_DEG;
    state.valid = valid;
    for (size_t i = 0; i < kArmServoCount; ++i) {
        state.servo_deg[i] = servo_deg[i];
    }

    ArmPose pose = {};
    if (valid && kinematics.ForwardFromServoDegrees(servo_deg, &pose)) {
        state.x = pose.x;
        state.y = pose.y;
        state.z = pose.z;
        state.pitch_deg = pose.alpha_rad / kDegToRad;
    } else {
        state.valid = false;
    }
    xQueueOverwrite(q_arm_state, &state);
}

static void log_ik_solution(
    const ArmKinematics &kinematics,
    const ArmPose &target,
    const ArmJoints &joints,
    const float servo_deg[kArmServoCount]) {
    const float solved_alpha_deg =
        (joints.shoulder_rad + joints.elbow_rad + joints.wrist_rad) * kRadToDeg;

    ArmPose fk_pose = {};
    const bool fk_valid = kinematics.ForwardFromServoDegrees(servo_deg, &fk_pose);
    if (fk_valid) {
        ESP_LOGI(TAG,
                 "IK solved: target=(%.3f, %.3f, %.3f) fk=(%.3f, %.3f, %.3f) alpha=%.1f joints=[%.1f, %.1f, %.1f, %.1f] servo=[%.1f, %.1f, %.1f, %.1f]",
                 target.x,
                 target.y,
                 target.z,
                 fk_pose.x,
                 fk_pose.y,
                 fk_pose.z,
                 solved_alpha_deg,
                 joints.base_rad * kRadToDeg,
                 joints.shoulder_rad * kRadToDeg,
                 joints.elbow_rad * kRadToDeg,
                 joints.wrist_rad * kRadToDeg,
                 servo_deg[0],
                 servo_deg[1],
                 servo_deg[2],
                 servo_deg[3]);
    } else {
        ESP_LOGI(TAG,
                 "IK solved: target=(%.3f, %.3f, %.3f) fk=invalid alpha=%.1f joints=[%.1f, %.1f, %.1f, %.1f] servo=[%.1f, %.1f, %.1f, %.1f]",
                 target.x,
                 target.y,
                 target.z,
                 solved_alpha_deg,
                 joints.base_rad * kRadToDeg,
                 joints.shoulder_rad * kRadToDeg,
                 joints.elbow_rad * kRadToDeg,
                 joints.wrist_rad * kRadToDeg,
                 servo_deg[0],
                 servo_deg[1],
                 servo_deg[2],
                 servo_deg[3]);
    }
}

void arm_task(void *p) {
    (void)p;

    ArmModel model = {};
    model.a1_m = 0.200f;
    model.a2_m = 0.105f;
    model.a3_m = 0.105f;
    model.a4_m = 0.180f;
    model.p_m = 0.080f;
    model.servo_zero_deg[0] = 0.0f;
    model.servo_zero_deg[1] = 90.0f;
    model.servo_zero_deg[2] = 90.0f;
    model.servo_zero_deg[3] = 90.0f;
    model.servo_dir[0] = 1.0f;
    model.servo_dir[1] = 1.0f;
    model.servo_dir[2] = 1.0f;
    model.servo_dir[3] = 1.0f;
    model.joint_min_deg[0] = 0.0f;
    model.joint_min_deg[1] = 0.0f;
    model.joint_min_deg[2] = 0.0f;
    model.joint_min_deg[3] = 0.0f;
    model.joint_max_deg[0] = 180.0f;
    model.joint_max_deg[1] = 90.0f;
    model.joint_max_deg[2] = 90.0f;
    model.joint_max_deg[3] = 90.0f;

    ArmKinematics kinematics(model);
    float current_servo_deg[kServoCount] = {
        kServoStartupAngles[0],
        kServoStartupAngles[1],
        kServoStartupAngles[2],
        kServoStartupAngles[3],
        kServoStartupAngles[4],
    };
    ServoMotionPlan motion_plan = {};
    publish_arm_state(kinematics, current_servo_deg, true);

    while (1) {
        const TickType_t now_tick = xTaskGetTickCount();
        bool state_changed = update_motion_plan(&motion_plan, now_tick, current_servo_deg);

        ArmMsg arm_cmd = {};
        if (q_arm_cmd != nullptr && xQueueReceive(q_arm_cmd, &arm_cmd, 0) == pdTRUE) {
            if (arm_cmd.mode == ARM_CMD_CARTESIAN) {
                ArmPose target = {};
                target.x = arm_cmd.x;
                target.y = arm_cmd.y;
                target.z = arm_cmd.z;
                target.alpha_rad = arm_cmd.pitch_deg * kDegToRad;
                target.pitch_rad = target.alpha_rad;

                ArmJoints joints = {};
                float target_servo_deg[kServoCount] = {};
                copy_servo_angles(target_servo_deg, current_servo_deg);
                if (kinematics.Inverse(target, &joints) &&
                    kinematics.JointsToServoDegrees(joints, target_servo_deg)) {
                    log_ik_solution(kinematics, target, joints, target_servo_deg);
                    if (!servo_angles_close(current_servo_deg, target_servo_deg)) {
                        start_motion_plan(&motion_plan, current_servo_deg, target_servo_deg, now_tick);
                    } else {
                        motion_plan.active = false;
                    }
                    state_changed = true;
                } else {
                    ESP_LOGW(TAG, "IK failed: x=%.3f y=%.3f z=%.3f pitch=%.1f",
                             target.x, target.y, target.z, arm_cmd.pitch_deg);
                    ESP_LOGW(TAG, "Model: A1=%.3f A2=%.3f A3=%.3f A4=%.3f P=%.3f",
                             model.a1_m, model.a2_m, model.a3_m, model.a4_m, model.p_m);
                    publish_arm_state(kinematics, current_servo_deg, false);
                }
            } else if (arm_cmd.mode == ARM_CMD_JOINT_SERVO_DEG) {
                float target_servo_deg[kServoCount] = {};
                copy_servo_angles(target_servo_deg, current_servo_deg);
                for (size_t i = 0; i < kArmServoCount; ++i) {
                    if (isfinite(arm_cmd.servo_deg[i])) {
                        target_servo_deg[i] = arm_cmd.servo_deg[i];
                    }
                }
                motion_plan.active = false;
                start_motion_plan(&motion_plan, current_servo_deg, target_servo_deg, now_tick);
                ESP_LOGI(TAG, "Joint servo target: servo=[%.1f, %.1f, %.1f, %.1f]",
                         target_servo_deg[0],
                         target_servo_deg[1],
                         target_servo_deg[2],
                         target_servo_deg[3]);
                state_changed = true;
            }
        }

        ServoMsg servo_cmd = {};
        if (q_servo_cmd != nullptr && xQueueReceive(q_servo_cmd, &servo_cmd, 0) == pdTRUE) {
            if (servo_cmd.channel >= 1 && servo_cmd.channel <= 5 && isfinite(servo_cmd.angle)) {
                const size_t index = static_cast<size_t>(servo_cmd.channel - 1);
                float target_servo_deg[kServoCount] = {};
                copy_servo_angles(target_servo_deg, current_servo_deg);
                target_servo_deg[index] = servo_cmd.angle;
                if (!servo_angles_close(current_servo_deg, target_servo_deg)) {
                    start_motion_plan(&motion_plan, current_servo_deg, target_servo_deg, now_tick);
                } else {
                    motion_plan.active = false;
                }
                ESP_LOGI(TAG, "Single servo target: ch=%u angle=%.1f servo=[%.1f, %.1f, %.1f, %.1f, %.1f]",
                         static_cast<unsigned>(servo_cmd.channel),
                         servo_cmd.angle,
                         target_servo_deg[0],
                         target_servo_deg[1],
                         target_servo_deg[2],
                         target_servo_deg[3],
                         target_servo_deg[4]);
                state_changed = true;
            }
        }

        if (state_changed) {
            set_all_servo_angles(current_servo_deg);
            publish_arm_state(kinematics, current_servo_deg, true);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
