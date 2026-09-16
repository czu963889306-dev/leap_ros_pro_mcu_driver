#include "microros_internal.h"

#include <math.h>

#include "esp_err.h"
#include "esp_log.h"
#include "msg/arm_msg.h"
#include "msg/pid_msg.h"
#include "pid_config.h"
#include "system_globals.h"

namespace microros {
namespace {

void set_arm_pose_response(
    leap_interfaces__srv__SetArmPose_Response *res,
    bool success,
    const ArmMsg &arm_msg) {
    if (res == nullptr) {
        return;
    }
    res->success = success;
    res->x = arm_msg.x;
    res->y = arm_msg.y;
    res->z = arm_msg.z;
    res->pitch_deg = arm_msg.pitch_deg;
}

void set_speed_pid_response(
    leap_interfaces__srv__SetSpeedPid_Response *res,
    bool success,
    const PidMsg &pid_msg) {
    if (res == nullptr) {
        return;
    }
    res->success = success;
    res->kp = pid_msg.kp;
    res->ki = pid_msg.ki;
    res->kd = pid_msg.kd;
}

void get_speed_pid_response(
    leap_interfaces__srv__GetSpeedPid_Response *res,
    bool success,
    const PidMsg &pid_msg) {
    if (res == nullptr) {
        return;
    }
    res->success = success;
    res->kp = pid_msg.kp;
    res->ki = pid_msg.ki;
    res->kd = pid_msg.kd;
}

}  // namespace

void handle_set_arm_pose_service(const void *req, void *res) {
    auto *request = static_cast<const leap_interfaces__srv__SetArmPose_Request *>(req);
    auto *response = static_cast<leap_interfaces__srv__SetArmPose_Response *>(res);
    if (request == nullptr || response == nullptr || system_globals().queues.arm_cmd == nullptr ||
        !isfinite(request->x) ||
        !isfinite(request->y) ||
        !isfinite(request->z) ||
        !isfinite(request->pitch_deg)) {
        ArmMsg empty = {};
        set_arm_pose_response(response, false, empty);
        return;
    }

    ArmMsg arm_cmd = {};
    arm_cmd.mode = ARM_CMD_CARTESIAN;
    arm_cmd.valid = true;
    arm_cmd.x = request->x;
    arm_cmd.y = request->y;
    arm_cmd.z = request->z;
    arm_cmd.pitch_deg = request->pitch_deg;
    xQueueOverwrite(system_globals().queues.arm_cmd, &arm_cmd);
    set_arm_pose_response(response, true, arm_cmd);
    ESP_LOGI(kTag, "Arm pose service target: x=%.3f, y=%.3f, z=%.3f, pitch=%.1f",
             arm_cmd.x,
             arm_cmd.y,
             arm_cmd.z,
             arm_cmd.pitch_deg);
}

void handle_set_speed_pid_service(const void *req, void *res) {
    auto *request = static_cast<const leap_interfaces__srv__SetSpeedPid_Request *>(req);
    auto *response = static_cast<leap_interfaces__srv__SetSpeedPid_Response *>(res);
    if (request == nullptr || response == nullptr || system_globals().queues.speedpid_cmd == nullptr ||
        !isfinite(request->kp) || !isfinite(request->ki) || !isfinite(request->kd)) {
        set_speed_pid_response(response, false, system_globals().control.speed_pid);
        return;
    }

    PidMsg pid_msg = {};
    pid_msg.kp = request->kp;
    pid_msg.ki = request->ki;
    pid_msg.kd = request->kd;
    esp_err_t save_err = pid_save_speed_config(&pid_msg);
    if (save_err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to save speed PID config: %s", esp_err_to_name(save_err));
        set_speed_pid_response(response, false, system_globals().control.speed_pid);
        return;
    }

    system_globals().control.speed_pid = pid_msg;
    xQueueOverwrite(system_globals().queues.speedpid_cmd, &pid_msg);
    set_speed_pid_response(response, true, pid_msg);
    ESP_LOGI(kTag, "Speed PID service set and saved: kp=%.3f, ki=%.3f, kd=%.3f",
             pid_msg.kp, pid_msg.ki, pid_msg.kd);
}

void handle_get_speed_pid_service(const void *, void *res) {
    auto *response = static_cast<leap_interfaces__srv__GetSpeedPid_Response *>(res);
    get_speed_pid_response(response, true, system_globals().control.speed_pid);
}

void spin_services() {
    if (s_service_executor_initialized) {
        MICROROS_RCSOFTCHECK(rclc_executor_spin_some(&s_service_executor, RCL_MS_TO_NS(0)));
    }
}

}  // namespace microros
