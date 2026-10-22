#include "microros_internal.h"

#include <string.h>

#include "esp_log.h"
#include "msg/motion_msg.h"
#include "msg/servo_msg.h"
#include "system_globals.h"

namespace microros {
namespace {

void copy_host_info_field(
    char *out,
    size_t out_size,
    const char *start,
    size_t len) {
    if (out == nullptr || out_size == 0) {
        return;
    }

    const size_t copy_len = len < (out_size - 1) ? len : (out_size - 1);
    if (copy_len > 0 && start != nullptr) {
        memcpy(out, start, copy_len);
    }
    out[copy_len] = '\0';
}

bool extract_host_info_token(
    const char *text,
    const char *key,
    char *out,
    size_t out_size) {
    if (text == nullptr || key == nullptr || out == nullptr || out_size == 0) {
        return false;
    }

    const char *start = strstr(text, key);
    if (start == nullptr) {
        out[0] = '\0';
        return false;
    }
    start += strlen(key);

    const char *end = start;
    while (*end != '\0' && *end != ' ' && *end != ',' && *end != ';') {
        ++end;
    }

    copy_host_info_field(out, out_size, start, static_cast<size_t>(end - start));
    return out[0] != '\0';
}

}  // namespace

void prepare_host_info_message_buffer() {
    memset(s_host_info_data, 0, sizeof(s_host_info_data));
    s_host_info_msg.data.data = s_host_info_data;
    s_host_info_msg.data.size = 0;
    s_host_info_msg.data.capacity = sizeof(s_host_info_data);
}

void handle_cmd_vel(const geometry_msgs__msg__Twist *msg) {
    if (msg == nullptr || system_globals().queues.motion_cmd == nullptr) {
        return;
    }

    MotionMsg cmd = {};
    cmd.source = MOTION_SRC_MICROROS;
    cmd.control_mode = 0;
    cmd.target_vx = static_cast<float>(msg->linear.x * 1000.0);
    cmd.target_vy = static_cast<float>(msg->linear.y * 1000.0);
    cmd.target_wz = static_cast<float>(msg->angular.z);
    xQueueOverwrite(system_globals().queues.motion_cmd, &cmd);
    system_globals().flags.emergency_stop = false;
}

void handle_servo_cmd_vector(const geometry_msgs__msg__Vector3 *msg) {
    if (msg == nullptr || system_globals().queues.servo_cmd == nullptr) {
        return;
    }

    const int channel = static_cast<int>(msg->x);
    if (channel < 1 || channel > 5) {
        return;
    }

    ServoMsg servo_cmd = {};
    servo_cmd.channel = static_cast<uint8_t>(channel);
    servo_cmd.angle = static_cast<float>(msg->y);
    xQueueOverwrite(system_globals().queues.servo_cmd, &servo_cmd);
}

void handle_host_info(const std_msgs__msg__String *msg) {
    if (msg == nullptr || msg->data.data == nullptr || msg->data.size == 0) {
        return;
    }

    char text[ros_cfg::kHostInfoTextCapacity] = {};
    const size_t text_len = msg->data.size < (sizeof(text) - 1)
        ? msg->data.size
        : (sizeof(text) - 1);
    memcpy(text, msg->data.data, text_len);
    text[text_len] = '\0';

    char hostname[sizeof(system_globals().runtime.target_host_info.hostname)] = {};
    char hostip[sizeof(system_globals().runtime.target_host_info.hostip)] = {};
    const bool has_hostname =
        extract_host_info_token(text, "hostname=", hostname, sizeof(hostname));
    bool has_hostip =
        extract_host_info_token(text, "hostip=", hostip, sizeof(hostip));
    if (!has_hostip) {
        has_hostip = extract_host_info_token(text, "ip=", hostip, sizeof(hostip));
    }

    if (!has_hostname || !has_hostip) {
        ESP_LOGW(kTag, "Invalid host_info text: %s", text);
        return;
    }

    system_globals().runtime.target_host_info.valid = true;
    system_globals().runtime.target_host_info.last_update_ms =
        static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    snprintf(system_globals().runtime.target_host_info.hostname, sizeof(system_globals().runtime.target_host_info.hostname), "%s", hostname);
    snprintf(system_globals().runtime.target_host_info.hostip, sizeof(system_globals().runtime.target_host_info.hostip), "%s", hostip);
    snprintf(system_globals().runtime.target_host_info.text, sizeof(system_globals().runtime.target_host_info.text), "%s", text);
    ESP_LOGI(kTag, "Host info: hostname=%s hostip=%s",
             system_globals().runtime.target_host_info.hostname,
             system_globals().runtime.target_host_info.hostip);
}

void reset_host_info_state() {
    system_globals().runtime.target_host_info.valid = false;
    system_globals().runtime.target_host_info.last_update_ms = 0;
    system_globals().runtime.target_host_info.hostname[0] = '\0';
    system_globals().runtime.target_host_info.hostip[0] = '\0';
    system_globals().runtime.target_host_info.text[0] = '\0';
}

}  // namespace microros
