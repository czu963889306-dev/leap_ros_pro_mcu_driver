#include "microros_internal.h"

#include <math.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "msg/arm_msg.h"
#include "msg/imu_msg.h"
#include "msg/lidar_msg.h"
#include "msg/motion_msg.h"
#include "rmw_microros/rmw_microros.h"
#include "system_globals.h"
#include "wifi_app.h"

namespace microros {
namespace {

bool append_char(char *out, size_t out_size, size_t *offset, char ch) {
    if (out == nullptr || offset == nullptr || *offset + 1 >= out_size) {
        return false;
    }
    out[(*offset)++] = ch;
    out[*offset] = '\0';
    return true;
}

bool append_literal(char *out, size_t out_size, size_t *offset, const char *text) {
    if (text == nullptr) {
        return false;
    }
    while (*text != '\0') {
        if (!append_char(out, out_size, offset, *text++)) {
            return false;
        }
    }
    return true;
}

bool append_json_escaped(char *out, size_t out_size, size_t *offset, const char *text) {
    if (text == nullptr) {
        return false;
    }
    while (*text != '\0') {
        const char ch = *text++;
        if (ch == '"' || ch == '\\') {
            if (!append_char(out, out_size, offset, '\\')) {
                return false;
            }
        }
        if (!append_char(out, out_size, offset, ch)) {
            return false;
        }
    }
    return true;
}

bool build_wifi_config_payload(char *out, size_t out_size, char *ssid_out, size_t ssid_out_size) {
    if (out == nullptr || out_size == 0 || ssid_out == nullptr || ssid_out_size == 0) {
        return false;
    }

    char ssid[33] = {};
    char password[65] = {};
    if (wifi_app().LoadStaCredentialsOrDefault(ssid, sizeof(ssid), password, sizeof(password)) != ESP_OK ||
        ssid[0] == '\0') {
        return false;
    }

    size_t offset = 0;
    out[0] = '\0';
    bool ok = append_literal(out, out_size, &offset, "{\"ssid\":\"");
    ok = ok && append_json_escaped(out, out_size, &offset, ssid);
    ok = ok && append_literal(out, out_size, &offset, "\",\"password\":\"");
    ok = ok && append_json_escaped(out, out_size, &offset, password);
    ok = ok && append_literal(out, out_size, &offset, "\"}");
    if (!ok) {
        out[0] = '\0';
        return false;
    }

    snprintf(ssid_out, ssid_out_size, "%s", ssid);
    return true;
}

void set_stamp(std_msgs__msg__Header *header, int64_t stamp_ms) {
    header->stamp.sec = static_cast<int32_t>(stamp_ms / 1000);
    header->stamp.nanosec = static_cast<uint32_t>((stamp_ms % 1000) * 1000000);
}

void publish_odom(int64_t stamp_ms) {
    MotionMsg motion = {};
    if (system_globals().queues.motion_state == nullptr || xQueuePeek(system_globals().queues.motion_state, &motion, 0) != pdTRUE) {
        return;
    }

    set_stamp(&s_odom_msg.header, stamp_ms);
    s_odom_msg.pose.pose.position.x = motion.x / 1000.0;
    s_odom_msg.pose.pose.position.y = motion.y / 1000.0;
    s_odom_msg.pose.pose.position.z = 0.0;
    s_odom_msg.pose.pose.orientation.w = motion.qw;
    s_odom_msg.pose.pose.orientation.x = motion.qx;
    s_odom_msg.pose.pose.orientation.y = motion.qy;
    s_odom_msg.pose.pose.orientation.z = motion.qz;
    s_odom_msg.twist.twist.linear.x = motion.vx / 1000.0;
    s_odom_msg.twist.twist.linear.y = motion.vy / 1000.0;
    s_odom_msg.twist.twist.angular.z = motion.wz;
    MICROROS_RCSOFTCHECK(rcl_publish(&s_odom_publisher, &s_odom_msg, nullptr));
}

void publish_imu(int64_t stamp_ms) {
    ImuMsg imu = {};
    if (system_globals().queues.imu_state == nullptr || xQueuePeek(system_globals().queues.imu_state, &imu, 0) != pdTRUE) {
        return;
    }

    set_stamp(&s_imu_msg.header, stamp_ms);
    s_imu_msg.orientation.w = imu.qw;
    s_imu_msg.orientation.x = imu.qx;
    s_imu_msg.orientation.y = imu.qy;
    s_imu_msg.orientation.z = imu.qz;
    s_imu_msg.angular_velocity.x = imu.gyro_x * ros_cfg::kDegToRad;
    s_imu_msg.angular_velocity.y = imu.gyro_y * ros_cfg::kDegToRad;
    s_imu_msg.angular_velocity.z = imu.gyro_z * ros_cfg::kDegToRad;
    s_imu_msg.linear_acceleration.x = imu.acc_x * ros_cfg::kGravity;
    s_imu_msg.linear_acceleration.y = imu.acc_y * ros_cfg::kGravity;
    s_imu_msg.linear_acceleration.z = imu.acc_z * ros_cfg::kGravity;
    MICROROS_RCSOFTCHECK(rcl_publish(&s_imu_publisher, &s_imu_msg, nullptr));
}

void publish_scan(int64_t stamp_ms) {
    LidarMsg lidar = {};
    if (system_globals().queues.lidar_state == nullptr || xQueuePeek(system_globals().queues.lidar_state, &lidar, 0) != pdTRUE) {
        return;
    }

    set_stamp(&s_scan_msg.header, stamp_ms);
    for (size_t i = 0; i < ros_cfg::kLaserScanPointCount; ++i) {
        s_scan_msg.ranges.data[i] = lidar.distances[i] > 0
            ? static_cast<float>(lidar.distances[i]) / 1000.0f
            : 0.0f;
    }
    MICROROS_RCSOFTCHECK(rcl_publish(&s_scan_publisher, &s_scan_msg, nullptr));
}

void publish_arm_pose() {
    ArmMsg arm = {};
    if (system_globals().queues.arm_state == nullptr || xQueuePeek(system_globals().queues.arm_state, &arm, 0) != pdTRUE || !arm.valid) {
        return;
    }

    const double pitch_rad = arm.pitch_deg * ros_cfg::kDegToRad;
    s_arm_pose_msg.position.x = arm.x;
    s_arm_pose_msg.position.y = arm.y;
    s_arm_pose_msg.position.z = arm.z;
    s_arm_pose_msg.orientation.x = 0.0;
    s_arm_pose_msg.orientation.y = sin(pitch_rad * 0.5);
    s_arm_pose_msg.orientation.z = 0.0;
    s_arm_pose_msg.orientation.w = cos(pitch_rad * 0.5);
    MICROROS_RCSOFTCHECK(rcl_publish(&s_arm_pose_publisher, &s_arm_pose_msg, nullptr));
}

void publish_wifi_config() {
    if (!s_wifi_config_publisher_initialized || s_wifi_config_msg.data.data == nullptr) {
        return;
    }

    char ssid[33] = {};
    if (!build_wifi_config_payload(
            s_wifi_config_data,
            sizeof(s_wifi_config_data),
            ssid,
            sizeof(ssid))) {
        return;
    }

    s_wifi_config_msg.data.size = strlen(s_wifi_config_data);
    s_wifi_config_msg.data.capacity = sizeof(s_wifi_config_data);
    MICROROS_RCSOFTCHECK(rcl_publish(&s_wifi_config_publisher, &s_wifi_config_msg, nullptr));

    if (strcmp(s_last_wifi_config_ssid, ssid) != 0) {
        snprintf(s_last_wifi_config_ssid, sizeof(s_last_wifi_config_ssid), "%s", ssid);
        ESP_LOGI(kTag, "Publishing wifi_config every 1s for SSID: %s", ssid);
    }
}

}  // namespace

void prepare_wifi_config_message_buffer() {
    memset(s_wifi_config_data, 0, sizeof(s_wifi_config_data));
    s_wifi_config_msg.data.data = s_wifi_config_data;
    s_wifi_config_msg.data.size = 0;
    s_wifi_config_msg.data.capacity = sizeof(s_wifi_config_data);
    s_last_wifi_config_ssid[0] = '\0';
}

void publish_state_timer(rcl_timer_t *timer, int64_t) {
    if (timer == nullptr ||
        (system_globals().runtime.wifi_comm_mode != WifiCommMode::kMicroRos &&
         system_globals().runtime.wifi_comm_mode != WifiCommMode::kMicroRosUart)) {
        return;
    }

    const int64_t stamp_ms = rmw_uros_epoch_millis();
    publish_odom(stamp_ms);
    publish_imu(stamp_ms);
    publish_arm_pose();

    if ((++s_publish_tick % ros_cfg::kScanPublishDivider) == 0) {
        publish_scan(stamp_ms);
    }
    if ((s_publish_tick % ros_cfg::kWifiConfigPublishTicks) == 0) {
        publish_wifi_config();
    }
}

}  // namespace microros
