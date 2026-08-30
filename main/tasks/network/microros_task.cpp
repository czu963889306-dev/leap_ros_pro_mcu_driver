#include "system_globals.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "msg/imu_msg.h"
#include "msg/lidar_msg.h"
#include "msg/arm_msg.h"
#include "msg/motion_msg.h"
#include "msg/servo_msg.h"
#include "pid_config.h"

#include "geometry_msgs/msg/pose.h"
#include "geometry_msgs/msg/vector3.h"
#include "geometry_msgs/msg/twist.h"
#include "leap_interfaces/srv/get_speed_pid.h"
#include "leap_interfaces/srv/set_speed_pid.h"
#include "nav_msgs/msg/odometry.h"
#include "rcl/rcl.h"
#include "rcl/node_options.h"
#include "rcl/time.h"
#include "rcl/timer.h"
#include "rcl/wait.h"
#include "rclc/executor.h"
#include "rmw/qos_profiles.h"
#include "rmw_microros/custom_transport.h"
#include "rmw_microros/ping.h"
#include "rmw_microros/rmw_microros.h"
#include "rmw_microros/time_sync.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/laser_scan.h"
#include "uxr/client/profile/transport/custom/custom_transport.h"

static const char *TAG = "MICROROS";
static constexpr size_t kLaserScanPointCount = 360;
static constexpr double kDegToRad = 0.017453292519943295;
static constexpr double kGravity = 9.80665;

#ifndef CONFIG_MICRO_ROS_LOCAL_PORT
#define CONFIG_MICRO_ROS_LOCAL_PORT "8888"
#endif

struct UdpTransportContext {
    int fd;
    sockaddr_in remote;
    uint16_t local_port;
};

static UdpTransportContext s_udp_ctx = {};
static geometry_msgs__msg__Twist s_cmd_vel_msg = {};
static geometry_msgs__msg__Vector3 s_servo_cmd_msg = {};
static geometry_msgs__msg__Pose s_arm_pose_cmd_msg = {};
static geometry_msgs__msg__Pose s_arm_pose_msg = {};
static nav_msgs__msg__Odometry s_odom_msg = {};
static sensor_msgs__msg__Imu s_imu_msg = {};
static sensor_msgs__msg__LaserScan s_scan_msg = {};
static rcl_publisher_t s_odom_publisher = {};
static rcl_publisher_t s_imu_publisher = {};
static rcl_publisher_t s_scan_publisher = {};
static rcl_publisher_t s_arm_pose_publisher = {};
static rcl_subscription_t s_cmd_vel_subscriber = {};
static rcl_subscription_t s_servo_cmd_subscriber = {};
static rcl_subscription_t s_arm_pose_cmd_subscriber = {};
static rcl_service_t s_set_speed_pid_service = {};
static rcl_service_t s_get_speed_pid_service = {};
static rclc_executor_t s_service_executor = {};
static leap_interfaces__srv__SetSpeedPid_Request s_set_speed_pid_req = {};
static leap_interfaces__srv__SetSpeedPid_Response s_set_speed_pid_res = {};
static leap_interfaces__srv__GetSpeedPid_Request s_get_speed_pid_req = {};
static leap_interfaces__srv__GetSpeedPid_Response s_get_speed_pid_res = {};
static rcl_init_options_t s_init_options = {};
static rcl_context_t s_context = {};
static rcl_allocator_t s_allocator = {};
static rcl_node_t s_node = {};
static rcl_timer_t s_publish_timer = {};
static rcl_wait_set_t s_wait_set = {};
static rcl_clock_t s_clock = {};
static bool s_ros_created = false;
static bool s_strings_initialized = false;
static bool s_scan_ranges_initialized = false;
static bool s_init_options_initialized = false;
static bool s_context_initialized = false;
static bool s_node_initialized = false;
static bool s_odom_publisher_initialized = false;
static bool s_imu_publisher_initialized = false;
static bool s_scan_publisher_initialized = false;
static bool s_arm_pose_publisher_initialized = false;
static bool s_cmd_vel_subscriber_initialized = false;
static bool s_servo_cmd_subscriber_initialized = false;
static bool s_arm_pose_cmd_subscriber_initialized = false;
static bool s_set_speed_pid_service_initialized = false;
static bool s_get_speed_pid_service_initialized = false;
static bool s_service_executor_initialized = false;
static bool s_clock_initialized = false;
static bool s_timer_initialized = false;
static bool s_wait_set_initialized = false;
static uint32_t s_publish_tick = 0;

#define RCCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(TAG, "rcl status line %d: %d", __LINE__, static_cast<int>(rc)); \
        return false; \
    } \
} while (0)

#define RCSOFTCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(TAG, "rcl status line %d: %d", __LINE__, static_cast<int>(rc)); \
    } \
} while (0)

extern "C" bool transport_open_udp(uxrCustomTransport *transport) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    ctx->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (ctx->fd < 0) {
        return false;
    }

    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(ctx->local_port);
    bind(ctx->fd, reinterpret_cast<sockaddr *>(&local), sizeof(local));
    return true;
}

extern "C" bool transport_close_udp(uxrCustomTransport *transport) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    if (ctx->fd >= 0) {
        close(ctx->fd);
        ctx->fd = -1;
    }
    return true;
}

extern "C" size_t transport_write_udp(
    uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    const int sent = sendto(
        ctx->fd,
        buf,
        len,
        0,
        reinterpret_cast<sockaddr *>(&ctx->remote),
        sizeof(ctx->remote));
    return sent > 0 ? static_cast<size_t>(sent) : 0;
}

extern "C" size_t transport_read_udp(
    uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(ctx->fd, &readfds);

    timeval tv = {};
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;
    const int ret = select(ctx->fd + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
        return 0;
    }

    const int received = recv(ctx->fd, buf, len, 0);
    return received > 0 ? static_cast<size_t>(received) : 0;
}

static void cleanup_result(rcl_ret_t ret) {
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "cleanup status: %d", static_cast<int>(ret));
    }
}

static void reset_ros_handles() {
    s_odom_publisher = rcl_get_zero_initialized_publisher();
    s_imu_publisher = rcl_get_zero_initialized_publisher();
    s_scan_publisher = rcl_get_zero_initialized_publisher();
    s_arm_pose_publisher = rcl_get_zero_initialized_publisher();
    s_cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
    s_servo_cmd_subscriber = rcl_get_zero_initialized_subscription();
    s_arm_pose_cmd_subscriber = rcl_get_zero_initialized_subscription();
    s_set_speed_pid_service = rcl_get_zero_initialized_service();
    s_get_speed_pid_service = rcl_get_zero_initialized_service();
    s_service_executor = rclc_executor_get_zero_initialized_executor();
    s_init_options = rcl_get_zero_initialized_init_options();
    s_context = rcl_get_zero_initialized_context();
    s_node = rcl_get_zero_initialized_node();
    s_publish_timer = rcl_get_zero_initialized_timer();
    s_wait_set = rcl_get_zero_initialized_wait_set();
    s_clock = {};
}

static void handle_cmd_vel(const geometry_msgs__msg__Twist *msg) {
    if (msg == nullptr || q_motion_cmd == nullptr) {
        return;
    }

    MotionMsg cmd = {};
    cmd.source = MOTION_SRC_MICROROS;
    cmd.control_mode = 0;
    cmd.target_vx = static_cast<float>(msg->linear.x * 1000.0);
    cmd.target_vy = static_cast<float>(msg->linear.y * 1000.0);
    cmd.target_wz = static_cast<float>(msg->angular.z);
    xQueueOverwrite(q_motion_cmd, &cmd);
    g_emergency_stop = false;
}

static void handle_servo_cmd_vector(const geometry_msgs__msg__Vector3 *msg) {
    if (msg == nullptr || q_servo_cmd == nullptr) {
        return;
    }

    const int channel = static_cast<int>(msg->x);
    if (channel < 1 || channel > 5) {
        return;
    }

    ServoMsg servo_cmd = {};
    servo_cmd.channel = static_cast<uint8_t>(channel);
    servo_cmd.angle = static_cast<float>(msg->y);
    xQueueOverwrite(q_servo_cmd, &servo_cmd);
}

static double normalized_pitch_from_pose(const geometry_msgs__msg__Pose *msg) {
    const double x = msg->orientation.x;
    const double y = msg->orientation.y;
    const double z = msg->orientation.z;
    const double w = msg->orientation.w;
    if (x == 0.0 && y == 0.0 && z == 0.0 && w == 0.0) {
        return 0.0;
    }

    double sinp = 2.0 * (w * y - z * x);
    if (sinp > 1.0) {
        sinp = 1.0;
    } else if (sinp < -1.0) {
        sinp = -1.0;
    }
    return asin(sinp);
}

static void handle_arm_pose_cmd(const geometry_msgs__msg__Pose *msg) {
    if (msg == nullptr || q_arm_cmd == nullptr ||
        !isfinite(msg->position.x) ||
        !isfinite(msg->position.y) ||
        !isfinite(msg->position.z) ||
        !isfinite(msg->orientation.x) ||
        !isfinite(msg->orientation.y) ||
        !isfinite(msg->orientation.z) ||
        !isfinite(msg->orientation.w)) {
        return;
    }

    ArmMsg arm_cmd = {};
    arm_cmd.mode = ARM_CMD_CARTESIAN;
    arm_cmd.valid = true;
    arm_cmd.x = static_cast<float>(msg->position.x);
    arm_cmd.y = static_cast<float>(msg->position.y);
    arm_cmd.z = static_cast<float>(msg->position.z);
    arm_cmd.pitch_deg = static_cast<float>(normalized_pitch_from_pose(msg) / kDegToRad);
    ESP_LOGI(TAG, "arm_pose_cmd ros=(%.3f, %.3f, %.3f) ik=(%.3f, %.3f, %.3f) pitch=%.1f",
             msg->position.x,
             msg->position.y,
             msg->position.z,
             arm_cmd.x,
             arm_cmd.y,
             arm_cmd.z,
             arm_cmd.pitch_deg);
    xQueueOverwrite(q_arm_cmd, &arm_cmd);
}

static void set_speed_pid_response(
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

static void get_speed_pid_response(
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

static void handle_set_speed_pid_service(const void *req, void *res) {
    auto *request = static_cast<const leap_interfaces__srv__SetSpeedPid_Request *>(req);
    auto *response = static_cast<leap_interfaces__srv__SetSpeedPid_Response *>(res);
    if (request == nullptr || response == nullptr || q_speedpid_cmd == nullptr ||
        !isfinite(request->kp) || !isfinite(request->ki) || !isfinite(request->kd)) {
        set_speed_pid_response(response, false, g_speed_pid_state);
        return;
    }

    PidMsg pid_msg = {};
    pid_msg.kp = request->kp;
    pid_msg.ki = request->ki;
    pid_msg.kd = request->kd;
    esp_err_t save_err = pid_save_speed_config(&pid_msg);
    if (save_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save speed PID config: %s", esp_err_to_name(save_err));
        set_speed_pid_response(response, false, g_speed_pid_state);
        return;
    }

    g_speed_pid_state = pid_msg;
    xQueueOverwrite(q_speedpid_cmd, &pid_msg);
    set_speed_pid_response(response, true, pid_msg);
    ESP_LOGI(TAG, "Speed PID service set and saved: kp=%.3f, ki=%.3f, kd=%.3f",
             pid_msg.kp, pid_msg.ki, pid_msg.kd);
}

static void handle_get_speed_pid_service(const void *, void *res) {
    auto *response = static_cast<leap_interfaces__srv__GetSpeedPid_Response *>(res);
    get_speed_pid_response(response, true, g_speed_pid_state);
}

static void spin_pid_services() {
    if (s_service_executor_initialized) {
        RCSOFTCHECK(rclc_executor_spin_some(&s_service_executor, RCL_MS_TO_NS(0)));
    }
}

static void set_stamp(std_msgs__msg__Header *header, int64_t stamp_ms) {
    header->stamp.sec = static_cast<int32_t>(stamp_ms / 1000);
    header->stamp.nanosec = static_cast<uint32_t>((stamp_ms % 1000) * 1000000);
}

static void publish_odom(int64_t stamp_ms) {
    MotionMsg motion = {};
    if (q_motion_state == nullptr || xQueuePeek(q_motion_state, &motion, 0) != pdTRUE) {
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
    RCSOFTCHECK(rcl_publish(&s_odom_publisher, &s_odom_msg, nullptr));
}

static void publish_imu(int64_t stamp_ms) {
    ImuMsg imu = {};
    if (q_imu_state == nullptr || xQueuePeek(q_imu_state, &imu, 0) != pdTRUE) {
        return;
    }

    set_stamp(&s_imu_msg.header, stamp_ms);
    s_imu_msg.orientation.w = imu.qw;
    s_imu_msg.orientation.x = imu.qx;
    s_imu_msg.orientation.y = imu.qy;
    s_imu_msg.orientation.z = imu.qz;
    s_imu_msg.angular_velocity.x = imu.gyro_x * kDegToRad;
    s_imu_msg.angular_velocity.y = imu.gyro_y * kDegToRad;
    s_imu_msg.angular_velocity.z = imu.gyro_z * kDegToRad;
    s_imu_msg.linear_acceleration.x = imu.acc_x * kGravity;
    s_imu_msg.linear_acceleration.y = imu.acc_y * kGravity;
    s_imu_msg.linear_acceleration.z = imu.acc_z * kGravity;
    RCSOFTCHECK(rcl_publish(&s_imu_publisher, &s_imu_msg, nullptr));
}

static void publish_scan(int64_t stamp_ms) {
    LidarMsg lidar = {};
    if (q_lidar_state == nullptr || xQueuePeek(q_lidar_state, &lidar, 0) != pdTRUE) {
        return;
    }

    set_stamp(&s_scan_msg.header, stamp_ms);
    for (size_t i = 0; i < kLaserScanPointCount; ++i) {
        s_scan_msg.ranges.data[i] = lidar.distances[i] > 0
            ? static_cast<float>(lidar.distances[i]) / 1000.0f
            : 0.0f;
    }
    RCSOFTCHECK(rcl_publish(&s_scan_publisher, &s_scan_msg, nullptr));
}

static void publish_arm_pose() {
    ArmMsg arm = {};
    if (q_arm_state == nullptr || xQueuePeek(q_arm_state, &arm, 0) != pdTRUE || !arm.valid) {
        return;
    }

    const double pitch_rad = arm.pitch_deg * kDegToRad;
    s_arm_pose_msg.position.x = arm.x;
    s_arm_pose_msg.position.y = arm.y;
    s_arm_pose_msg.position.z = arm.z;
    s_arm_pose_msg.orientation.x = 0.0;
    s_arm_pose_msg.orientation.y = sin(pitch_rad * 0.5);
    s_arm_pose_msg.orientation.z = 0.0;
    s_arm_pose_msg.orientation.w = cos(pitch_rad * 0.5);
    RCSOFTCHECK(rcl_publish(&s_arm_pose_publisher, &s_arm_pose_msg, nullptr));
}

static void publish_state_timer(rcl_timer_t *timer, int64_t) {
    if (timer == nullptr || g_wifi_comm_mode != WifiCommMode::kMicroRos) {
        return;
    }

    const int64_t stamp_ms = rmw_uros_epoch_millis();
    publish_odom(stamp_ms);
    publish_imu(stamp_ms);
    publish_arm_pose();

    if ((++s_publish_tick % 5) == 0) {
        publish_scan(stamp_ms);
    }
}

static bool setup_udp_transport() {
    memset(&s_udp_ctx, 0, sizeof(s_udp_ctx));
    s_udp_ctx.fd = -1;
    s_udp_ctx.local_port = static_cast<uint16_t>(atoi(CONFIG_MICRO_ROS_LOCAL_PORT));
    s_udp_ctx.remote.sin_family = AF_INET;
    s_udp_ctx.remote.sin_port = htons(g_microros_agent_port);

    if (inet_pton(AF_INET, g_microros_agent_ip, &s_udp_ctx.remote.sin_addr.s_addr) != 1) {
        ESP_LOGE(TAG, "invalid micro-ROS agent IP: %s", g_microros_agent_ip);
        return false;
    }

    rmw_uros_set_custom_transport(
        false,
        &s_udp_ctx,
        transport_open_udp,
        transport_close_udp,
        transport_write_udp,
        transport_read_udp);
    return true;
}

static bool create_ros_entities() {
    s_publish_tick = 0;
    reset_ros_handles();

    rosidl_runtime_c__String__init(&s_odom_msg.header.frame_id);
    rosidl_runtime_c__String__init(&s_odom_msg.child_frame_id);
    rosidl_runtime_c__String__init(&s_imu_msg.header.frame_id);
    rosidl_runtime_c__String__init(&s_scan_msg.header.frame_id);
    s_strings_initialized = true;

    (void)rosidl_runtime_c__String__assign(&s_odom_msg.header.frame_id, "odom");
    (void)rosidl_runtime_c__String__assign(&s_odom_msg.child_frame_id, "base_link");
    (void)rosidl_runtime_c__String__assign(&s_imu_msg.header.frame_id, "imu_link");
    (void)rosidl_runtime_c__String__assign(&s_scan_msg.header.frame_id, "laser_frame");

    if (!rosidl_runtime_c__float32__Sequence__init(&s_scan_msg.ranges, kLaserScanPointCount)) {
        ESP_LOGE(TAG, "failed to allocate LaserScan ranges");
        return false;
    }
    s_scan_ranges_initialized = true;

    s_scan_msg.angle_min = 0.0f;
    s_scan_msg.angle_max = static_cast<float>((kLaserScanPointCount - 1) * kDegToRad);
    s_scan_msg.angle_increment = static_cast<float>(kDegToRad);
    s_scan_msg.time_increment = 0.0f;
    s_scan_msg.scan_time = 0.1f;
    s_scan_msg.range_min = 0.02f;
    s_scan_msg.range_max = 12.0f;

    s_imu_msg.orientation_covariance[0] = -1.0;
    s_imu_msg.angular_velocity_covariance[0] = -1.0;
    s_imu_msg.linear_acceleration_covariance[0] = -1.0;

    s_allocator = rcl_get_default_allocator();
    s_init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&s_init_options, s_allocator));
    s_init_options_initialized = true;

    const rmw_ret_t ping_ret = rmw_uros_ping_agent(300, 3);
    if (ping_ret != RMW_RET_OK) {
        // ESP_LOGW(TAG, "micro-ROS agent unavailable: %s:%u",
        //          g_microros_agent_ip, static_cast<unsigned>(g_microros_agent_port));
        return false;
    }

    s_context = rcl_get_zero_initialized_context();
    RCCHECK(rcl_init(0, nullptr, &s_init_options, &s_context));
    s_context_initialized = true;

    if (!rmw_uros_epoch_synchronized()) {
        (void)rmw_uros_sync_session(1000);
    }

    s_node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    node_options.enable_rosout = false;
    const rcl_ret_t node_ret = rcl_node_init(&s_node, "leap_low_driver", "", &s_context, &node_options);
    cleanup_result(rcl_node_options_fini(&node_options));
    if (node_ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl status line %d: %d", __LINE__, static_cast<int>(node_ret));
        return false;
    }
    s_node_initialized = true;

    rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
    rcl_publisher_options_t sensor_pub_options = rcl_publisher_get_default_options();
    sensor_pub_options.qos = rmw_qos_profile_sensor_data;
    s_odom_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_odom_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom",
        &pub_options));
    s_odom_publisher_initialized = true;

    s_imu_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_imu_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu",
        &sensor_pub_options));
    s_imu_publisher_initialized = true;

    s_scan_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_scan_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan",
        &pub_options));
    s_scan_publisher_initialized = true;

    s_arm_pose_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_arm_pose_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
        "arm_pose",
        &pub_options));
    s_arm_pose_publisher_initialized = true;

    rcl_subscription_options_t sub_options = rcl_subscription_get_default_options();
    sub_options.qos = rmw_qos_profile_sensor_data;
    s_cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
    RCCHECK(rcl_subscription_init(
        &s_cmd_vel_subscriber,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel",
        &sub_options));
    s_cmd_vel_subscriber_initialized = true;

    s_servo_cmd_subscriber = rcl_get_zero_initialized_subscription();
    RCCHECK(rcl_subscription_init(
        &s_servo_cmd_subscriber,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "servo_cmd",
        &sub_options));
    s_servo_cmd_subscriber_initialized = true;

    s_arm_pose_cmd_subscriber = rcl_get_zero_initialized_subscription();
    RCCHECK(rcl_subscription_init(
        &s_arm_pose_cmd_subscriber,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
        "arm_pose_cmd",
        &sub_options));
    s_arm_pose_cmd_subscriber_initialized = true;

    rcl_service_options_t service_options = rcl_service_get_default_options();
    service_options.qos = rmw_qos_profile_services_default;
    RCCHECK(rcl_service_init(
        &s_set_speed_pid_service,
        &s_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(leap_interfaces, srv, SetSpeedPid),
        "set_speed_pid",
        &service_options));
    s_set_speed_pid_service_initialized = true;

    RCCHECK(rcl_service_init(
        &s_get_speed_pid_service,
        &s_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(leap_interfaces, srv, GetSpeedPid),
        "get_speed_pid",
        &service_options));
    s_get_speed_pid_service_initialized = true;

    RCCHECK(rclc_executor_init(&s_service_executor, &s_context, 2, &s_allocator));
    s_service_executor_initialized = true;
    RCCHECK(rclc_executor_add_service(
        &s_service_executor,
        &s_set_speed_pid_service,
        &s_set_speed_pid_req,
        &s_set_speed_pid_res,
        handle_set_speed_pid_service));
    RCCHECK(rclc_executor_add_service(
        &s_service_executor,
        &s_get_speed_pid_service,
        &s_get_speed_pid_req,
        &s_get_speed_pid_res,
        handle_get_speed_pid_service));

    s_clock = {};
    s_publish_timer = rcl_get_zero_initialized_timer();
    RCCHECK(rcl_steady_clock_init(&s_clock, &s_allocator));
    s_clock_initialized = true;
    RCCHECK(rcl_timer_init(
        &s_publish_timer,
        &s_clock,
        &s_context,
        RCL_MS_TO_NS(20),
        publish_state_timer,
        s_allocator));
    s_timer_initialized = true;

    s_wait_set = rcl_get_zero_initialized_wait_set();
    RCCHECK(rcl_wait_set_init(&s_wait_set, 3, 0, 1, 0, 0, 0, &s_context, s_allocator));
    s_wait_set_initialized = true;
    s_ros_created = true;
    return true;
}

static void destroy_ros_entities() {
    if (!s_ros_created &&
        !s_strings_initialized &&
        !s_scan_ranges_initialized &&
        !s_init_options_initialized &&
        !s_context_initialized) {
        return;
    }

    if (s_context_initialized) {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&s_context);
        if (rmw_context) {
            (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        }
    }

    if (s_wait_set_initialized) {
        cleanup_result(rcl_wait_set_fini(&s_wait_set));
        s_wait_set_initialized = false;
    }
    if (s_timer_initialized) {
        cleanup_result(rcl_timer_fini(&s_publish_timer));
        s_timer_initialized = false;
    }
    if (s_clock_initialized) {
        cleanup_result(rcl_clock_fini(&s_clock));
        s_clock_initialized = false;
    }
    if (s_scan_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_scan_publisher, &s_node));
        s_scan_publisher_initialized = false;
    }
    if (s_arm_pose_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_arm_pose_publisher, &s_node));
        s_arm_pose_publisher_initialized = false;
    }
    if (s_imu_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_imu_publisher, &s_node));
        s_imu_publisher_initialized = false;
    }
    if (s_odom_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_odom_publisher, &s_node));
        s_odom_publisher_initialized = false;
    }
    if (s_cmd_vel_subscriber_initialized) {
        cleanup_result(rcl_subscription_fini(&s_cmd_vel_subscriber, &s_node));
        s_cmd_vel_subscriber_initialized = false;
    }
    if (s_servo_cmd_subscriber_initialized) {
        cleanup_result(rcl_subscription_fini(&s_servo_cmd_subscriber, &s_node));
        s_servo_cmd_subscriber_initialized = false;
    }
    if (s_arm_pose_cmd_subscriber_initialized) {
        cleanup_result(rcl_subscription_fini(&s_arm_pose_cmd_subscriber, &s_node));
        s_arm_pose_cmd_subscriber_initialized = false;
    }
    if (s_service_executor_initialized) {
        cleanup_result(rclc_executor_fini(&s_service_executor));
        s_service_executor_initialized = false;
    }
    if (s_set_speed_pid_service_initialized) {
        cleanup_result(rcl_service_fini(&s_set_speed_pid_service, &s_node));
        s_set_speed_pid_service_initialized = false;
    }
    if (s_get_speed_pid_service_initialized) {
        cleanup_result(rcl_service_fini(&s_get_speed_pid_service, &s_node));
        s_get_speed_pid_service_initialized = false;
    }
    if (s_node_initialized) {
        cleanup_result(rcl_node_fini(&s_node));
        s_node_initialized = false;
    }
    if (s_context_initialized) {
        cleanup_result(rcl_shutdown(&s_context));
        cleanup_result(rcl_context_fini(&s_context));
        s_context_initialized = false;
    }
    if (s_init_options_initialized) {
        cleanup_result(rcl_init_options_fini(&s_init_options));
        s_init_options_initialized = false;
    }
    if (s_strings_initialized) {
        rosidl_runtime_c__String__fini(&s_odom_msg.header.frame_id);
        rosidl_runtime_c__String__fini(&s_odom_msg.child_frame_id);
        rosidl_runtime_c__String__fini(&s_imu_msg.header.frame_id);
        rosidl_runtime_c__String__fini(&s_scan_msg.header.frame_id);
        s_strings_initialized = false;
    }
    if (s_scan_ranges_initialized) {
        rosidl_runtime_c__float32__Sequence__fini(&s_scan_msg.ranges);
        s_scan_ranges_initialized = false;
    }
    s_ros_created = false;
    reset_ros_handles();
}

static bool spin_once(int timeout_ms) {
    if (!s_wait_set_initialized) {
        return false;
    }

    rcl_ret_t ret = rcl_wait_set_clear(&s_wait_set);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_wait_set_clear failed: %d", static_cast<int>(ret));
        return false;
    }
    ret = rcl_wait_set_add_subscription(&s_wait_set, &s_cmd_vel_subscriber, nullptr);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_wait_set_add_subscription failed: %d", static_cast<int>(ret));
        return false;
    }
    ret = rcl_wait_set_add_subscription(&s_wait_set, &s_servo_cmd_subscriber, nullptr);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_wait_set_add_subscription failed: %d", static_cast<int>(ret));
        return false;
    }
    ret = rcl_wait_set_add_subscription(&s_wait_set, &s_arm_pose_cmd_subscriber, nullptr);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_wait_set_add_subscription failed: %d", static_cast<int>(ret));
        return false;
    }
    ret = rcl_wait_set_add_timer(&s_wait_set, &s_publish_timer, nullptr);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_wait_set_add_timer failed: %d", static_cast<int>(ret));
        return false;
    }

    ret = rcl_wait(&s_wait_set, RCL_MS_TO_NS(timeout_ms));
    if (ret == RCL_RET_TIMEOUT) {
        spin_pid_services();
        return true;
    }
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_wait failed: %d", static_cast<int>(ret));
        return false;
    }

    if (s_wait_set.subscriptions[0] != nullptr) {
        const rcl_ret_t take_ret = rcl_take(&s_cmd_vel_subscriber, &s_cmd_vel_msg, nullptr, nullptr);
        if (take_ret == RCL_RET_OK) {
            handle_cmd_vel(&s_cmd_vel_msg);
        } else if (take_ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
            ESP_LOGW(TAG, "rcl_take failed: %d", static_cast<int>(take_ret));
        }
    }

    if (s_wait_set.subscriptions[1] != nullptr) {
        const rcl_ret_t take_ret = rcl_take(&s_servo_cmd_subscriber, &s_servo_cmd_msg, nullptr, nullptr);
        if (take_ret == RCL_RET_OK) {
            handle_servo_cmd_vector(&s_servo_cmd_msg);
        } else if (take_ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
            ESP_LOGW(TAG, "rcl_take failed: %d", static_cast<int>(take_ret));
        }
    }

    if (s_wait_set.subscriptions[2] != nullptr) {
        const rcl_ret_t take_ret = rcl_take(&s_arm_pose_cmd_subscriber, &s_arm_pose_cmd_msg, nullptr, nullptr);
        if (take_ret == RCL_RET_OK) {
            handle_arm_pose_cmd(&s_arm_pose_cmd_msg);
        } else if (take_ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
            ESP_LOGW(TAG, "rcl_take failed: %d", static_cast<int>(take_ret));
        }
    }

    if (s_wait_set.timers[0] != nullptr) {
        RCSOFTCHECK(rcl_timer_call(&s_publish_timer));
    }
    spin_pid_services();
    return true;
}

void microros_task(void *p) {
    (void)p;

    while (1) {
        if (g_wifi_comm_mode == WifiCommMode::kMicroRos) {
            if (!s_ros_created) {
                if (setup_udp_transport() && create_ros_entities()) {
                    ESP_LOGI(TAG, "micro-ROS active: agent=%s:%u, pub=/odom,/imu,/scan,/arm_pose, sub=/cmd_vel,/servo_cmd,/arm_pose_cmd, srv=/set_speed_pid,/get_speed_pid",
                             g_microros_agent_ip,
                             static_cast<unsigned>(g_microros_agent_port));
                } else {
                    destroy_ros_entities();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
            }
            if (!spin_once(20)) {
                destroy_ros_entities();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } else {
            if (s_ros_created) {
                destroy_ros_entities();
                ESP_LOGI(TAG, "micro-ROS inactive");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
