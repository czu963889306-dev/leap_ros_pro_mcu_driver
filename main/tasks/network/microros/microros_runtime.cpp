#include "microros_internal.h"
#include "microros_runtime.h"
#include "microros_time_sync.h"

#include "esp_log.h"
#include "geometry_msgs/msg/pose.h"
#include "geometry_msgs/msg/vector3.h"
#include "geometry_msgs/msg/twist.h"
#include "leap_interfaces/srv/get_speed_pid.h"
#include "leap_interfaces/srv/set_arm_pose.h"
#include "leap_interfaces/srv/set_speed_pid.h"
#include "nav_msgs/msg/odometry.h"
#include "rcl/node_options.h"
#include "rmw/qos_profiles.h"
#include "rmw_microxrcedds_c/config.h"
#include "rmw_microros/ping.h"
#include "rmw_microros/rmw_microros.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/laser_scan.h"
#include "std_msgs/msg/string.h"
#include "system_globals.h"

namespace microros {
namespace {

enum WaitSetSubscriptionIndex : size_t {
    kCmdVelSubscriptionIndex = 0,
    kServoCmdSubscriptionIndex = 1,
    kHostInfoSubscriptionIndex = 2,
    kSubscriptionCount = 3,
};

constexpr size_t kPublishTimerCount = 1;
static_assert(RMW_UXRCE_MAX_PUBLISHERS == ros_cfg::kPublisherCount,
              "Publisher limit mismatch: rebuild libmicroros with app-colcon.meta");

void reset_ros_handles() {
    s_odom_publisher = rcl_get_zero_initialized_publisher();
    s_imu_publisher = rcl_get_zero_initialized_publisher();
    s_battery_publisher = rcl_get_zero_initialized_publisher();
    s_scan_publisher = rcl_get_zero_initialized_publisher();
    s_arm_pose_publisher = rcl_get_zero_initialized_publisher();
    s_wifi_config_publisher = rcl_get_zero_initialized_publisher();
    s_cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
    s_servo_cmd_subscriber = rcl_get_zero_initialized_subscription();
    s_host_info_subscriber = rcl_get_zero_initialized_subscription();
    s_set_arm_pose_service = rcl_get_zero_initialized_service();
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

bool init_message_storage() {
    rosidl_runtime_c__String__init(&s_odom_msg.header.frame_id);
    rosidl_runtime_c__String__init(&s_odom_msg.child_frame_id);
    rosidl_runtime_c__String__init(&s_imu_msg.header.frame_id);
    rosidl_runtime_c__String__init(&s_scan_msg.header.frame_id);
    s_strings_initialized = true;

    if (!sensor_msgs__msg__BatteryState__init(&s_battery_msg)) {
        ESP_LOGE(kTag, "failed to initialize BatteryState");
        return false;
    }
    s_battery_msg_initialized = true;
    if (!rosidl_runtime_c__String__assign(&s_battery_msg.header.frame_id, ros_cfg::kBatteryFrame) ||
        !rosidl_runtime_c__String__assign(&s_battery_msg.location, "main")) {
        ESP_LOGE(kTag, "failed to allocate BatteryState strings");
        return false;
    }

    (void)rosidl_runtime_c__String__assign(&s_odom_msg.header.frame_id, ros_cfg::kOdomFrame);
    (void)rosidl_runtime_c__String__assign(&s_odom_msg.child_frame_id, ros_cfg::kBaseFrame);
    (void)rosidl_runtime_c__String__assign(&s_imu_msg.header.frame_id, ros_cfg::kImuFrame);
    (void)rosidl_runtime_c__String__assign(&s_scan_msg.header.frame_id, ros_cfg::kLaserFrame);

    if (!rosidl_runtime_c__float32__Sequence__init(&s_scan_msg.ranges, ros_cfg::kLaserScanPointCount)) {
        ESP_LOGE(kTag, "failed to allocate LaserScan ranges");
        return false;
    }
    s_scan_ranges_initialized = true;

    s_scan_msg.angle_min = 0.0f;
    s_scan_msg.angle_max = static_cast<float>((ros_cfg::kLaserScanPointCount - 1) * ros_cfg::kDegToRad);
    s_scan_msg.angle_increment = static_cast<float>(ros_cfg::kDegToRad);
    s_scan_msg.time_increment = 0.0f;
    s_scan_msg.scan_time = ros_cfg::kScanTimeS;
    s_scan_msg.range_min = ros_cfg::kLaserRangeMinM;
    s_scan_msg.range_max = ros_cfg::kLaserRangeMaxM;

    s_imu_msg.orientation_covariance[0] = -1.0;
    s_imu_msg.angular_velocity_covariance[0] = -1.0;
    s_imu_msg.linear_acceleration_covariance[0] = -1.0;
    prepare_wifi_config_message_buffer();
    prepare_host_info_message_buffer();
    return true;
}

bool init_context() {
    s_allocator = rcl_get_default_allocator();
    s_init_options = rcl_get_zero_initialized_init_options();
    MICROROS_RCCHECK(rcl_init_options_init(&s_init_options, s_allocator));
    s_init_options_initialized = true;

    // Keep the agent availability check used by the reference UART transport;
    // this is independent of the removed UART start-command handshake.
    const rmw_ret_t ping_ret = rmw_uros_ping_agent(
        ros_cfg::kInitialAgentPingTimeoutMs,
        ros_cfg::kInitialAgentPingAttempts);
    if (ping_ret != RMW_RET_OK) {
        return false;
    }

    s_context = rcl_get_zero_initialized_context();
    MICROROS_RCCHECK(rcl_init(0, nullptr, &s_init_options, &s_context));
    s_context_initialized = true;

    update_time_sync(ros_cfg::kTimeSyncTimeoutMs);
    return true;
}

bool init_node() {
    s_node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    node_options.enable_rosout = false;
    const rcl_ret_t node_ret =
        rcl_node_init(&s_node, ros_cfg::kNodeName, "", &s_context, &node_options);
    cleanup_result(rcl_node_options_fini(&node_options));
    if (node_ret != RCL_RET_OK) {
        ESP_LOGW(kTag, "rcl status line %d: %d", __LINE__, static_cast<int>(node_ret));
        return false;
    }
    s_node_initialized = true;
    return true;
}

bool init_publishers() {
    rcl_publisher_options_t state_pub_options = rcl_publisher_get_default_options();
    state_pub_options.qos = rmw_qos_profile_sensor_data;

    s_odom_publisher = rcl_get_zero_initialized_publisher();
    MICROROS_RCCHECK(rcl_publisher_init(
        &s_odom_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        ros_cfg::kOdomTopic,
        &state_pub_options));
    s_odom_publisher_initialized = true;

    s_imu_publisher = rcl_get_zero_initialized_publisher();
    MICROROS_RCCHECK(rcl_publisher_init(
        &s_imu_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        ros_cfg::kImuTopic,
        &state_pub_options));
    s_imu_publisher_initialized = true;

    s_scan_publisher = rcl_get_zero_initialized_publisher();
    MICROROS_RCCHECK(rcl_publisher_init(
        &s_scan_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        ros_cfg::kScanTopic,
        &state_pub_options));
    s_scan_publisher_initialized = true;

    s_battery_publisher = rcl_get_zero_initialized_publisher();
    MICROROS_RCCHECK(rcl_publisher_init(
        &s_battery_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        ros_cfg::kBatteryTopic,
        &state_pub_options));
    s_battery_publisher_initialized = true;

    s_arm_pose_publisher = rcl_get_zero_initialized_publisher();
    MICROROS_RCCHECK(rcl_publisher_init(
        &s_arm_pose_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
        ros_cfg::kArmPoseTopic,
        &state_pub_options));
    s_arm_pose_publisher_initialized = true;

    rcl_publisher_options_t wifi_config_pub_options = rcl_publisher_get_default_options();
    wifi_config_pub_options.qos = rmw_qos_profile_sensor_data;
    s_wifi_config_publisher = rcl_get_zero_initialized_publisher();
    MICROROS_RCCHECK(rcl_publisher_init(
        &s_wifi_config_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        ros_cfg::kWifiConfigTopic,
        &wifi_config_pub_options));
    s_wifi_config_publisher_initialized = true;
    return true;
}

bool init_subscribers() {
    rcl_subscription_options_t sub_options = rcl_subscription_get_default_options();
    sub_options.qos = rmw_qos_profile_sensor_data;

    s_cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
    MICROROS_RCCHECK(rcl_subscription_init(
        &s_cmd_vel_subscriber,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        ros_cfg::kCmdVelTopic,
        &sub_options));
    s_cmd_vel_subscriber_initialized = true;

    s_servo_cmd_subscriber = rcl_get_zero_initialized_subscription();
    MICROROS_RCCHECK(rcl_subscription_init(
        &s_servo_cmd_subscriber,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        ros_cfg::kServoCmdTopic,
        &sub_options));
    s_servo_cmd_subscriber_initialized = true;

    rcl_subscription_options_t host_info_sub_options = rcl_subscription_get_default_options();
    MICROROS_RCCHECK(rcl_subscription_init(
        &s_host_info_subscriber,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        ros_cfg::kHostInfoTopic,
        &host_info_sub_options));
    s_host_info_subscriber_initialized = true;
    return true;
}

bool init_services() {
    rcl_service_options_t service_options = rcl_service_get_default_options();
    service_options.qos = rmw_qos_profile_services_default;

    MICROROS_RCCHECK(rcl_service_init(
        &s_set_arm_pose_service,
        &s_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(leap_interfaces, srv, SetArmPose),
        ros_cfg::kSetArmPoseService,
        &service_options));
    s_set_arm_pose_service_initialized = true;

    MICROROS_RCCHECK(rcl_service_init(
        &s_set_speed_pid_service,
        &s_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(leap_interfaces, srv, SetSpeedPid),
        ros_cfg::kSetSpeedPidService,
        &service_options));
    s_set_speed_pid_service_initialized = true;

    MICROROS_RCCHECK(rcl_service_init(
        &s_get_speed_pid_service,
        &s_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(leap_interfaces, srv, GetSpeedPid),
        ros_cfg::kGetSpeedPidService,
        &service_options));
    s_get_speed_pid_service_initialized = true;

    MICROROS_RCCHECK(rclc_executor_init(&s_service_executor, &s_context, 3, &s_allocator));
    s_service_executor_initialized = true;
    MICROROS_RCCHECK(rclc_executor_add_service(
        &s_service_executor,
        &s_set_arm_pose_service,
        &s_set_arm_pose_req,
        &s_set_arm_pose_res,
        handle_set_arm_pose_service));
    MICROROS_RCCHECK(rclc_executor_add_service(
        &s_service_executor,
        &s_set_speed_pid_service,
        &s_set_speed_pid_req,
        &s_set_speed_pid_res,
        handle_set_speed_pid_service));
    MICROROS_RCCHECK(rclc_executor_add_service(
        &s_service_executor,
        &s_get_speed_pid_service,
        &s_get_speed_pid_req,
        &s_get_speed_pid_res,
        handle_get_speed_pid_service));
    return true;
}

bool init_timer_and_wait_set() {
    s_clock = {};
    s_publish_timer = rcl_get_zero_initialized_timer();
    MICROROS_RCCHECK(rcl_steady_clock_init(&s_clock, &s_allocator));
    s_clock_initialized = true;
    MICROROS_RCCHECK(rcl_timer_init(
        &s_publish_timer,
        &s_clock,
        &s_context,
        RCL_MS_TO_NS(ros_cfg::kPublishPeriodMs),
        publish_state_timer,
        s_allocator));
    s_timer_initialized = true;

    s_wait_set = rcl_get_zero_initialized_wait_set();
    MICROROS_RCCHECK(rcl_wait_set_init(
        &s_wait_set,
        kSubscriptionCount,
        0,
        kPublishTimerCount,
        0,
        0,
        0,
        &s_context,
        s_allocator));
    s_wait_set_initialized = true;
    return true;
}

bool create_ros_entities() {
    s_publish_tick = 0;
    reset_time_sync();
    reset_ros_handles();
    reset_host_info_state();

    if (!init_message_storage() ||
        !init_context() ||
        !init_node() ||
        !init_publishers() ||
        !init_subscribers() ||
        !init_services() ||
        !init_timer_and_wait_set()) {
        return false;
    }

    s_ros_created = true;
    return true;
}

void destroy_ros_entities() {
    reset_time_sync();
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
    if (s_battery_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_battery_publisher, &s_node));
        s_battery_publisher_initialized = false;
    }
    if (s_arm_pose_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_arm_pose_publisher, &s_node));
        s_arm_pose_publisher_initialized = false;
    }
    if (s_wifi_config_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_wifi_config_publisher, &s_node));
        s_wifi_config_publisher_initialized = false;
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
    if (s_host_info_subscriber_initialized) {
        cleanup_result(rcl_subscription_fini(&s_host_info_subscriber, &s_node));
        s_host_info_subscriber_initialized = false;
    }
    if (s_service_executor_initialized) {
        cleanup_result(rclc_executor_fini(&s_service_executor));
        s_service_executor_initialized = false;
    }
    if (s_set_arm_pose_service_initialized) {
        cleanup_result(rcl_service_fini(&s_set_arm_pose_service, &s_node));
        s_set_arm_pose_service_initialized = false;
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
    if (s_battery_msg_initialized) {
        sensor_msgs__msg__BatteryState__fini(&s_battery_msg);
        s_battery_msg_initialized = false;
    }
    s_ros_created = false;
    reset_ros_handles();
}

bool add_wait_set_subscription(rcl_subscription_t *subscriber) {
    const rcl_ret_t ret = rcl_wait_set_add_subscription(&s_wait_set, subscriber, nullptr);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(kTag, "rcl_wait_set_add_subscription failed: %d", static_cast<int>(ret));
        return false;
    }
    return true;
}

void take_cmd_vel_if_ready() {
    if (s_wait_set.subscriptions[kCmdVelSubscriptionIndex] == nullptr) {
        return;
    }

    const rcl_ret_t take_ret =
        rcl_take(&s_cmd_vel_subscriber, &s_cmd_vel_msg, nullptr, nullptr);
    if (take_ret == RCL_RET_OK) {
        handle_cmd_vel(&s_cmd_vel_msg);
    } else if (take_ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
        ESP_LOGW(kTag, "rcl_take failed: %d", static_cast<int>(take_ret));
    }
}

void take_servo_cmd_if_ready() {
    if (s_wait_set.subscriptions[kServoCmdSubscriptionIndex] == nullptr) {
        return;
    }

    const rcl_ret_t take_ret =
        rcl_take(&s_servo_cmd_subscriber, &s_servo_cmd_msg, nullptr, nullptr);
    if (take_ret == RCL_RET_OK) {
        handle_servo_cmd_vector(&s_servo_cmd_msg);
    } else if (take_ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
        ESP_LOGW(kTag, "rcl_take failed: %d", static_cast<int>(take_ret));
    }
}

void take_host_info_if_ready() {
    if (s_wait_set.subscriptions[kHostInfoSubscriptionIndex] == nullptr) {
        return;
    }

    const rcl_ret_t take_ret =
        rcl_take(&s_host_info_subscriber, &s_host_info_msg, nullptr, nullptr);
    if (take_ret == RCL_RET_OK) {
        handle_host_info(&s_host_info_msg);
    } else if (take_ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
        ESP_LOGW(kTag, "rcl_take failed: %d", static_cast<int>(take_ret));
    }
}

bool spin_once(int timeout_ms) {
    if (!s_wait_set_initialized) {
        return false;
    }

    rcl_ret_t ret = rcl_wait_set_clear(&s_wait_set);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(kTag, "rcl_wait_set_clear failed: %d", static_cast<int>(ret));
        return false;
    }
    if (!add_wait_set_subscription(&s_cmd_vel_subscriber) ||
        !add_wait_set_subscription(&s_servo_cmd_subscriber) ||
        !add_wait_set_subscription(&s_host_info_subscriber)) {
        return false;
    }
    ret = rcl_wait_set_add_timer(&s_wait_set, &s_publish_timer, nullptr);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(kTag, "rcl_wait_set_add_timer failed: %d", static_cast<int>(ret));
        return false;
    }

    ret = rcl_wait(&s_wait_set, RCL_MS_TO_NS(timeout_ms));
    if (ret == RCL_RET_TIMEOUT) {
        spin_services();
        return true;
    }
    if (ret != RCL_RET_OK) {
        ESP_LOGW(kTag, "rcl_wait failed: %d", static_cast<int>(ret));
        return false;
    }

    take_cmd_vel_if_ready();
    take_servo_cmd_if_ready();
    take_host_info_if_ready();

    if (s_wait_set.timers[0] != nullptr) {
        MICROROS_RCSOFTCHECK(rcl_timer_call(&s_publish_timer));
    }
    spin_services();
    return true;
}

void log_active_transport(bool use_uart) {
    if (use_uart) {
        ESP_LOGI(kTag, "micro-ROS UART active: UART%d TX=%d RX=%d baud=%d, pub=/odom,/imu,/scan,/arm_pose,/wifi_config,/battery_state, sub=/cmd_vel,/servo_cmd,/host_info, srv=/set_arm_pose,/set_speed_pid,/get_speed_pid",
                 static_cast<int>(net_cfg::kMicroRosUartPort),
                 static_cast<int>(net_cfg::kMicroRosUartTxPin),
                 static_cast<int>(net_cfg::kMicroRosUartRxPin),
                 net_cfg::kMicroRosUartBaudRate);
        return;
    }

    ESP_LOGI(kTag, "micro-ROS UDP active: agent=%s:%u, pub=/odom,/imu,/scan,/arm_pose,/wifi_config,/battery_state, sub=/cmd_vel,/servo_cmd,/host_info, srv=/set_arm_pose,/set_speed_pid,/get_speed_pid",
             system_globals().runtime.microros_agent_ip,
             static_cast<unsigned>(system_globals().runtime.microros_agent_port));
}

}  // namespace
}  // namespace microros

void microros_runtime_loop(void) {
    using namespace microros;

    // UART0 is also the boot console. Allow startup Wi-Fi/IP logs to be
    // emitted before switching to the micro-ROS transport at 921600 baud.
    if (system_globals().runtime.wifi_comm_mode == WifiCommMode::kMicroRosUart) {
        ESP_LOGI(kTag, "Delaying micro-ROS UART startup for %u ms",
                 static_cast<unsigned>(net_cfg::kMicroRosUartStartupDelayMs));
        vTaskDelay(pdMS_TO_TICKS(net_cfg::kMicroRosUartStartupDelayMs));
    }

    while (1) {
        const bool use_udp = system_globals().runtime.wifi_comm_mode == WifiCommMode::kMicroRos;
        const bool use_uart = system_globals().runtime.wifi_comm_mode == WifiCommMode::kMicroRosUart;
        if (use_udp || use_uart) {
            if (!s_ros_created) {
                const bool transport_ready = use_uart ? setup_uart_transport() : setup_udp_transport();
                if (transport_ready && create_ros_entities()) {
                    log_active_transport(use_uart);
                } else {
                    destroy_ros_entities();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
            }
            if (!spin_once(20)) {
                ESP_LOGW(kTag, "micro-ROS transport stopped; recreating session");
                destroy_ros_entities();
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            // Serialize time exchange with publishing/subscriptions on this task.
            update_time_sync(ros_cfg::kTimeSyncRuntimeTimeoutMs);
        } else {
            if (s_ros_created) {
                destroy_ros_entities();
                ESP_LOGI(kTag, "micro-ROS inactive");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
