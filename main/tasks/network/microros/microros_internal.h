#pragma once

#include <stdint.h>
#include <stddef.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "geometry_msgs/msg/pose.h"
#include "geometry_msgs/msg/vector3.h"
#include "geometry_msgs/msg/twist.h"
#include "leap_interfaces/srv/get_speed_pid.h"
#include "leap_interfaces/srv/set_arm_pose.h"
#include "leap_interfaces/srv/set_speed_pid.h"
#include "lwip/sockets.h"
#include "nav_msgs/msg/odometry.h"
#include "rcl/rcl.h"
#include "rcl/timer.h"
#include "rcl/wait.h"
#include "rclc/executor.h"
#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/laser_scan.h"
#include "std_msgs/msg/header.h"
#include "std_msgs/msg/string.h"

#include "network_config.h"
#include "ros_config.h"

namespace microros {

extern const char *const kTag;

namespace net_cfg = leap_config::network;
namespace ros_cfg = leap_config::ros;

struct UdpTransportContext {
    int fd;
    sockaddr_in remote;
    uint16_t local_port;
};

struct UartTransportContext {
    uart_port_t port;
};

extern UdpTransportContext s_udp_ctx;
extern UartTransportContext s_uart_ctx;
extern bool s_uart_transport_initialized;
extern bool s_uart_start_received;
extern uint8_t s_uart_start_filter_pending[sizeof(net_cfg::kMicroRosUartStartCommand) - 1];
extern size_t s_uart_start_filter_pending_len;
extern bool s_uart_start_filter_drop_delimiter;

extern geometry_msgs__msg__Twist s_cmd_vel_msg;
extern geometry_msgs__msg__Vector3 s_servo_cmd_msg;
extern std_msgs__msg__String s_host_info_msg;
extern std_msgs__msg__String s_wifi_config_msg;
extern geometry_msgs__msg__Pose s_arm_pose_msg;
extern nav_msgs__msg__Odometry s_odom_msg;
extern sensor_msgs__msg__Imu s_imu_msg;
extern sensor_msgs__msg__LaserScan s_scan_msg;

extern rcl_publisher_t s_odom_publisher;
extern rcl_publisher_t s_imu_publisher;
extern rcl_publisher_t s_scan_publisher;
extern rcl_publisher_t s_arm_pose_publisher;
extern rcl_publisher_t s_wifi_config_publisher;
extern rcl_subscription_t s_cmd_vel_subscriber;
extern rcl_subscription_t s_servo_cmd_subscriber;
extern rcl_subscription_t s_host_info_subscriber;
extern rcl_service_t s_set_arm_pose_service;
extern rcl_service_t s_set_speed_pid_service;
extern rcl_service_t s_get_speed_pid_service;
extern rclc_executor_t s_service_executor;

extern leap_interfaces__srv__SetArmPose_Request s_set_arm_pose_req;
extern leap_interfaces__srv__SetArmPose_Response s_set_arm_pose_res;
extern leap_interfaces__srv__SetSpeedPid_Request s_set_speed_pid_req;
extern leap_interfaces__srv__SetSpeedPid_Response s_set_speed_pid_res;
extern leap_interfaces__srv__GetSpeedPid_Request s_get_speed_pid_req;
extern leap_interfaces__srv__GetSpeedPid_Response s_get_speed_pid_res;

extern rcl_init_options_t s_init_options;
extern rcl_context_t s_context;
extern rcl_allocator_t s_allocator;
extern rcl_node_t s_node;
extern rcl_timer_t s_publish_timer;
extern rcl_wait_set_t s_wait_set;
extern rcl_clock_t s_clock;

extern bool s_ros_created;
extern bool s_strings_initialized;
extern bool s_scan_ranges_initialized;
extern bool s_init_options_initialized;
extern bool s_context_initialized;
extern bool s_node_initialized;
extern bool s_odom_publisher_initialized;
extern bool s_imu_publisher_initialized;
extern bool s_scan_publisher_initialized;
extern bool s_arm_pose_publisher_initialized;
extern bool s_wifi_config_publisher_initialized;
extern bool s_cmd_vel_subscriber_initialized;
extern bool s_servo_cmd_subscriber_initialized;
extern bool s_host_info_subscriber_initialized;
extern bool s_set_arm_pose_service_initialized;
extern bool s_set_speed_pid_service_initialized;
extern bool s_get_speed_pid_service_initialized;
extern bool s_service_executor_initialized;
extern bool s_clock_initialized;
extern bool s_timer_initialized;
extern bool s_wait_set_initialized;

extern uint32_t s_publish_tick;
extern uint32_t s_ros_session_start_ms;
extern uint32_t s_last_host_info_timeout_log_ms;
extern char s_host_info_data[ros_cfg::kHostInfoTextCapacity];
extern char s_wifi_config_data[ros_cfg::kWifiConfigTextCapacity];
extern char s_last_wifi_config_ssid[33];

void cleanup_result(rcl_ret_t ret);

bool setup_udp_transport();
bool setup_uart_transport();
bool wait_for_uart_start_command();

void prepare_host_info_message_buffer();
void prepare_wifi_config_message_buffer();
void handle_cmd_vel(const geometry_msgs__msg__Twist *msg);
void handle_servo_cmd_vector(const geometry_msgs__msg__Vector3 *msg);
void handle_host_info(const std_msgs__msg__String *msg);
bool host_info_timed_out();
void reset_host_info_state();

void handle_set_arm_pose_service(const void *req, void *res);
void handle_set_speed_pid_service(const void *req, void *res);
void handle_get_speed_pid_service(const void *req, void *res);
void spin_services();

void publish_state_timer(rcl_timer_t *timer, int64_t last_call_time);

}  // namespace microros

#define MICROROS_RCCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(microros::kTag, "rcl status line %d: %d", __LINE__, static_cast<int>(rc)); \
        return false; \
    } \
} while (0)

#define MICROROS_RCSOFTCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(microros::kTag, "rcl status line %d: %d", __LINE__, static_cast<int>(rc)); \
    } \
} while (0)
