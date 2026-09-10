#include "system_globals.h"
#include "motion_controller.h"

volatile bool g_emergency_stop = false;
volatile bool g_motion_busy = false;
volatile uint32_t g_lidar_scan_sequence = 0;
volatile WifiCommMode g_wifi_comm_mode = WifiCommMode::kMicroRos;
ChassisType g_chassis_type = ChassisType::kMecanum;
char g_microros_agent_ip[16] = "192.168.31.214";
uint16_t g_microros_agent_port = 8888;
char g_device_name[32] = "Maturo_UNKNOWN";
TargetHostInfo g_target_host_info = {
    .valid = false,
    .last_update_ms = 0,
    .hostname = {0},
    .hostip = {0},
    .text = {0},
};

bool wifi_comm_mode_is_valid(WifiCommMode mode) {
    return mode == WifiCommMode::kMicroRos ||
           mode == WifiCommMode::kMicroRosUart;
}

WifiCommMode wifi_comm_mode_next(WifiCommMode mode) {
    switch (mode) {
    case WifiCommMode::kMicroRos:
        return WifiCommMode::kMicroRosUart;
    case WifiCommMode::kMicroRosUart:
        return WifiCommMode::kMicroRos;
    default:
        return WifiCommMode::kMicroRos;
    }
}

const char *wifi_comm_mode_to_runtime_value(WifiCommMode mode) {
    switch (mode) {
    case WifiCommMode::kMicroRos:
        return "micro_ros";
    case WifiCommMode::kMicroRosUart:
        return "microros_uart";
    default:
        return "unknown";
    }
}

const char *wifi_comm_mode_to_display_name(WifiCommMode mode) {
    switch (mode) {
    case WifiCommMode::kMicroRos:
        return "micro-ROS";
    case WifiCommMode::kMicroRosUart:
        return "micro-ROS UART";
    default:
        return "Unknown";
    }
}

bool chassis_type_is_valid(ChassisType type) {
    return type == ChassisType::kMecanum ||
           type == ChassisType::kDifferential;
}

const char *chassis_type_to_runtime_value(ChassisType type) {
    switch (type) {
    case ChassisType::kMecanum:
        return "mecanum";
    case ChassisType::kDifferential:
        return "differential";
    default:
        return "unknown";
    }
}

const char *chassis_type_to_display_name(ChassisType type) {
    switch (type) {
    case ChassisType::kMecanum:
        return "Mecanum";
    case ChassisType::kDifferential:
        return "4WD Differential";
    default:
        return "Unknown";
    }
}

QueueHandle_t q_imu_state = nullptr;
QueueHandle_t q_motion_state = nullptr;
QueueHandle_t q_ultrasonic_state = nullptr;
QueueHandle_t q_lidar_state = nullptr;   
QueueHandle_t q_gamepad_state = nullptr;
QueueHandle_t q_temperature_state = nullptr;


QueueHandle_t q_motion_cmd = nullptr;
QueueHandle_t q_rgb_cmd = nullptr;
QueueHandle_t q_servo_cmd = nullptr;
QueueHandle_t q_arm_cmd = nullptr;
QueueHandle_t q_arm_state = nullptr;
QueueHandle_t q_speedpid_cmd = nullptr;

PidMsg g_speed_pid_state = {
    .kp = 1.0f,
    .ki = 6.0f,
    .kd = 0.0f,
};
