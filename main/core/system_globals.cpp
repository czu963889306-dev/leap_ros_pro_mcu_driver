#include "system_globals.h"

#include <stdio.h>

#include "motion_config.h"
#include "motion_controller.h"
#include "network_config.h"

SystemGlobals::SystemGlobals() {
    runtime.chassis_type = ChassisType::kMecanum;
    runtime.microros_agent_port = leap_config::network::kDefaultMicroRosAgentPort;
    control.speed_pid.kp = leap_config::motion::kDefaultSpeedKp;
    control.speed_pid.ki = leap_config::motion::kDefaultSpeedKi;
    control.speed_pid.kd = leap_config::motion::kDefaultSpeedKd;

    snprintf(runtime.microros_agent_ip, sizeof(runtime.microros_agent_ip), "%s",
             LEAP_CONFIG_DEFAULT_MICROROS_AGENT_IP);
    snprintf(runtime.device_name, sizeof(runtime.device_name), "%s", "Maturo_UNKNOWN");
}

SystemGlobals& SystemGlobals::Instance() {
    static SystemGlobals instance;
    return instance;
}

SystemGlobals& system_globals() {
    return SystemGlobals::Instance();
}

bool SystemGlobals::IsWifiCommModeValid(WifiCommMode mode) {
    return mode == WifiCommMode::kMicroRos ||
           mode == WifiCommMode::kMicroRosUart;
}

WifiCommMode SystemGlobals::NextWifiCommMode(WifiCommMode mode) {
    switch (mode) {
    case WifiCommMode::kMicroRos:
        return WifiCommMode::kMicroRosUart;
    case WifiCommMode::kMicroRosUart:
        return WifiCommMode::kMicroRos;
    default:
        return WifiCommMode::kMicroRos;
    }
}

const char *SystemGlobals::WifiCommModeToRuntimeValue(WifiCommMode mode) {
    switch (mode) {
    case WifiCommMode::kMicroRos:
        return "micro_ros";
    case WifiCommMode::kMicroRosUart:
        return "microros_uart";
    default:
        return "unknown";
    }
}

const char *SystemGlobals::WifiCommModeToDisplayName(WifiCommMode mode) {
    switch (mode) {
    case WifiCommMode::kMicroRos:
        return "micro-ROS";
    case WifiCommMode::kMicroRosUart:
        return "micro-ROS UART";
    default:
        return "Unknown";
    }
}

bool SystemGlobals::IsChassisTypeValid(ChassisType type) {
    return type == ChassisType::kMecanum ||
           type == ChassisType::kDifferential;
}

const char *SystemGlobals::ChassisTypeToRuntimeValue(ChassisType type) {
    switch (type) {
    case ChassisType::kMecanum:
        return "mecanum";
    case ChassisType::kDifferential:
        return "differential";
    default:
        return "unknown";
    }
}

const char *SystemGlobals::ChassisTypeToDisplayName(ChassisType type) {
    switch (type) {
    case ChassisType::kMecanum:
        return "Mecanum";
    case ChassisType::kDifferential:
        return "4WD Differential";
    default:
        return "Unknown";
    }
}
