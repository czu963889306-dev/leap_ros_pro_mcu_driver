#pragma once

#include <stddef.h>
#include <stdint.h>

namespace leap_config {
namespace ros {

static constexpr size_t kLaserScanPointCount = 360;
// odom, imu, scan, arm_pose, wifi_config, battery_state; rosout is disabled.
static constexpr size_t kPublisherCount = 6;
static constexpr double kDegToRad = 0.017453292519943295;
static constexpr double kGravity = 9.80665;

static constexpr size_t kHostInfoTextCapacity = 96;
static constexpr size_t kWifiConfigTextCapacity = 256;
static constexpr uint32_t kWifiConfigPublishTicks = 50;

static constexpr const char kNodeName[] = "leap_ros_pro";
static constexpr const char kOdomFrame[] = "odom";
static constexpr const char kBaseFrame[] = "base_link";
static constexpr const char kImuFrame[] = "imu_link";
static constexpr const char kLaserFrame[] = "laser_frame";
static constexpr const char kBatteryFrame[] = "battery";

static constexpr const char kOdomTopic[] = "odom";
static constexpr const char kImuTopic[] = "imu";
static constexpr const char kScanTopic[] = "scan";
static constexpr const char kBatteryTopic[] = "battery_state";
static constexpr const char kArmPoseTopic[] = "arm_pose";
static constexpr const char kWifiConfigTopic[] = "wifi_config";
static constexpr const char kCmdVelTopic[] = "cmd_vel";
static constexpr const char kServoCmdTopic[] = "servo_cmd";
static constexpr const char kHostInfoTopic[] = "host_info";
static constexpr const char kSetArmPoseService[] = "set_arm_pose";
static constexpr const char kSetSpeedPidService[] = "set_speed_pid";
static constexpr const char kGetSpeedPidService[] = "get_speed_pid";

static constexpr uint32_t kPublishPeriodMs = 20;
static constexpr uint32_t kBatteryPublishDivider = 25; // 2 Hz.
static constexpr uint32_t kScanPublishDivider = 5;
static constexpr float kScanTimeS = 0.1f;
static constexpr float kLaserRangeMinM = 0.02f;
static constexpr float kLaserRangeMaxM = 12.0f;
static constexpr int kInitialAgentPingTimeoutMs = 300;
static constexpr uint8_t kInitialAgentPingAttempts = 3;
static constexpr int kTimeSyncTimeoutMs = 1000;
// Runtime sync runs on the ROS task, so bound its wait separately from startup.
static constexpr int kTimeSyncRuntimeTimeoutMs = 50;
static constexpr uint32_t kTimeSyncIntervalMs = 5000;
static constexpr uint32_t kTimeSyncRetryIntervalMs = 1000;
static constexpr uint32_t kTimeSyncMaxAgeMs = 30000;

}  // namespace ros
}  // namespace leap_config
