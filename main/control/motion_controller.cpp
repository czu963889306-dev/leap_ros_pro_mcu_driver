// Copyright 2026. All rights reserved.

#include "motion_controller.h"
#include "motion_config.h"
#include <algorithm>
#include <cmath>

#define DEBUG_PID 0
#if DEBUG_PID
#include "esp_log.h"
static const char* TAG = "PID_CTRL";
#endif
namespace motion_cfg = leap_config::motion;

static bool IsNearZero(float value) {
  return std::abs(value) < 0.001f;
}

float MotionController::NormalizeAngle(float angle) {
  while (angle > M_PI) angle -= 2.0f * M_PI;
  while (angle < -M_PI) angle += 2.0f * M_PI;
  return angle;
}

MotionController::MotionController(
    At8236Motor& fl, At8236Motor& fr, At8236Motor& bl, At8236Motor& br,
    QuadratureEncoder& enc_fl, QuadratureEncoder& enc_fr,
    QuadratureEncoder& enc_bl, QuadratureEncoder& enc_br)
    : motors_{&fl, &fr, &bl, &br},
      encs_{&enc_fl, &enc_fr, &enc_bl, &enc_br},
      pid_vel_{
          PidController(PidMode::kIncremental, motion_cfg::kDefaultSpeedKp, motion_cfg::kDefaultSpeedKi, motion_cfg::kDefaultSpeedKd, -motion_cfg::kMaxPwm, motion_cfg::kMaxPwm),
          PidController(PidMode::kIncremental, motion_cfg::kDefaultSpeedKp, motion_cfg::kDefaultSpeedKi, motion_cfg::kDefaultSpeedKd, -motion_cfg::kMaxPwm, motion_cfg::kMaxPwm),
          PidController(PidMode::kIncremental, motion_cfg::kDefaultSpeedKp, motion_cfg::kDefaultSpeedKi, motion_cfg::kDefaultSpeedKd, -motion_cfg::kMaxPwm, motion_cfg::kMaxPwm),
          PidController(PidMode::kIncremental, motion_cfg::kDefaultSpeedKp, motion_cfg::kDefaultSpeedKi, motion_cfg::kDefaultSpeedKd, -motion_cfg::kMaxPwm, motion_cfg::kMaxPwm)} {}

void MotionController::SetChassisType(ChassisType type) {
  chassis_type_ = type;
  SelectKinematics();
  Stop();
}

void MotionController::SelectKinematics() {
  switch (chassis_type_) {
    case ChassisType::kMecanum:
      kinematics_ = &mecanum_kinematics_;
      break;
    case ChassisType::kDifferential:
    default:
      kinematics_ = &differential_kinematics_;
      break;
  }
}

float MotionController::WheelRpmToMmS(float wheel_rpm) const {
  const float rpm_to_mms =
      (chassis_type_ == ChassisType::kMecanum)
          ? motion_cfg::kMecanumRpmToMmS
          : motion_cfg::kDifferentialRpmToMmS;
  return wheel_rpm * rpm_to_mms;
}

float MotionController::MmSToWheelRpm(float speed_mms) const {
  const float rpm_to_mms =
      (chassis_type_ == ChassisType::kMecanum)
          ? motion_cfg::kMecanumRpmToMmS
          : motion_cfg::kDifferentialRpmToMmS;
  return speed_mms / rpm_to_mms;
}

void MotionController::SetMaxAcceleration(float accel_mms2) {
  max_accel_rpm_s_ = MmSToWheelRpm(accel_mms2);
}

void MotionController::GetOdometry(float* x, float* y, float* yaw) const {
  if (x) *x = pos_x_;
  if (y) *y = pos_y_;
  if (yaw) *yaw = odom_yaw_;
}

void MotionController::GetOdometryQuaternion(
    float* qw, float* qx, float* qy, float* qz) const {
  const float half_yaw = odom_yaw_ * 0.5f;

  if (qw) *qw = std::cos(half_yaw);
  if (qx) *qx = 0.0f;
  if (qy) *qy = 0.0f;
  if (qz) *qz = std::sin(half_yaw);
}

float MotionController::GetTotalDistance() const {
  return total_distance_;
}

void MotionController::ResetOdometry() {
  for (int i = 0; i < kNumWheels; ++i) {
    encs_[i]->ResetCount();
    last_count_[i] = 0;
    filtered_rpm_[i] = 0.0f;
    pid_vel_[i].Reset();
    final_target_vel_[i] = 0.0f;
    target_vel_[i] = 0.0f;
  }
  need_reset_yaw_offset_ = true;
  total_distance_ = 0.0f;
  pos_x_ = pos_y_ = odom_yaw_ = imu_yaw_rel_ = 0.0f;
}

void MotionController::CalculateKinematics(float linear_x, float linear_y, float angular_z) {
  float v_mms[4] = {0};
  kinematics_->Inverse(linear_x, linear_y, angular_z, v_mms);
  for (int i = 0; i < kNumWheels; ++i) {
    final_target_vel_[i] = MmSToWheelRpm(v_mms[i]);
  }
}

void MotionController::Drive(float linear_x, float linear_y, float angular_z) {
  if (IsNearZero(linear_x) && IsNearZero(linear_y) && IsNearZero(angular_z)) {
    Stop();
    return;
  }

  control_mode_ = ControlMode::kVelocity;
  CalculateKinematics(linear_x, linear_y, angular_z);
}

int16_t MotionController::ApplyDeadband(float target_vel, float pid_output) {
  if (std::abs(target_vel) > 0.1f) {
    float out = pid_output;
    if (std::abs(out) < motion_cfg::kMotorPwmDeadband) {
      out = (out >= 0.0f) ? motion_cfg::kMotorPwmDeadband : -motion_cfg::kMotorPwmDeadband;
    }
    return static_cast<int16_t>(std::clamp(out, -motion_cfg::kMaxPwm, motion_cfg::kMaxPwm));
  }
  return 0;
}

void MotionController::Update(float dt, float current_imu_yaw_rad) {
  if (dt <= 0.0f) return;

  for (int i = 0; i < kNumWheels; ++i) {
    int32_t current_count = encs_[i]->GetCount() * motion_cfg::kEncoderSigns[i];
    float pps = (current_count - last_count_[i]) / dt;
    last_count_[i] = current_count;
    float raw_rpm = (pps / motion_cfg::kPulsesPerWheelRev) * 60.0f;
    filtered_rpm_[i] = motion_cfg::kVelocityFilterAlpha * raw_rpm + (1.0f - motion_cfg::kVelocityFilterAlpha) * filtered_rpm_[i];
  }

  float local_vx, local_vy, local_vw;
  GetVelocity(&local_vx, &local_vy, &local_vw);

  float step_dist = std::sqrt(local_vx * local_vx + local_vy * local_vy) * dt;
  total_distance_ += step_dist;

  if (need_reset_yaw_offset_) {
    yaw_offset_ = current_imu_yaw_rad;
    need_reset_yaw_offset_ = false;
  }
  imu_yaw_rel_ = NormalizeAngle(current_imu_yaw_rad - yaw_offset_);
  odom_yaw_ += local_vw * dt;
  odom_yaw_ = NormalizeAngle(odom_yaw_);
  pos_x_ += (local_vx * std::cos(odom_yaw_) - local_vy * std::sin(odom_yaw_)) * dt;
  pos_y_ += (local_vx * std::sin(odom_yaw_) + local_vy * std::cos(odom_yaw_)) * dt;

  if (control_mode_ == ControlMode::kDirectPwm) {
    for (int i = 0; i < kNumWheels; ++i) {
      motors_[i]->SetSpeed(direct_pwm_[i]);
    }
    return;
  }

  float max_rpm_step = max_accel_rpm_s_ * dt;
  bool velocity_stop_command = (control_mode_ == ControlMode::kVelocity);
  for (int i = 0; i < kNumWheels && velocity_stop_command; ++i) {
    velocity_stop_command = IsNearZero(final_target_vel_[i]);
  }

  if (velocity_stop_command) {
    for (int i = 0; i < kNumWheels; ++i) {
      target_vel_[i] = 0.0f;
      final_target_vel_[i] = 0.0f;
      pid_vel_[i].Reset();
      motors_[i]->Brake();
    }
    return;
  }

  for (int i = 0; i < kNumWheels; ++i) {
    const float delta = final_target_vel_[i] - target_vel_[i];
    if (delta > max_rpm_step) {
      target_vel_[i] += max_rpm_step;
    } else if (delta < -max_rpm_step) {
      target_vel_[i] -= max_rpm_step;
    } else {
      target_vel_[i] = final_target_vel_[i];
    }

    if (std::abs(target_vel_[i]) < 0.1f) pid_vel_[i].Reset();

    float raw_pwm = pid_vel_[i].Calculate(target_vel_[i], filtered_rpm_[i], dt);
    int16_t final_pwm = ApplyDeadband(target_vel_[i], raw_pwm);

#if DEBUG_PID
    if (i == 2) {
      ESP_LOGI(TAG, "Mode:%d Pos:(%.1f, %.1f) Yaw:%.2f PWM_BL:%d",
               (int)control_mode_, pos_x_, pos_y_, odom_yaw_, final_pwm);
    }
#endif
    motors_[i]->SetSpeed(final_pwm);
  }
}

void MotionController::GetVelocity(float* linear_x, float* linear_y, float* angular_z) const {
  if (!linear_x || !linear_y || !angular_z) return;

  float v[4];
  for (int i = 0; i < kNumWheels; ++i) v[i] = WheelRpmToMmS(filtered_rpm_[i]);

  kinematics_->Forward(v, linear_x, linear_y, angular_z);
}

void MotionController::Stop() {
  control_mode_ = ControlMode::kVelocity;
  for (int i = 0; i < kNumWheels; ++i) {
    direct_pwm_[i] = 0;
    target_vel_[i] = 0.0f;
    final_target_vel_[i] = 0.0f;
    pid_vel_[i].Reset();
    motors_[i]->Brake();
  }
}

void MotionController::SetVelocityPidGains(float kp, float ki, float kd) {
  for (int i = 0; i < kNumWheels; ++i) pid_vel_[i].SetGains(kp, ki, kd);
}

void MotionController::SetMotorTargetVelocity(MotorID motor_id, float target_mms) {
  control_mode_ = ControlMode::kVelocity;
  const int motor_index = static_cast<int>(motor_id);
  if (motor_index < 0 || motor_index >= kNumWheels) {
    return;
  }
  direct_pwm_[motor_index] = 0;
  final_target_vel_[motor_index] = MmSToWheelRpm(target_mms);
  if (IsNearZero(target_mms)) {
    target_vel_[motor_index] = 0.0f;
    pid_vel_[motor_index].Reset();
    motors_[motor_index]->Brake();
  }
}

void MotionController::SetAllMotorTargetsVelocity(
    float fl_mms, float fr_mms, float bl_mms, float br_mms) {
  if (IsNearZero(fl_mms) && IsNearZero(fr_mms) &&
      IsNearZero(bl_mms) && IsNearZero(br_mms)) {
    Stop();
    return;
  }

  control_mode_ = ControlMode::kVelocity;
  for (int i = 0; i < kNumWheels; ++i) {
    direct_pwm_[i] = 0;
  }
  final_target_vel_[0] = MmSToWheelRpm(fl_mms);
  final_target_vel_[1] = MmSToWheelRpm(fr_mms);
  final_target_vel_[2] = MmSToWheelRpm(bl_mms);
  final_target_vel_[3] = MmSToWheelRpm(br_mms);
}

void MotionController::SetMotorTargetPwm(MotorID motor_id, int16_t pwm) {
  control_mode_ = ControlMode::kDirectPwm;
  const int motor_index = static_cast<int>(motor_id);
  if (motor_index < 0 || motor_index >= kNumWheels) {
    return;
  }
  direct_pwm_[motor_index] = std::clamp<int>(pwm, -static_cast<int>(motion_cfg::kMaxPwm), static_cast<int>(motion_cfg::kMaxPwm));
  target_vel_[motor_index] = 0.0f;
  final_target_vel_[motor_index] = 0.0f;
  pid_vel_[motor_index].Reset();
}

void MotionController::SetAllMotorTargetsPwm(
    int16_t fl_pwm, int16_t fr_pwm, int16_t bl_pwm, int16_t br_pwm) {
  control_mode_ = ControlMode::kDirectPwm;
  direct_pwm_[0] = std::clamp<int>(fl_pwm, -static_cast<int>(motion_cfg::kMaxPwm), static_cast<int>(motion_cfg::kMaxPwm));
  direct_pwm_[1] = std::clamp<int>(fr_pwm, -static_cast<int>(motion_cfg::kMaxPwm), static_cast<int>(motion_cfg::kMaxPwm));
  direct_pwm_[2] = std::clamp<int>(bl_pwm, -static_cast<int>(motion_cfg::kMaxPwm), static_cast<int>(motion_cfg::kMaxPwm));
  direct_pwm_[3] = std::clamp<int>(br_pwm, -static_cast<int>(motion_cfg::kMaxPwm), static_cast<int>(motion_cfg::kMaxPwm));
  for (int i = 0; i < kNumWheels; ++i) {
    target_vel_[i] = 0.0f;
    final_target_vel_[i] = 0.0f;
    pid_vel_[i].Reset();
  }
}

float MotionController::GetMotorVelocity(MotorID motor_id) const {
  const int motor_index = static_cast<int>(motor_id);
  if (motor_index < 0 || motor_index >= kNumWheels) {
    return 0.0f;
  }
  return WheelRpmToMmS(filtered_rpm_[motor_index]);
}

void MotionController::GetAllMotorVelocities(
    float* fl_mms, float* fr_mms, float* bl_mms, float* br_mms) const {
  if (fl_mms) *fl_mms = WheelRpmToMmS(filtered_rpm_[0]);
  if (fr_mms) *fr_mms = WheelRpmToMmS(filtered_rpm_[1]);
  if (bl_mms) *bl_mms = WheelRpmToMmS(filtered_rpm_[2]);
  if (br_mms) *br_mms = WheelRpmToMmS(filtered_rpm_[3]);
}
