// Copyright 2026. All rights reserved.

#include "motion_controller.h"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

#define DEBUG_PID 0
static const char* TAG = "PID_CTRL";

static const float kWheelDiameter = 65.0f;
static const float kWheelbase = 175.0f;
static const float kTrackWidth = 170.0f;
static const float kDifferentialEffectiveTrackWidth = 340.0f;
static const float kLx = kWheelbase / 2.0f;
static const float kLy = kTrackWidth / 2.0f;
static const float kDifferentialHalfTrack = kDifferentialEffectiveTrackWidth / 2.0f;
static const float kRpmToMms = (M_PI * kWheelDiameter) / 60.0f;
static const float kMmsToRpm = 1.0f / kRpmToMms;
static const float kGearReduction = 46.8f;
static const float kEncoderCountsPerMotorRev = 44.0f;
static const float kPulsesPerWheelRev = kEncoderCountsPerMotorRev * kGearReduction;

static const float kKp = 3.0f, kKi = 5.0f, kKd = 0.0f, kMaxPwm = 255.0f;

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
          PidController(PidMode::kIncremental, kKp, kKi, kKd, -kMaxPwm, kMaxPwm),
          PidController(PidMode::kIncremental, kKp, kKi, kKd, -kMaxPwm, kMaxPwm),
          PidController(PidMode::kIncremental, kKp, kKi, kKd, -kMaxPwm, kMaxPwm),
          PidController(PidMode::kIncremental, kKp, kKi, kKd, -kMaxPwm, kMaxPwm)} {}

void MotionController::SetChassisType(ChassisType type) {
  chassis_type_ = type;
  Stop();
}

void MotionController::SetMaxAcceleration(float accel_mms2) {
  max_accel_rpm_s_ = accel_mms2 * kMmsToRpm;
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
  if (chassis_type_ == ChassisType::kMecanum) {
    // X-pattern mecanum inverse kinematics. Wheel order: FL, FR, BL, BR.
    // +x is forward, +y is left and +z is counter-clockwise.
    v_mms[0] = linear_x - linear_y - angular_z * (kLx + kLy);  // FL
    v_mms[1] = linear_x + linear_y + angular_z * (kLx + kLy);  // FR
    v_mms[2] = linear_x + linear_y - angular_z * (kLx + kLy);  // BL
    v_mms[3] = linear_x - linear_y + angular_z * (kLx + kLy);  // BR
  } else {
    v_mms[0] = v_mms[2] = linear_x - angular_z * kDifferentialHalfTrack;  // FL, BL
    v_mms[1] = v_mms[3] = linear_x + angular_z * kDifferentialHalfTrack;  // FR, BR
  }
  for (int i = 0; i < kNumWheels; ++i) {
    final_target_vel_[i] = v_mms[i] * kMmsToRpm;
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
  const float kDeadband = 150.0f;
  if (std::abs(target_vel) > 0.1f) {
    float out = pid_output;
    if (std::abs(out) < kDeadband) {
      out = (out >= 0.0f) ? kDeadband : -kDeadband;
    }
    return static_cast<int16_t>(std::clamp(out, -kMaxPwm, kMaxPwm));
  }
  return 0;
}

void MotionController::Update(float dt, float current_imu_yaw_rad) {
  if (dt <= 0.0f) return;

  const float kAlpha = 0.35f;
  // 左侧电机/编码器与右侧是镜像安装，统一到“前进为正”的约定。
  const int sign[4] = {1, 1, 1, 1};

  for (int i = 0; i < kNumWheels; ++i) {
    int32_t current_count = encs_[i]->GetCount() * sign[i];
    float pps = (current_count - last_count_[i]) / dt;
    last_count_[i] = current_count;
    float raw_rpm = (pps / kPulsesPerWheelRev) * 60.0f;
    filtered_rpm_[i] = kAlpha * raw_rpm + (1.0f - kAlpha) * filtered_rpm_[i];
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
  for (int i = 0; i < kNumWheels; ++i) v[i] = filtered_rpm_[i] * kRpmToMms;

  if (chassis_type_ == ChassisType::kMecanum) {
    // X-pattern mecanum forward kinematics, inverse of CalculateKinematics().
    *linear_x = (v[0] + v[1] + v[2] + v[3]) / 4.0f;
    *linear_y = (-v[0] + v[1] + v[2] - v[3]) / 4.0f;
    *angular_z = (-v[0] + v[1] - v[2] + v[3]) / (4.0f * (kLx + kLy));
  } else {
    *linear_x = (v[0] + v[1] + v[2] + v[3]) / 4.0f;
    *linear_y = 0.0f;
    *angular_z = (-v[0] + v[1] - v[2] + v[3]) / (4.0f * kDifferentialHalfTrack);
  }
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
  final_target_vel_[motor_index] = target_mms * kMmsToRpm;
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
  final_target_vel_[0] = fl_mms * kMmsToRpm;
  final_target_vel_[1] = fr_mms * kMmsToRpm;
  final_target_vel_[2] = bl_mms * kMmsToRpm;
  final_target_vel_[3] = br_mms * kMmsToRpm;
}

void MotionController::SetMotorTargetPwm(MotorID motor_id, int16_t pwm) {
  control_mode_ = ControlMode::kDirectPwm;
  const int motor_index = static_cast<int>(motor_id);
  if (motor_index < 0 || motor_index >= kNumWheels) {
    return;
  }
  direct_pwm_[motor_index] = std::clamp<int>(pwm, -255, 255);
  target_vel_[motor_index] = 0.0f;
  final_target_vel_[motor_index] = 0.0f;
  pid_vel_[motor_index].Reset();
}

void MotionController::SetAllMotorTargetsPwm(
    int16_t fl_pwm, int16_t fr_pwm, int16_t bl_pwm, int16_t br_pwm) {
  control_mode_ = ControlMode::kDirectPwm;
  direct_pwm_[0] = std::clamp<int>(fl_pwm, -255, 255);
  direct_pwm_[1] = std::clamp<int>(fr_pwm, -255, 255);
  direct_pwm_[2] = std::clamp<int>(bl_pwm, -255, 255);
  direct_pwm_[3] = std::clamp<int>(br_pwm, -255, 255);
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
  return filtered_rpm_[motor_index] * kRpmToMms;
}

void MotionController::GetAllMotorVelocities(
    float* fl_mms, float* fr_mms, float* bl_mms, float* br_mms) const {
  if (fl_mms) *fl_mms = filtered_rpm_[0] * kRpmToMms;
  if (fr_mms) *fr_mms = filtered_rpm_[1] * kRpmToMms;
  if (bl_mms) *bl_mms = filtered_rpm_[2] * kRpmToMms;
  if (br_mms) *br_mms = filtered_rpm_[3] * kRpmToMms;
}
