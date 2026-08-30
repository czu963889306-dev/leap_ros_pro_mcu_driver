#ifndef MOTION_CONTROLLER_H_
#define MOTION_CONTROLLER_H_

#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid_controller.h"

enum class ControlMode { kVelocity, kDirectPwm };
enum class ChassisType { kMecanum, kDifferential };

enum class MotorID {
  kFrontLeft = 0, kFrontRight = 1, kBackLeft = 2, kBackRight = 3
};

class MotionController {
 public:
  MotionController(At8236Motor& fl, At8236Motor& fr, At8236Motor& bl, At8236Motor& br,
                   QuadratureEncoder& enc_fl, QuadratureEncoder& enc_fr, 
                   QuadratureEncoder& enc_bl, QuadratureEncoder& enc_br);
                   
  int16_t ApplyDeadband(float target_vel, float pid_output);
  void SetChassisType(ChassisType type);
  void SetMaxAcceleration(float accel_mms2);
  
  void GetOdometry(float* x, float* y, float* yaw) const;
  void GetOdometryQuaternion(float* qw, float* qx, float* qy, float* qz) const;
  float GetTotalDistance() const; // 新增：获取小车真实走过的总轨迹路程
  void ResetOdometry();

  void SetMotorTargetVelocity(MotorID motor_id, float target_mms);
  void SetAllMotorTargetsVelocity(float fl_mms, float fr_mms, float bl_mms, float br_mms);
  void SetMotorTargetPwm(MotorID motor_id, int16_t pwm);
  void SetAllMotorTargetsPwm(int16_t fl_pwm, int16_t fr_pwm, int16_t bl_pwm, int16_t br_pwm);
  float GetMotorVelocity(MotorID motor_id) const;
  void GetAllMotorVelocities(float* fl_mms, float* fr_mms, float* bl_mms, float* br_mms) const;

  void Drive(float linear_x, float linear_y, float angular_z);
  void GetVelocity(float* linear_x, float* linear_y, float* angular_z) const;
  void SetVelocityPidGains(float kp, float ki, float kd);
  void Update(float dt, float current_imu_yaw_rad);
  void Stop();

 private:
  void CalculateKinematics(float linear_x, float linear_y, float angular_z);
  static float NormalizeAngle(float angle);

  static constexpr int kNumWheels = 4;
  At8236Motor* motors_[kNumWheels];
  QuadratureEncoder* encs_[kNumWheels];
  PidController pid_vel_[kNumWheels];
  
  float target_vel_[kNumWheels] = {0};
  float final_target_vel_[kNumWheels] = {0};
  int16_t direct_pwm_[kNumWheels] = {0};
  int32_t last_count_[kNumWheels] = {0};
  float filtered_rpm_[kNumWheels] = {0};
  ControlMode control_mode_ = ControlMode::kVelocity;
  ChassisType chassis_type_ = ChassisType::kDifferential;

  float pos_x_ = 0.0f, pos_y_ = 0.0f, odom_yaw_ = 0.0f;
  float imu_yaw_rel_ = 0.0f;
  float total_distance_ = 0.0f; // 新增：累计轨迹路程(包含曲线)
  float yaw_offset_ = 0.0f;
  bool need_reset_yaw_offset_ = true;

  float max_accel_rpm_s_ = 200.0f; 
};

#endif // MOTION_CONTROLLER_H_
