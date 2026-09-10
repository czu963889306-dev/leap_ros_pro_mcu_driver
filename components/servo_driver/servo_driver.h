#ifndef SERVO_DRIVER_H_
#define SERVO_DRIVER_H_

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h" // 引入 MCPWM 库

class ServoDriver {
 public:
  // default_angle and SetAngle use the caller's calibrated command angle.
  // When reversed is true, the physical PWM angle is mirrored as 180 - angle.
  ServoDriver(gpio_num_t pin, int group_id, float default_angle = 90.0f, bool reversed = false);

  bool Init();
  void SetAngle(float angle);

 private:
  gpio_num_t pin_;
  int group_id_;
  float default_angle_;
  bool reversed_;
  mcpwm_timer_handle_t timer_;
  mcpwm_oper_handle_t oper_;
  mcpwm_cmpr_handle_t comparator_; // 比较器句柄，用于调节占空比
  mcpwm_gen_handle_t generator_;
  
  // 舵机脉宽参数
  const uint32_t kMinPulseWidthUs = 500;
  const uint32_t kMaxPulseWidthUs = 2500;
  const float kMaxAngle = 180.0f;
};

#endif  // SERVO_DRIVER_H_
