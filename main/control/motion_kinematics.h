#pragma once

class MotionKinematics {
 public:
  virtual ~MotionKinematics() = default;

  virtual void Inverse(
      float linear_x,
      float linear_y,
      float angular_z,
      float wheel_mms[4]) const = 0;

  virtual void Forward(
      const float wheel_mms[4],
      float* linear_x,
      float* linear_y,
      float* angular_z) const = 0;
};
