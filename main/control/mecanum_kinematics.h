#pragma once

#include "motion_kinematics.h"

class MecanumKinematics final : public MotionKinematics {
 public:
  void Inverse(
      float linear_x,
      float linear_y,
      float angular_z,
      float wheel_mms[4]) const override;

  void Forward(
      const float wheel_mms[4],
      float* linear_x,
      float* linear_y,
      float* angular_z) const override;
};
