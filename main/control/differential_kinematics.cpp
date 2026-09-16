#include "differential_kinematics.h"

#include "motion_config.h"

namespace motion_cfg = leap_config::motion;

void DifferentialKinematics::Inverse(
    float linear_x,
    float,
    float angular_z,
    float wheel_mms[4]) const {
  if (wheel_mms == nullptr) {
    return;
  }

  wheel_mms[0] = linear_x - angular_z * motion_cfg::kDifferentialHalfTrackMm;  // FL
  wheel_mms[1] = linear_x + angular_z * motion_cfg::kDifferentialHalfTrackMm;  // FR
  wheel_mms[2] = wheel_mms[0];                                                 // BL
  wheel_mms[3] = wheel_mms[1];                                                 // BR
}

void DifferentialKinematics::Forward(
    const float wheel_mms[4],
    float* linear_x,
    float* linear_y,
    float* angular_z) const {
  if (wheel_mms == nullptr || linear_x == nullptr ||
      linear_y == nullptr || angular_z == nullptr) {
    return;
  }

  *linear_x = (wheel_mms[0] + wheel_mms[1] + wheel_mms[2] + wheel_mms[3]) / 4.0f;
  *linear_y = 0.0f;
  *angular_z = (-wheel_mms[0] + wheel_mms[1] - wheel_mms[2] + wheel_mms[3]) /
               (4.0f * motion_cfg::kDifferentialHalfTrackMm);
}
