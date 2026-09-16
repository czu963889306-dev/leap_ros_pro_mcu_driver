#include "mecanum_kinematics.h"

#include "motion_config.h"

namespace motion_cfg = leap_config::motion;

namespace {

float MecanumRotationRadiusMm() {
  return motion_cfg::kMecanumHalfWheelbaseMm +
         motion_cfg::kMecanumHalfTrackWidthMm;
}

}  // namespace

void MecanumKinematics::Inverse(
    float linear_x,
    float linear_y,
    float angular_z,
    float wheel_mms[4]) const {
  if (wheel_mms == nullptr) {
    return;
  }

  const float radius = MecanumRotationRadiusMm();
  // X-pattern mecanum inverse kinematics. Wheel order: FL, FR, BL, BR.
  // +x is forward, +y is left and +z is counter-clockwise.
  wheel_mms[0] = linear_x - linear_y - angular_z * radius;  // FL
  wheel_mms[1] = linear_x + linear_y + angular_z * radius;  // FR
  wheel_mms[2] = linear_x + linear_y - angular_z * radius;  // BL
  wheel_mms[3] = linear_x - linear_y + angular_z * radius;  // BR
}

void MecanumKinematics::Forward(
    const float wheel_mms[4],
    float* linear_x,
    float* linear_y,
    float* angular_z) const {
  if (wheel_mms == nullptr || linear_x == nullptr ||
      linear_y == nullptr || angular_z == nullptr) {
    return;
  }

  const float radius = MecanumRotationRadiusMm();
  // X-pattern mecanum forward kinematics, inverse of Inverse().
  *linear_x = (wheel_mms[0] + wheel_mms[1] + wheel_mms[2] + wheel_mms[3]) / 4.0f;
  *linear_y = (-wheel_mms[0] + wheel_mms[1] + wheel_mms[2] - wheel_mms[3]) / 4.0f;
  *angular_z = (-wheel_mms[0] + wheel_mms[1] - wheel_mms[2] + wheel_mms[3]) /
               (4.0f * radius);
}
