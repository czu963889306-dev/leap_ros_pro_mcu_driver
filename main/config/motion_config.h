#pragma once

#include <stdint.h>

namespace leap_config {
namespace motion {

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kDegToRad = kPi / 180.0f;

static constexpr float kDifferentialWheelDiameterMm = 65.0f;
static constexpr float kMecanumWheelDiameterMm = 80.0f;
static constexpr float kMecanumWheelbaseMm = 175.0f;
static constexpr float kMecanumTrackWidthMm = 170.0f;
static constexpr float kDifferentialEffectiveTrackWidthMm = 340.0f;
static constexpr float kMecanumHalfWheelbaseMm = kMecanumWheelbaseMm / 2.0f;
static constexpr float kMecanumHalfTrackWidthMm = kMecanumTrackWidthMm / 2.0f;
static constexpr float kDifferentialHalfTrackMm = kDifferentialEffectiveTrackWidthMm / 2.0f;
static constexpr float kDifferentialRpmToMmS =
    (kPi * kDifferentialWheelDiameterMm) / 60.0f;
static constexpr float kMecanumRpmToMmS =
    (kPi * kMecanumWheelDiameterMm) / 60.0f;

static constexpr float kGearReduction = 46.8f;
static constexpr float kEncoderCountsPerMotorRev = 44.0f;
static constexpr float kPulsesPerWheelRev = kEncoderCountsPerMotorRev * kGearReduction;

static constexpr float kDefaultSpeedKp = 5.0f;
static constexpr float kDefaultSpeedKi = 10.0f;
static constexpr float kDefaultSpeedKd = 0.0f;
static constexpr float kMaxPwm = 255.0f;
static constexpr float kMotorPwmDeadband = 150.0f;
static constexpr float kVelocityFilterAlpha = 0.35f;
static constexpr int kEncoderSigns[4] = {1, 1, 1, 1};

static constexpr float kControlLoopDtS = 0.02f;
static constexpr uint32_t kControlLoopPeriodMs = 20;

}  // namespace motion
}  // namespace leap_config
