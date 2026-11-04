#pragma once

#include <math.h>
#include <stddef.h>

#include "arm_config.h"

// Internal queue payload; the legacy packed ServoMsg/UDP format is unchanged.
struct ServosMsg {
    float angles[leap_config::arm::kServoCount] = {};
};

inline bool make_servos_msg(const float *angles, size_t count, ServosMsg *out) {
    if (angles == nullptr || out == nullptr || count != leap_config::arm::kServoCount) {
        return false;
    }
    for (size_t i = 0; i < count; ++i) {
        if (!isfinite(angles[i]) || angles[i] < 0.0f || angles[i] > 180.0f) {
            return false;
        }
    }
    for (size_t i = 0; i < count; ++i) {
        out->angles[i] = angles[i];
    }
    return true;
}
