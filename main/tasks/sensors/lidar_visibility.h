#pragma once

// Original forward-facing mask: 0–89 and 269–359 degrees remain visible.
constexpr bool lidar_angle_visible(int angle_deg) {
    return angle_deg < 90 || angle_deg >= 269;
}
