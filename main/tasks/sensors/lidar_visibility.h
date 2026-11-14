#pragma once

// 210 one-degree bins centered on the forward (0-degree) direction.
constexpr bool lidar_angle_visible(int angle_deg) {
    return angle_deg < 105 || angle_deg >= 255;
}
