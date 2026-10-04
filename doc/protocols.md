# Leap ROS Four micro-ROS Protocol

This document describes the micro-ROS communication and motion-control surface
implemented by `leap_ros_four`.

## Runtime Mode

The firmware supports two micro-ROS transports:

| Mode | Runtime value | Default |
| --- | --- | --- |
| micro-ROS UDP | `micro_ros` | no |
| micro-ROS UART | `microros_uart` | yes |

The mode is stored in NVS and can be changed from the web configuration page or
with a short BOOT-button press. Unsupported saved mode values are rejected and the
firmware falls back to the default micro-ROS UART configuration.

UDP uses the runtime agent address, default `192.168.31.214:8888`. UART uses
UART2, GPIO37 TX, GPIO38 RX, 921600 baud, 8N1, without hardware flow control.
In UART mode, the firmware keeps the UART silent until it receives the host
start command `MATURO_MICROROS_START` on UART2. The AML host sends this command
after Linux has booted, then starts the micro-ROS agent.
The host watches `/wifi_config` as the ESP32 heartbeat and resumes sending the
start command if that topic becomes stale. The ESP32 watches `/host_info` as the
Linux heartbeat in UART mode; if it does not receive host info after startup, or
the heartbeat becomes stale, it stops micro-ROS and waits for the host start
command again.

The same web runtime form selects the chassis model. The value is stored in NVS
as `mecanum` or `differential`, then applied after the automatic restart.

## Units

| Quantity | Internal unit | ROS unit |
| --- | --- | --- |
| Linear position | mm | m |
| Linear velocity | mm/s | m/s |
| Yaw / angular velocity | rad / rad/s | rad / rad/s |
| IMU acceleration | g | m/s^2 |
| IMU gyro | deg/s | rad/s |
| Lidar distance | mm | m |

## Mecanum Chassis Model

The default chassis is a four-wheel X-pattern mecanum base; the web page can
switch to the four-wheel differential model. Wheel order is front left (FL),
front right (FR), back left (BL), back right (BR). Body coordinates
follow ROS conventions: positive `x` is forward, positive `y` is left, and
positive `angular.z` is counter-clockwise.

Geometry:

| Item | Value |
| --- | --- |
| Wheel diameter | `80 mm` |
| Wheelbase | `175 mm` |
| Track width | `170 mm` |
| Motor reduction ratio | `46.8:1` |

For `a = wheelbase / 2 + track_width / 2`, inverse kinematics are:

```text
FL = vx - vy - a*wz
FR = vx + vy + a*wz
BL = vx + vy - a*wz
BR = vx - vy + a*wz
```

Forward kinematics used for odometry are:

```text
vx = (FL + FR + BL + BR) / 4
vy = (-FL + FR + BL - BR) / 4
wz = (-FL + FR - BL + BR) / (4*a)
```

`/cmd_vel` therefore supports forward/backward, lateral translation, rotation,
and combined holonomic motion. Encoder-derived `vx`, `vy`, and `wz` are integrated
into planar odometry.

In `differential` mode, `linear.y` is ignored, the left wheels share one target,
the right wheels share one target, and reported lateral velocity is zero. The
effective differential track width is `340 mm`.

## micro-ROS Interface

| Item | Value |
| --- | --- |
| Node name | `leap_ros_pro` |
| UDP local port | `CONFIG_MICRO_ROS_LOCAL_PORT`, default `8888` |
| Timer period | 20 ms |

### Subscribed Topics

| Topic | Type | Mapping |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x/y` m/s to `vx/vy` mm/s; `angular.z` to `wz` rad/s |
| `/servo_cmd` | `geometry_msgs/msg/Vector3` | `x` = servo channel 1..5; `y` = angle in degrees |
| `/host_info` | `std_msgs/msg/String` | Target host info text: `hostname=<name> ip=<ipv4>` or `hostname=<name> hostip=<ipv4>` |

Receiving `/cmd_vel` writes velocity mode to `q_motion_cmd` and clears the
emergency stop.

### I2C Gamepad Control

The firmware polls the I2C gamepad at address `0x50` at 20 Hz. Valid stick
input is placed in a separate manual-override command path with the same
velocity units as `/cmd_vel`; it does not overwrite the navigation command.

| Gamepad input | Motion command | Maximum |
| --- | --- | --- |
| Left stick forward/back channel | `linear.x` | `300 mm/s` (`0.3 m/s`) |
| Right stick turn channel | `angular.z` | `2.5 rad/s` |

The raw stick range is `-127..127` with a dead zone of 12. A non-zero stick
acts as a manual override and is published at 20 Hz. When the sticks return to
the dead zone, the firmware only clears the manual-override flag; it does not
send a zero `/cmd_vel`. The most recent navigation command resumes immediately.
A disconnect or invalid frame also clears the override without overwriting the
navigation command.

### Published Topics

| Topic | Type | Rate | Frame | Notes |
| --- | --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz | `odom` / `base_link` | Pose and body velocity for the selected chassis model |
| `/imu` | `sensor_msgs/msg/Imu` | 50 Hz | `imu_link` | Quaternion, gyro, acceleration |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | `laser_frame` | 360 points, 0.02-12.0 m |
| `/arm_pose` | `geometry_msgs/msg/Pose` | 50 Hz | n/a | Current arm Cartesian pose |
| `/wifi_config` | `std_msgs/msg/String` | 1 Hz | n/a | Saved STA credentials as JSON: `{"ssid":"<name>","password":"<password>"}` |

### Services

| Service | Type | Effect |
| --- | --- | --- |
| `/set_arm_pose` | `leap_interfaces/srv/SetArmPose` | Accepts Cartesian arm target `x/y/z` in metres plus `pitch_deg` |
| `/set_speed_pid` | `leap_interfaces/srv/SetSpeedPid` | Applies and persists wheel speed PID gains |
| `/get_speed_pid` | `leap_interfaces/srv/GetSpeedPid` | Returns current wheel speed PID gains |

Only the selected micro-ROS transport is active. UDP starts after Wi-Fi STA
connects; UART waits for `MATURO_MICROROS_START` from the host before starting.
