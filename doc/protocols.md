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
UART0, GPIO43 TX, GPIO44 RX, 921600 baud, 8N1, without hardware flow control.
In UART mode, the micro-ROS task waits 5 seconds before initializing UART0 and
connecting to the agent, leaving time for startup Wi-Fi/IP console logs.
UDP starts after Wi-Fi STA connects. Neither transport requires a separate host
handshake or heartbeat gate.

The same web runtime form selects the chassis model. The value is stored in NVS
as `mecanum` or `differential`, then applied after the automatic restart.

## Sensor time synchronization

Both transports synchronize with the micro-ROS Agent when creating a session
(up to 1000 ms), then refresh every 5 seconds. Runtime exchanges wait up to
50 ms on the existing ROS task; failures retry every second without recreating
the session or disabling command subscriptions. Scheduling uses the monotonic
ESP timer, independent of host clock adjustments.

`/odom`, `/imu`, `/scan` and `/battery_state` are published only after a successful
time exchange. A confirmed epoch is extrapolated with the monotonic timer for
up to 30 seconds during temporary sync failures; after that stamped data pauses
until synchronization succeeds. `/arm_pose` and `/wifi_config` have no timestamp
and continue publishing. A backward clock correction pauses stamped data until
the corrected time passes the previous stamp, rather than emitting decreasing
or artificially future timestamps. Large corrections and failure/recovery are
logged under `microros_time`.

The host should still finish NTP synchronization before starting the Agent:
MCU synchronization follows the Agent clock and cannot establish whether that
clock is correct relative to UTC. Later host adjustments are picked up by the
periodic exchange. Brief publication delays can occur during these exchanges or
backward corrections; nominal publication rates remain unchanged.

Host regression tests (no hardware required): `bash tests/run_time_sync_tests.sh`.

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
| `/host_info` | `std_msgs/msg/String` | Optional target host info text: `hostname=<name> ip=<ipv4>` or `hostname=<name> hostip=<ipv4>` |

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
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 2 Hz | `battery` | Pack voltage in V and estimated percentage in 0..1 |
| `/wifi_config` | `std_msgs/msg/String` | 1 Hz | n/a | Saved STA credentials as JSON: `{"ssid":"<name>","password":"<password>"}` |

### Battery Measurement

GPIO2 uses ADC1 channel 1 on ESP32-S3 with 12 dB attenuation and calibrated
voltage conversion. Every 500 ms, 16 readings are averaged. Invalid samples
are excluded; an entirely failed measurement is not published. If ADC
calibration is unavailable, the battery task stops instead of reporting an
uncalibrated voltage. UDP and UART publish the same battery message using
sensor-data QoS (best effort, volatile).

`main/config/board_config.h` contains the voltage and divider settings:

- `kBatteryFullVoltage = 12.6V`, `kBatteryEmptyVoltage = 9.0V` (3S assumption).
- `kBatteryDividerRatio = (12.6 / 2.6857) * (12.07 / 7.9)` (approximately
  `7.167921`) applies the measured correction: the old coefficient reported
  7.90V while the actual battery voltage was 12.07V. This single-point gain
  calibration applies to both the web page and micro-ROS battery readings.
  Confirm the physical divider keeps GPIO2 within the ADC input range;
  changing this software factor does not change the voltage on the pin.
- Voltage is `V_GPIO2 * kBatteryDividerRatio`; percentage is the rounded,
  clamped linear estimate `100 * (V_battery - 9.0) / (12.6 - 9.0)`.
  ROS reports that integer percentage divided by 100. This is a voltage-based
  estimate, not a coulomb counter.

Unknown temperature/current/charge/capacity fields are `NaN`, charging and
health status are unknown, and per-cell arrays remain empty.

The web status page also shows battery voltage, percentage (0..100), and ADC
voltage in its Sensors panel, refreshed every second through `/api/status`.
The JSON `battery` object contains `valid`, `voltage_v`, `adc_voltage_v`,
`percentage`, and `raw`, matching `leap_ros`. Invalid readings display No Data.
The web page reads the same battery queue without consuming it or requiring a
micro-ROS agent connection.

There are exactly **6 publishers**, matching `RMW_UXRCE_MAX_PUBLISHERS=6` in
`app-colcon.meta`; rosout is disabled. There are still 3 subscriptions and
3 services, each with its own pool. A compile-time check rejects a stale
micro-ROS header with a different publisher limit.

The managed component caches `libmicroros.a`; changing `app-colcon.meta`
alone does not rebuild it. To rebuild the cached library from existing
sources after changing the limit, run in the ESP-IDF environment:

```bash
touch managed_components/micro_ros_espidf_component/esp32_toolchain.cmake
rm -f managed_components/micro_ros_espidf_component/libmicroros.a
rm -rf managed_components/micro_ros_espidf_component/include
idf.py reconfigure build
```

The removed files are generated artifacts. Removing the exported `include`
directory also prevents the component's recursive copy from nesting new headers
under `include/include` while leaving the old public headers in use.

For an agent-side check:

```bash
ros2 topic echo /battery_state sensor_msgs/msg/BatteryState --qos-reliability best_effort
ros2 topic hz /battery_state
```

### Services

| Service | Type | Effect |
| --- | --- | --- |
| `/set_arm_pose` | `leap_interfaces/srv/SetArmPose` | Accepts Cartesian arm target `x/y/z` in metres plus `pitch_deg` |
| `/set_speed_pid` | `leap_interfaces/srv/SetSpeedPid` | Applies and persists wheel speed PID gains |
| `/get_speed_pid` | `leap_interfaces/srv/GetSpeedPid` | Returns current wheel speed PID gains |

Only the selected micro-ROS transport is active. UDP starts after Wi-Fi STA
connects; UART starts after a 5-second startup delay when UART mode is selected.
