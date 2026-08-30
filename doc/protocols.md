# Leap ROS Four MAVLink and micro-ROS Protocols

This document describes the network protocol surface implemented by `leap_ros_four`.

## Runtime Mode

The firmware supports one active communication mode at a time:

| Mode | Runtime value | Default |
| --- | --- | --- |
| micro-ROS | `micro_ros` | yes |
| MAVLink UDP | `mavlink_udp` | no |
| MAVLink UART | `uart_mavlink` | no |

The mode is stored in NVS under runtime configuration and can be changed from the web configuration page. The default micro-ROS agent is `192.168.31.214:8888`.

## Common Units

| Quantity | Internal unit | MAVLink unit | ROS unit |
| --- | --- | --- | --- |
| Linear position | mm | m | m |
| Linear velocity | mm/s | m/s | m/s |
| Yaw / angular velocity | rad / rad/s | rad / rad/s | rad / rad/s |
| IMU acceleration | g | mg in `SCALED_IMU`, m/s^2 in ROS | m/s^2 |
| IMU gyro | deg/s | rad/s or mrad/s | rad/s |
| Lidar distance | mm | cm in `OBSTACLE_DISTANCE` | m |

## Chassis Model

`leap_ros_four` uses a four-wheel differential chassis with four independent wheel motors and an arm servo set.

| Item | Value |
| --- | --- |
| Wheel layout | four-wheel differential |
| Left/right track width | `170 mm` |
| Differential effective track width | `340 mm` |
| Front/rear wheelbase | `175 mm` |
| Motor reduction ratio | `21.6:1` |
| Wheel diameter | `65 mm` |
| Arm channels | `servo 1..5` |

Wheel velocity and odometry calculations use the differential model in `main/control/motion_controller.cpp`.
ROS `angular.z` follows the standard right-hand convention: positive yaw turns
counter-clockwise/left, negative yaw turns clockwise/right. The odometry and yaw
command mapping use an effective differential track width of `340 mm`, calibrated
from turn behavior. A pure `1.0 rad/s` yaw command maps to about `-170 mm/s` on
the left wheels and `+170 mm/s` on the right wheels.

## micro-ROS

Transport: custom UDP transport over Wi-Fi.

| Item | Value |
| --- | --- |
| Node name | `leap_low_driver` |
| Local UDP port | `CONFIG_MICRO_ROS_LOCAL_PORT`, default `8888` |
| Agent address | runtime `g_microros_agent_ip:g_microros_agent_port`, default `192.168.31.214:8888` |
| Timer period | 20 ms |

### Subscribed Topics

| Topic | Type | QoS | Mapping |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | sensor data | `linear.x/y` in m/s -> target `vx/vy` in mm/s; `angular.z` -> target `wz` in rad/s |
| `/servo_cmd` | `geometry_msgs/msg/Vector3` | sensor data | `x` = servo channel `1..5`, `y` = angle in degrees; `z` unused |

Receiving `/cmd_vel` writes a velocity command to `q_motion_cmd`, marks the source as micro-ROS, and clears `g_emergency_stop`.

### Published Topics

| Topic | Type | Rate | Frame | Notes |
| --- | --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz | `odom`, child `base_link` | position and velocity from motion state |
| `/imu` | `sensor_msgs/msg/Imu` | 50 Hz | `imu_link` | quaternion, gyro, acceleration |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | `laser_frame` | 360 points, 1 degree increment, 0.02-12.0 m |
There is no battery topic in the current firmware.

### Services

| Service | Type | Request | Response |
| --- | --- | --- | --- |
| `/set_speed_pid` | `leap_interfaces/srv/SetSpeedPid` | `float32 kp`, `float32 ki`, `float32 kd` | `bool success`, applied `kp/ki/kd` |
| `/get_speed_pid` | `leap_interfaces/srv/GetSpeedPid` | empty | `bool success`, current `kp/ki/kd` |

Example calls:

```bash
ros2 service call /set_speed_pid leap_interfaces/srv/SetSpeedPid "{kp: 3.0, ki: 5.0, kd: 0.0}"
ros2 service call /get_speed_pid leap_interfaces/srv/GetSpeedPid "{}"
```

The PID services update the same speed PID command path used by the motor
controller. `/set_speed_pid` commits the values to NVS before reporting success,
so the PID values persist after reboot. They do not use the ROS parameter
server.

## MAVLink UDP

Transport: UDP socket bound to port `14550`.

| Item | Value |
| --- | --- |
| Local port | `14550` |
| Initial target | broadcast `255.255.255.255:14550` |
| Connected target | first MAVLink sender address |
| System ID | `1` |
| Component ID | `1` |
| Main loop period | 20 ms |
| Vehicle type | `MAV_TYPE_GROUND_ROVER` |
| Autopilot | `MAV_AUTOPILOT_GENERIC` |

## MAVLink UART

Transport: `UART0`, `230400` baud, 8N1. Runtime logs use the same UART0
serial port.

| Item | Value |
| --- | --- |
| TX pin | GPIO43 |
| RX pin | GPIO44 |
| Log port | shared UART0 |
| System ID | `1` |
| Component ID | `1` |
| Main loop period | 20 ms |

## MAVLink Messages

### Received Messages

| Message | Effect |
| --- | --- |
| `SET_POSITION_TARGET_LOCAL_NED` | Velocity command if vx/vy/yaw_rate fields are enabled; x/y/yaw position fields are not supported |
| `COMMAND_LONG` | Handles arm/disarm, odometry reset, servo, direct wheel speed, and speed PID update commands |
| `SET_RGB_LED` | Sets RGB LED directly |
| `SET_ACTUATOR_CONTROL_TARGET` | Uses `controls[0..2] * 255` as RGB |
| `PARAM_REQUEST_READ` | Returns one PID parameter |
| `PARAM_REQUEST_LIST` | Returns all PID parameters |
| `PARAM_SET` | Updates one PID parameter and returns the new value |

### COMMAND_LONG Commands

| Command | Parameters | Result |
| --- | --- | --- |
| `MAV_CMD_COMPONENT_ARM_DISARM` | `param1 < 0.5`: stop; otherwise clear emergency stop | `ACCEPTED` |
| `MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS` | none | reset odometry |
| `MAV_CMD_DO_SET_SERVO` | `param1`: servo channel 1-5, `param2`: servo angle | writes `q_servo_cmd` |
| `MAV_CMD_DO_SET_ACTUATOR` | `param1..param4`: FL/FR/BL/BR wheel targets | wheel-speed mode |
| `MAV_CMD_USER_4` | `param1`: PID target, `param2`: kp, `param3`: ki, `param4`: kd | updates speed PID |

Custom command IDs:

| Name | Value | Meaning |
| --- | --- | --- |
| `MATURO_MAV_CMD_SET_PID` | `MAV_CMD_USER_4` | set PID values |

PID target values:

| Value | Target |
| --- | --- |
| `1` | speed PID |

### Parameters

MAVLink parameter protocol exposes:

| Parameter | Type | Meaning |
| --- | --- | --- |
| `SPD_KP` | `MAV_PARAM_TYPE_REAL32` | speed PID kp |
| `SPD_KI` | `MAV_PARAM_TYPE_REAL32` | speed PID ki |
| `SPD_KD` | `MAV_PARAM_TYPE_REAL32` | speed PID kd |

### Published Messages

Published every 20 ms when source data is available:

| Message | Contents |
| --- | --- |
| `ATTITUDE` | roll/pitch/yaw and gyro in rad |
| `ATTITUDE_QUATERNION` | quaternion and gyro in rad/s |
| `SCALED_IMU` | acceleration in mg, gyro in mrad/s, magnetometer zero |
| `ODOMETRY` | local FLU pose and body FRD twist |

Published every 100 ms:

| Message | Contents |
| --- | --- |
| `DISTANCE_SENSOR` | ultrasonic distance, 2-400 cm |
| `DEBUG` | index `1`, motion busy as `0.0` or `1.0` |
| `RAW_RPM` | index `0..3`: FL/FR/BL/BR wheel RPM |

Published once per completed lidar scan:

| Message | Contents |
| --- | --- |
| `OBSTACLE_DISTANCE` | full 360 degree lidar scan split into five packets; each packet carries 72 bins at 1 degree per bin, with `angle_offset` set to `0/72/144/216/288` degrees |

Published every 1 s:

| Message | Contents |
| --- | --- |
| `HEARTBEAT` | rover heartbeat |
| `STATUSTEXT` | `DEVICE_NAME:<device_name>` |
| `ONBOARD_COMPUTER_STATUS` | heap, flash, Wi-Fi link estimate, board temperature if valid |

Published once after first MAVLink client is seen:

| Message | Contents |
| --- | --- |
| `COMPONENT_INFORMATION_BASIC` | vendor `Maturo`, model `Driver`, software `2026.05`, hardware `ESP32`, serial set to device name |

MAVLink publishing follows the active communication mode and transport behavior described above.

## Mode Switching Notes

Only the active mode sends communication traffic:

| Active mode | Behavior |
| --- | --- |
| `micro_ros` | micro-ROS entities are created and spun; MAVLink UDP and MAVLink UART do not publish or receive |
| `mavlink_udp` | MAVLink UDP task receives and publishes; micro-ROS and MAVLink UART are inactive |
| `uart_mavlink` | MAVLink UART task receives and publishes; micro-ROS and MAVLink UDP are inactive |

The web runtime configuration saves the selected mode and micro-ROS agent address in NVS, so the setting persists after reboot.
