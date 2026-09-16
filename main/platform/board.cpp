#include "board.h"
#include "arm_config.h"
#include "board_config.h"
#include "system_globals.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "BOARD_INIT";
namespace arm_cfg = leap_config::arm;
namespace board_cfg = leap_config::board;

// Arm servos 1..4 use the same angle convention as ArmKinematics servo_deg.
// The claw servo is not part of IK.
const float kServoStartupAngles[5] = {
    arm_cfg::kServoStartupAngles[0],
    arm_cfg::kServoStartupAngles[1],
    arm_cfg::kServoStartupAngles[2],
    arm_cfg::kServoStartupAngles[3],
    arm_cfg::kServoStartupAngles[4],
};
const TickType_t kServoStartupDelayTicks = pdMS_TO_TICKS(arm_cfg::kServoStartupDelayMs);
// GPIO0/BOOT is a strapping/download pin. Using it as a runtime button is risky,
// but it is enabled here intentionally for the BOOT double-click action.

// ================= 对象实例化 =================
Lsm6ds3Imu imu(board_cfg::kImuI2cPort, board_cfg::kImuSdaPin, board_cfg::kImuSclPin);

// Motor polarity is inverted here so SetSpeed(+) matches chassis forward.
At8236Motor motor_fl(board_cfg::kMotorFlIn2, board_cfg::kMotorFlIn1);
At8236Motor motor_fr(board_cfg::kMotorFrIn2, board_cfg::kMotorFrIn1);
At8236Motor motor_bl(board_cfg::kMotorBlIn2, board_cfg::kMotorBlIn1);
At8236Motor motor_br(board_cfg::kMotorBrIn2, board_cfg::kMotorBrIn1);

// 实例化编码器
QuadratureEncoder encoder_fl(board_cfg::kEncoderFlA, board_cfg::kEncoderFlB);
QuadratureEncoder encoder_fr(board_cfg::kEncoderFrA, board_cfg::kEncoderFrB);
QuadratureEncoder encoder_bl(board_cfg::kEncoderBlA, board_cfg::kEncoderBlB);
QuadratureEncoder encoder_br(board_cfg::kEncoderBrA, board_cfg::kEncoderBrB);

// 实例化外设
MotionController robot(motor_fl, motor_fr, motor_bl, motor_br, 
                       encoder_fl, encoder_fr, encoder_bl, encoder_br);
UltrasonicSensor ultrasonic(board_cfg::kUltrasonicPin);
Ws2812Driver rgb_led(board_cfg::kRgbPin, board_cfg::kRgbLedCount);
StatusLed status_led(board_cfg::kStatusLedPin);
ButtonDriver io0_button(board_cfg::kIo0ButtonPin, true);

ServoDriver servo_1(board_cfg::kServo1Pin, 0, 90.0f, arm_cfg::kServoReversed[0]);
ServoDriver servo_2(board_cfg::kServo2Pin, 0, 90.0f, arm_cfg::kServoReversed[1]);
ServoDriver servo_3(board_cfg::kServo3Pin, 1, 90.0f, arm_cfg::kServoReversed[2]);
ServoDriver servo_4(board_cfg::kServo4Pin, 1, 90.0f, arm_cfg::kServoReversed[3]);
ServoDriver servo_5(board_cfg::kServo5Pin, 1, kServoStartupAngles[4], arm_cfg::kServoReversed[4]);

// ================= 统一初始化逻辑 =================
void board_init() {
    ESP_LOGI(TAG, "Initializing hardware...");
    robot.SetChassisType(system_globals().runtime.chassis_type);
    motor_fl.Init(); encoder_fl.Init();
    motor_fr.Init(); encoder_fr.Init();
    motor_bl.Init(); encoder_bl.Init();
    motor_br.Init(); encoder_br.Init();
    ESP_LOGI(TAG, "Front and rear wheels initialized for %s chassis.",
             SystemGlobals::ChassisTypeToDisplayName(system_globals().runtime.chassis_type));

    // 2. 初始化传感器和外设
    ultrasonic.Init();
    rgb_led.Init();
    status_led.Init();
    io0_button.Init();
    ESP_LOGW(TAG, "BOOT/IO0 is used as a runtime user button. Holding it during reset may enter download mode.");
    const bool servos_ready =
        servo_1.Init() &&
        servo_2.Init() &&
        servo_3.Init() &&
        servo_4.Init();
    const bool claw_ready = servo_5.Init();
    const bool servos_ready_all = servos_ready && claw_ready;
    if (!servos_ready_all) {
        ESP_LOGE(TAG, "Servo initialization failed");
    } else {
        servo_1.SetAngle(kServoStartupAngles[0]);
        servo_2.SetAngle(kServoStartupAngles[1]);
        servo_3.SetAngle(kServoStartupAngles[2]);
        servo_4.SetAngle(kServoStartupAngles[3]);
        vTaskDelay(kServoStartupDelayTicks);
        servo_5.SetAngle(kServoStartupAngles[4]);
    }

    // 3. 初始化 IMU
    if (imu.Init()) {
        ESP_LOGI(TAG, "IMU (LSM6DS3) initialized.");
    } else {
        ESP_LOGE(TAG, "IMU Initialization FAILED!");
    }

    ESP_LOGI(TAG, "Hardware board ready.");
}
