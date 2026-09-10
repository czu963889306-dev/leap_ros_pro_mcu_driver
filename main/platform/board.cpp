#include "board.h"
#include "system_globals.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "BOARD_INIT";

// 电机引脚 (M3=FL, M4=FR, M2=BL, M1=BR)
const gpio_num_t kMotorFL_In1 = GPIO_NUM_4;  const gpio_num_t kMotorFL_In2 = GPIO_NUM_5;
const gpio_num_t kEncoderFL_A = GPIO_NUM_6;  const gpio_num_t kEncoderFL_B = GPIO_NUM_7;

const gpio_num_t kMotorFR_In1 = GPIO_NUM_3;  const gpio_num_t kMotorFR_In2 = GPIO_NUM_8;
const gpio_num_t kEncoderFR_A = GPIO_NUM_19; const gpio_num_t kEncoderFR_B = GPIO_NUM_20;

const gpio_num_t kMotorBL_In1 = GPIO_NUM_15; const gpio_num_t kMotorBL_In2 = GPIO_NUM_16;
const gpio_num_t kEncoderBL_A = GPIO_NUM_17; const gpio_num_t kEncoderBL_B = GPIO_NUM_18;

const gpio_num_t kMotorBR_In1 = GPIO_NUM_9; const gpio_num_t kMotorBR_In2 = GPIO_NUM_46;
const gpio_num_t kEncoderBR_A = GPIO_NUM_11; const gpio_num_t kEncoderBR_B = GPIO_NUM_10;


const gpio_num_t kUltrasonicPin = GPIO_NUM_21;
const gpio_num_t kRgbPin = GPIO_NUM_47;



const gpio_num_t kStatusLedPin = GPIO_NUM_14;
const gpio_num_t kServo1Pin = GPIO_NUM_36;
const gpio_num_t kServo2Pin = GPIO_NUM_35;
const gpio_num_t kServo3Pin = GPIO_NUM_45;
const gpio_num_t kServo4Pin = GPIO_NUM_48;
const gpio_num_t kServo5Pin = GPIO_NUM_1;
// Arm servos 1..4 use the same angle convention as ArmKinematics servo_deg.
// The claw servo is not part of IK.
const float kServoStartupAngles[5] = {90.0f, 0.0f, 180.0f, 180.0f, 180.0f};
const TickType_t kServoStartupDelayTicks = pdMS_TO_TICKS(2000);
// GPIO0/BOOT is a strapping/download pin. Using it as a runtime button is risky,
// but it is enabled here intentionally for the BOOT double-click action.
const gpio_num_t kIo0ButtonPin = GPIO_NUM_0;

// ================= 对象实例化 =================
Lsm6ds3Imu imu(I2C_NUM_0, GPIO_NUM_12, GPIO_NUM_13);

// Motor polarity is inverted here so SetSpeed(+) matches chassis forward.
At8236Motor motor_fl(kMotorFL_In2, kMotorFL_In1);
At8236Motor motor_fr(kMotorFR_In2, kMotorFR_In1);
At8236Motor motor_bl(kMotorBL_In2, kMotorBL_In1);
At8236Motor motor_br(kMotorBR_In2, kMotorBR_In1);

// 实例化编码器
QuadratureEncoder encoder_fl(kEncoderFL_A, kEncoderFL_B);
QuadratureEncoder encoder_fr(kEncoderFR_A, kEncoderFR_B);
QuadratureEncoder encoder_bl(kEncoderBL_A, kEncoderBL_B);
QuadratureEncoder encoder_br(kEncoderBR_A, kEncoderBR_B);

// 实例化外设
MotionController robot(motor_fl, motor_fr, motor_bl, motor_br, 
                       encoder_fl, encoder_fr, encoder_bl, encoder_br);
UltrasonicSensor ultrasonic(kUltrasonicPin);
Ws2812Driver rgb_led(kRgbPin, 6);
StatusLed status_led(kStatusLedPin);
ButtonDriver io0_button(kIo0ButtonPin, true);
static constexpr bool kServo1Reversed = false;
static constexpr bool kServo2Reversed = true;
static constexpr bool kServo3Reversed = true;
static constexpr bool kServo4Reversed = true;
static constexpr bool kServo5Reversed = false;

ServoDriver servo_1(kServo1Pin, 0, 90.0f, kServo1Reversed);
ServoDriver servo_2(kServo2Pin, 0, 90.0f, kServo2Reversed);
ServoDriver servo_3(kServo3Pin, 1, 90.0f, kServo3Reversed);
ServoDriver servo_4(kServo4Pin, 1, 90.0f, kServo4Reversed);
ServoDriver servo_5(kServo5Pin, 1, kServoStartupAngles[4], kServo5Reversed);

// ================= 统一初始化逻辑 =================
void board_init() {
    ESP_LOGI(TAG, "Initializing hardware...");
    robot.SetChassisType(g_chassis_type);
    motor_fl.Init(); encoder_fl.Init();
    motor_fr.Init(); encoder_fr.Init();
    motor_bl.Init(); encoder_bl.Init();
    motor_br.Init(); encoder_br.Init();
    ESP_LOGI(TAG, "Front and rear wheels initialized for %s chassis.",
             chassis_type_to_display_name(g_chassis_type));

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
        // vTaskDelay(kServoStartupDelayTicks);
        servo_2.SetAngle(kServoStartupAngles[1]);
        // vTaskDelay(kServoStartupDelayTicks);
        servo_3.SetAngle(kServoStartupAngles[2]);
        // vTaskDelay(kServoStartupDelayTicks);
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
