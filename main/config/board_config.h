#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"

namespace leap_config {
namespace board {

static constexpr gpio_num_t kMotorFlIn1 = GPIO_NUM_4;
static constexpr gpio_num_t kMotorFlIn2 = GPIO_NUM_5;
static constexpr gpio_num_t kMotorFrIn1 = GPIO_NUM_3;
static constexpr gpio_num_t kMotorFrIn2 = GPIO_NUM_8;
static constexpr gpio_num_t kMotorBlIn1 = GPIO_NUM_15;
static constexpr gpio_num_t kMotorBlIn2 = GPIO_NUM_16;
static constexpr gpio_num_t kMotorBrIn1 = GPIO_NUM_9;
static constexpr gpio_num_t kMotorBrIn2 = GPIO_NUM_46;

static constexpr gpio_num_t kEncoderFlA = GPIO_NUM_6;
static constexpr gpio_num_t kEncoderFlB = GPIO_NUM_7;
static constexpr gpio_num_t kEncoderFrA = GPIO_NUM_19;
static constexpr gpio_num_t kEncoderFrB = GPIO_NUM_20;
static constexpr gpio_num_t kEncoderBlA = GPIO_NUM_17;
static constexpr gpio_num_t kEncoderBlB = GPIO_NUM_18;
static constexpr gpio_num_t kEncoderBrA = GPIO_NUM_11;
static constexpr gpio_num_t kEncoderBrB = GPIO_NUM_10;

static constexpr i2c_port_t kImuI2cPort = I2C_NUM_0;
static constexpr gpio_num_t kImuSdaPin = GPIO_NUM_12;
static constexpr gpio_num_t kImuSclPin = GPIO_NUM_13;

static constexpr gpio_num_t kUltrasonicPin = GPIO_NUM_21;
static constexpr gpio_num_t kRgbPin = GPIO_NUM_47;
static constexpr uint16_t kRgbLedCount = 6;
static constexpr gpio_num_t kStatusLedPin = GPIO_NUM_14;
static constexpr gpio_num_t kIo0ButtonPin = GPIO_NUM_0;

static constexpr gpio_num_t kServo1Pin = GPIO_NUM_36;
static constexpr gpio_num_t kServo2Pin = GPIO_NUM_35;
static constexpr gpio_num_t kServo3Pin = GPIO_NUM_45;
static constexpr gpio_num_t kServo4Pin = GPIO_NUM_48;
static constexpr gpio_num_t kServo5Pin = GPIO_NUM_1;

static constexpr gpio_num_t kLidarTxPin = static_cast<gpio_num_t>(UART_PIN_NO_CHANGE);
static constexpr gpio_num_t kLidarRxPin = GPIO_NUM_41;

}  // namespace board
}  // namespace leap_config
