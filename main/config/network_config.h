#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#define LEAP_CONFIG_DEFAULT_MICROROS_AGENT_IP "192.168.31.214"

namespace leap_config {
namespace network {

static constexpr uint32_t kWifiConnectTimeoutMs = 15000;
static constexpr const char kWifiCfgNamespace[] = "wifi_cfg";
static constexpr const char kWifiCfgSsidKey[] = "sta_ssid";
static constexpr const char kWifiCfgPasswordKey[] = "sta_pass";
static constexpr const char kWifiCfgProvisionOnlyKey[] = "provision_only";
static constexpr const char kRuntimeCfgNamespace[] = "runtime_cfg";
static constexpr const char kRuntimeCfgCommModeKey[] = "comm_mode";
static constexpr const char kRuntimeCfgChassisTypeKey[] = "chassis";
static constexpr const char kRuntimeCfgMicroRosAgentIpKey[] = "uros_ip";
static constexpr const char kRuntimeCfgMicroRosAgentPortKey[] = "uros_port";

static constexpr const char kDefaultStaSsid[] = "Maturo";
static constexpr const char kDefaultStaPassword[] = "maturo2026";
static constexpr const char kDefaultMicroRosAgentIp[] = LEAP_CONFIG_DEFAULT_MICROROS_AGENT_IP;
static constexpr uint16_t kDefaultMicroRosAgentPort = 8888;
static constexpr uint16_t kMicroRosLocalPort = 8888;

static constexpr uart_port_t kMicroRosUartPort = UART_NUM_2;
static constexpr gpio_num_t kMicroRosUartTxPin = GPIO_NUM_37;
static constexpr gpio_num_t kMicroRosUartRxPin = GPIO_NUM_38;
static constexpr int kMicroRosUartBaudRate = 921600;
static constexpr size_t kMicroRosUartRxBufferSize = 4096;
static constexpr size_t kMicroRosUartTxBufferSize = 4096;
static constexpr int kMicroRosUartReadMaxWaitMs = 5;
static constexpr int kMicroRosUartStartReadWaitMs = 100;
static constexpr uint32_t kMicroRosUartStartLogPolls = 50;
static constexpr const char kMicroRosUartStartCommand[] = "MATURO_MICROROS_START";

static constexpr uart_port_t kLidarUartPort = UART_NUM_1;
static constexpr int kLidarBaudRate = 230400;
static constexpr uint32_t kLidarTaskPeriodMs = 10;

}  // namespace network
}  // namespace leap_config
