#ifndef LDSTL26_LIDAR_H_
#define LDSTL26_LIDAR_H_

#include <stdint.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#define MAX_LIDAR_POINTS_PER_SCAN 500
#define LDSTL26_PACKET_SIZE 47
#define LDSTL26_POINTS_PER_PACKET 12

struct LidarPoint {
    float angle;
    float distance;
};

class Ldstl26Lidar {
public:
    Ldstl26Lidar(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate = 230400);
    void Init();
    bool Poll(LidarPoint* out_scan, uint16_t* out_count);

private:
    uart_port_t uart_num_;
    gpio_num_t tx_pin_, rx_pin_;
    int baud_rate_;

    uint8_t state_, packet_idx_;
    uint8_t packet_[LDSTL26_PACKET_SIZE];

    float last_angle_;
    LidarPoint current_scan_[MAX_LIDAR_POINTS_PER_SCAN];
    uint16_t current_point_count_;

    bool ProcessByte(uint8_t data, LidarPoint* out_scan, uint16_t* out_count);
    uint8_t ParsePacket(const uint8_t* buffer, LidarPoint* points);
    uint8_t CRC8(const uint8_t* data, uint16_t len);
};

#endif