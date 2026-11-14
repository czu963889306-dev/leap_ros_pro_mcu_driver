#include "ldstl26_lidar.h"
#include "lidar_visibility.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <vector>

static std::deque<uint8_t> rx;
int uart_driver_install(uart_port_t, int, int, int, void*, int) { return 0; }
int uart_param_config(uart_port_t, const uart_config_t*) { return 0; }
int uart_set_pin(uart_port_t, int, int, int, int) { return 0; }
int uart_get_buffered_data_len(uart_port_t, size_t* size) { *size = rx.size(); return 0; }
int uart_read_bytes(uart_port_t, void* dest, uint32_t size, int) {
    auto* bytes = static_cast<uint8_t*>(dest);
    const auto count = std::min<size_t>(size, rx.size());
    for (size_t i = 0; i < count; ++i) { bytes[i] = rx.front(); rx.pop_front(); }
    return static_cast<int>(count);
}
static void put16(std::vector<uint8_t>& b, size_t i, uint16_t x) { b[i] = x & 255; b[i+1] = x >> 8; }
static std::vector<uint8_t> packet(int index) {
    std::vector<uint8_t> b(47);
    b[0] = 0x54; b[1] = 0x2c;
    put16(b, 4, index * 300); put16(b, 42, index * 300 + 275);
    for (int i = 0; i < 12; ++i) {
        uint16_t distance = 1000;
        if (index == 60 && i == 0) distance = 50;
        if (index == 60 && i == 2) distance = 700;
        put16(b, 6+i*3, distance);
    }
    uint16_t remainder = 0;
    for (size_t i = 0; i < 46; ++i) {
        remainder ^= static_cast<uint16_t>(b[i]) << 8;
        for (int bit = 15; bit >= 8; --bit) {
            if (remainder & (1U << bit)) remainder ^= 0x14d << (bit - 8);
        }
        remainder <<= 8;
    }
    b[46] = remainder >> 8;
    return b;
}
struct Scan { LidarPoint points[MAX_LIDAR_POINTS_PER_SCAN]; uint64_t guard = 0xabcdef1234567890ULL; uint16_t count = 0; };
static bool feed(Ldstl26Lidar& driver, const std::vector<uint8_t>& bytes, Scan& scan) {
    bool complete = false;
    for (size_t i = 0; i < bytes.size(); i += 7) {
        const size_t end = std::min(i+7, bytes.size());
        rx.insert(rx.end(), bytes.begin()+i, bytes.begin()+end);
        complete = driver.Poll(scan.points, &scan.count) || complete;
        assert(scan.guard == 0xabcdef1234567890ULL);
    }
    return complete;
}
int main() {
    int visible_count = 0;
    for (int angle = 0; angle < 360; ++angle) {
        if (lidar_angle_visible(angle)) ++visible_count;
    }
    assert(visible_count == 210);
    assert(lidar_angle_visible(0) && lidar_angle_visible(104));
    assert(!lidar_angle_visible(105) && !lidar_angle_visible(254));
    assert(lidar_angle_visible(255) && lidar_angle_visible(359));
    Ldstl26Lidar driver(1, -1, 41);
    driver.Init();
    Scan scan;
    auto corrupt = packet(0); corrupt[46] ^= 1;
    assert(!feed(driver, corrupt, scan));
    for (int i = 0; i < 120; ++i) assert(!feed(driver, packet(i), scan));
    assert(feed(driver, packet(0), scan));
    assert(scan.count == 360);
    for (int angle = 0; angle < 360; ++angle) {
        assert(scan.points[angle].angle == angle);
        assert(scan.points[angle].distance == (angle == 180 ? 50 : 1000));
    }
    std::puts("PASS: 1440 samples, 360 bins, 210-degree visibility, CRC and nearest nonzero distance");
}
