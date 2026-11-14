#include "system_globals.h"
#include "board_config.h"
#include "ldstl26_lidar.h"
#include "lidar_visibility.h"
#include "network_config.h"
#include "msg/lidar_msg.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "LIDAR_TASK";
namespace board_cfg = leap_config::board;
namespace net_cfg = leap_config::network;

void lidar_task(void *p) {
    Ldstl26Lidar* lidar = new Ldstl26Lidar(
        net_cfg::kLidarUartPort,
        board_cfg::kLidarTxPin,
        board_cfg::kLidarRxPin,
        net_cfg::kLidarBaudRate
    );

    lidar->Init();

    static LidarPoint full_scan_data[MAX_LIDAR_POINTS_PER_SCAN];
    uint16_t scan_point_count = 0;
    LidarMsg lidar_msg;

    ESP_LOGI(TAG, "Lidar task started on UART%d, RX=%d",
             static_cast<int>(net_cfg::kLidarUartPort),
             static_cast<int>(board_cfg::kLidarRxPin));

    while (1) {
        if (lidar->Poll(full_scan_data, &scan_point_count)) {
            memset(&lidar_msg, 0, sizeof(lidar_msg));

            for (int i = 0; i < scan_point_count; i++) {
                int angle = (int)full_scan_data[i].angle;
                uint16_t dist = (uint16_t)full_scan_data[i].distance;
                // 旋转180° + 左右镜像修正
                angle = (180 - angle + 360) % 360;
                // 保留前方 210°：0°~104° 和 255°~359°。
                if (!lidar_angle_visible(angle))
                    continue;

                if (dist > 0) {
                    if (lidar_msg.distances[angle] == 0 ||
                        dist < lidar_msg.distances[angle]) {
                        lidar_msg.distances[angle] = dist;
                    }
                }
            }
            xQueueOverwrite(system_globals().queues.lidar_state, &lidar_msg);
            system_globals().flags.lidar_scan_sequence =
                system_globals().flags.lidar_scan_sequence + 1;
        }

        vTaskDelay(pdMS_TO_TICKS(net_cfg::kLidarTaskPeriodMs));
    }
}
