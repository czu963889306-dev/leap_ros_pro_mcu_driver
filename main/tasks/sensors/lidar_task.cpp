#include "system_globals.h"
#include "ldstl26_lidar.h"
#include "msg/lidar_msg.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "LIDAR_TASK";

void lidar_task(void *p) {
    Ldstl26Lidar* lidar = new Ldstl26Lidar(
        UART_NUM_1,
        (gpio_num_t)UART_PIN_NO_CHANGE,
        GPIO_NUM_41,
        230400
    );

    lidar->Init();

    static LidarPoint full_scan_data[MAX_LIDAR_POINTS_PER_SCAN];
    uint16_t scan_point_count = 0;
    LidarMsg lidar_msg;

    ESP_LOGI(TAG, "Lidar task started on UART1, RX=41");

    while (1) {
        if (lidar->Poll(full_scan_data, &scan_point_count)) {
            memset(&lidar_msg, 0, sizeof(lidar_msg));

            for (int i = 0; i < scan_point_count; i++) {
                int angle = (int)full_scan_data[i].angle;
                uint16_t dist = (uint16_t)full_scan_data[i].distance;
                // 旋转180° + 左右镜像修正
                angle = (180 - angle + 360) % 360;
                if (angle >= 75 && angle < 285)
                    continue;

                if (dist > 0) {
                    if (lidar_msg.distances[angle] == 0 ||
                        dist < lidar_msg.distances[angle]) {
                        lidar_msg.distances[angle] = dist;
                    }
                }
            }
            xQueueOverwrite(q_lidar_state, &lidar_msg);
            g_lidar_scan_sequence++;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}