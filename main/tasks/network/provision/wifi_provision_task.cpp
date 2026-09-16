#include "provision_portal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void wifi_provision_task(void *pvParameters) {
    (void)pvParameters;

    provision_portal_start();
    vTaskDelete(NULL);
}
