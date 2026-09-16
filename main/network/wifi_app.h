#ifndef WIFI_APP_H_
#define WIFI_APP_H_

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "system_globals.h"

static constexpr size_t kMicroRosAgentIpMaxLen = 16;

struct WifiRuntimeConfig {
    WifiCommMode comm_mode;
    ChassisType chassis_type;
    char microros_agent_ip[kMicroRosAgentIpMaxLen];
    uint16_t microros_agent_port;
};

class WifiApp {
 public:
    static WifiApp& Instance();

    bool InitSta();
    void InitSoftAp();
    void InitEspNow();

    esp_err_t SaveStaCredentials(const char *ssid, const char *password);
    esp_err_t ClearStaCredentials();
    esp_err_t LoadStaCredentialsOrDefault(
        char *ssid,
        size_t ssid_size,
        char *password,
        size_t password_size);
    esp_err_t GetSoftApIpInfo(esp_netif_ip_info_t *ip_info) const;

    void GetDefaultRuntimeConfig(WifiRuntimeConfig *config) const;
    esp_err_t LoadRuntimeConfig(WifiRuntimeConfig *config);
    esp_err_t SaveRuntimeConfig(const WifiRuntimeConfig *config);
    esp_err_t SaveCommMode(WifiCommMode mode);

 private:
    WifiApp() = default;

    esp_err_t LoadSavedStaCredentials(
        char *ssid,
        size_t ssid_size,
        char *password,
        size_t password_size);
    void FillStaConfig(wifi_config_t *wifi_config, const char *ssid, const char *password) const;
    bool IsValidRuntimeConfig(const WifiRuntimeConfig *config) const;
    void EnsureWifiStackReady();
    void HandleWifiEvent(esp_event_base_t event_base, int32_t event_id, void *event_data);

    static void StaticWifiEventHandler(
        void *arg,
        esp_event_base_t event_base,
        int32_t event_id,
        void *event_data);

    EventGroupHandle_t wifi_event_group_ = nullptr;
    esp_netif_t *sta_netif_ = nullptr;
    esp_netif_t *ap_netif_ = nullptr;
    bool netif_initialized_ = false;
    bool event_loop_initialized_ = false;
    bool event_handlers_registered_ = false;
    bool wifi_driver_initialized_ = false;
    bool sta_auto_connect_enabled_ = false;
};

WifiApp& wifi_app();

#endif
