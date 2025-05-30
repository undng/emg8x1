#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define WIFI_MODE_BUTTON_PIN    GPIO_NUM_0
#define BUTTON_PRESS_TIME_MS    3000
#define AP_MODE_IP             "192.168.4.1"
#define AP_MODE_SSID           "EMG8x_Setup"
#define AP_MODE_PASSWORD       "emg8x123"

#define MAX_SSID_LEN           32
#define MAX_PASSWORD_LEN       64
#define MAX_IP_LEN             16

typedef struct {
    char sta_ssid[MAX_SSID_LEN];
    char sta_password[MAX_PASSWORD_LEN];
    char ap_ssid[MAX_SSID_LEN];
    char ap_password[MAX_PASSWORD_LEN];
    bool ap_mode;
    uint8_t ap_channel;
    uint8_t ap_max_connections;
} emg8x_wifi_config_t;


esp_err_t wifi_manager_init(void);
esp_err_t wifi_manager_start(void);
bool wifi_manager_is_ap_mode(void);
esp_err_t wifi_manager_get_ip(char *ip_buffer, size_t buffer_size);
esp_err_t wifi_manager_switch_mode(void);


esp_err_t wifi_manager_set_sta_config(const char *ssid, const char *password);
esp_err_t wifi_manager_set_ap_config(const char *ssid, const char *password, uint8_t channel);
esp_err_t wifi_manager_get_config(emg8x_wifi_config_t *config);
esp_err_t wifi_manager_apply_config(void);
esp_err_t wifi_manager_save_config(void);
esp_err_t wifi_manager_load_config(void);
esp_err_t wifi_manager_reset_config(void);

esp_err_t wifi_manager_scan_networks(void);
esp_err_t wifi_manager_get_scan_results(char *buffer, size_t buffer_size);
esp_err_t wifi_manager_get_connection_info(char *buffer, size_t buffer_size);

void wifi_mode_button_task(void *pvParameter);

#endif // WIFI_MANAGER_H