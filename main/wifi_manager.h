#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// WiFi Manager Configuration Constants
#define WIFI_MODE_BUTTON_PIN    GPIO_NUM_0
#define BUTTON_PRESS_TIME_MS    3000
#define AP_MODE_IP             "192.168.4.1"
#define AP_MODE_SSID           "EMG8x_Setup"
#define AP_MODE_PASSWORD       "emg8x123"

// WiFi Manager Functions
esp_err_t wifi_manager_init(void);
esp_err_t wifi_manager_start(void);
bool wifi_manager_is_ap_mode(void);
esp_err_t wifi_manager_get_ip(char *ip_buffer, size_t buffer_size);
esp_err_t wifi_manager_switch_mode(void);

// Button monitoring task (will be started automatically)
void wifi_mode_button_task(void *pvParameter);

#endif 