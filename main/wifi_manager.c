#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"              // For MAC address functions
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "lwip/inet.h"
#include <string.h>

// Define MAC formatting macros if not available
#ifndef MACSTR
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#endif

// Module-specific logging tag
static const char *TAG = "WIFI_MANAGER";

// Private module state
static bool g_wifi_ap_mode = false;
static bool g_wifi_initialized = false;
static bool g_button_was_pressed = false;
static int g_button_press_start_time = 0;
static TaskHandle_t g_wifi_mode_task_handle = NULL;
static char g_current_ip[16] = {0};

// Event group for WiFi connection (STA mode)
static EventGroupHandle_t g_wifi_event_group = NULL;
static int g_retry_count = 0;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define MAX_RETRY_COUNT 10

// Forward declarations of private functions
static void save_wifi_mode(bool ap_mode);
static bool load_wifi_mode(void);
static void indicate_wifi_mode(bool ap_mode);
static void wifi_init_ap_mode(void);
static void wifi_init_sta_mode(void);
static void wifi_event_handler_ap(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void wifi_event_handler_sta(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

static emg8x_wifi_config_t g_wifi_config = {
    .sta_ssid = CONFIG_WIFI_SSID,
    .sta_password = CONFIG_WIFI_PASSWORD,
    .ap_ssid = AP_MODE_SSID,
    .ap_password = AP_MODE_PASSWORD,
    .ap_mode = false,
    .ap_channel = 1,
    .ap_max_connections = 4
};



esp_err_t wifi_manager_init(void) {
    ESP_LOGI(TAG, "Initializing WiFi Manager");
    
    // Initialize button GPIO
    gpio_reset_pin(WIFI_MODE_BUTTON_PIN);
    gpio_set_direction(WIFI_MODE_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(WIFI_MODE_BUTTON_PIN, GPIO_PULLUP_ONLY);
    
    // Load saved WiFi mode from NVS
    g_wifi_ap_mode = load_wifi_mode();
    
    ESP_LOGI(TAG, "WiFi mode: %s", g_wifi_ap_mode ? "AP (Access Point)" : "STA (Client)");
    
    return ESP_OK;
}

esp_err_t wifi_manager_start(void) {
    if (g_wifi_initialized) {
        ESP_LOGW(TAG, "WiFi already initialized");
        return ESP_OK;
    }
    
    // Show current mode with LED indication
    indicate_wifi_mode(g_wifi_ap_mode);
    
    // Start appropriate WiFi mode
    if (g_wifi_ap_mode) {
        wifi_init_ap_mode();
    } else {
        wifi_init_sta_mode();
    }
    
    // Start button monitoring task
    xTaskCreatePinnedToCore(
        &wifi_mode_button_task,
        "wifi_mode_button_task",
        2048,
        NULL,
        tskIDLE_PRIORITY + 2,
        &g_wifi_mode_task_handle,
        0
    );
    
    if (g_wifi_mode_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi button task");
        return ESP_FAIL;
    }
    
    g_wifi_initialized = true;
    ESP_LOGI(TAG, "WiFi Manager started successfully");
    return ESP_OK;
}

bool wifi_manager_is_ap_mode(void) {
    return g_wifi_ap_mode;
}

esp_err_t wifi_manager_get_ip(char *ip_buffer, size_t buffer_size) {
    if (ip_buffer == NULL || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    strncpy(ip_buffer, g_current_ip, buffer_size - 1);
    ip_buffer[buffer_size - 1] = '\0';
    return ESP_OK;
}

esp_err_t wifi_manager_switch_mode(void) {
    ESP_LOGI(TAG, "Switching WiFi mode");
    
    g_wifi_ap_mode = !g_wifi_ap_mode;
    save_wifi_mode(g_wifi_ap_mode);
    
    indicate_wifi_mode(g_wifi_ap_mode);
    
    ESP_LOGI(TAG, "WiFi mode will be: %s after reboot", 
             g_wifi_ap_mode ? "AP (Access Point)" : "STA (Client)");
    
    // Restart to apply new mode
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_restart();
    
    return ESP_OK;
}

// ============================================
// BUTTON MONITORING TASK
// ============================================

void wifi_mode_button_task(void *pvParameter) {
    ESP_LOGI(TAG, "WiFi mode button task started");
    
    while (1) {
        // Check button state (active LOW)
        if (gpio_get_level(WIFI_MODE_BUTTON_PIN) == 0) {
            // Button pressed
            if (!g_button_was_pressed) {
                g_button_was_pressed = true;
                g_button_press_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                ESP_LOGI(TAG, "Button pressed, waiting for %d ms...", BUTTON_PRESS_TIME_MS);
            } else {
                // Check hold duration
                int current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if ((current_time - g_button_press_start_time) >= BUTTON_PRESS_TIME_MS) {
                    ESP_LOGI(TAG, "Button held for %d ms - switching WiFi mode", BUTTON_PRESS_TIME_MS);
                    wifi_manager_switch_mode();
                }
            }
        } else {
            // Button released
            if (g_button_was_pressed) {
                int press_duration = xTaskGetTickCount() * portTICK_PERIOD_MS - g_button_press_start_time;
                if (press_duration < BUTTON_PRESS_TIME_MS) {
                    ESP_LOGI(TAG, "Button released after %d ms (too short)", press_duration);
                }
                g_button_was_pressed = false;
            }
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// ============================================
// PRIVATE FUNCTIONS
// ============================================

static void save_wifi_mode(bool ap_mode) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("emg8x", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_set_u8(nvs_handle, "wifi_ap_mode", ap_mode ? 1 : 0);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "WiFi mode saved: %s", ap_mode ? "AP" : "STA");
    } else {
        ESP_LOGE(TAG, "Failed to save WiFi mode: %s", esp_err_to_name(err));
    }
}

static bool load_wifi_mode(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("emg8x", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        uint8_t mode = 0;
        err = nvs_get_u8(nvs_handle, "wifi_ap_mode", &mode);
        nvs_close(nvs_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "WiFi mode loaded: %s", mode ? "AP" : "STA");
            return mode != 0;
        }
    }
    ESP_LOGI(TAG, "WiFi mode not found, using default: STA");
    return false; // Default to STA mode
}

static void indicate_wifi_mode(bool ap_mode) {
    gpio_num_t led_pin = GPIO_NUM_2;  // Built-in LED
    
    // Configure LED pin
    gpio_reset_pin(led_pin);
    gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);
    
    // AP mode: 3 fast blinks, STA mode: 1 slow blink
    int blink_count = ap_mode ? 3 : 1;
    int blink_delay = ap_mode ? 200 : 500;
    
    for (int i = 0; i < blink_count; i++) {
        gpio_set_level(led_pin, 1);
        vTaskDelay(blink_delay / portTICK_PERIOD_MS);
        gpio_set_level(led_pin, 0);
        vTaskDelay(blink_delay / portTICK_PERIOD_MS);
    }
}

static void wifi_init_ap_mode(void) {
    ESP_LOGI(TAG, "Initializing WiFi AP mode");
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    
    // Configure AP IP address
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);
    
    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register event handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_ap, NULL, NULL));
    
    // Configure AP
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_MODE_SSID,
            .ssid_len = strlen(AP_MODE_SSID),
            .channel = 1,
            .password = AP_MODE_PASSWORD,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Store IP address
    strncpy(g_current_ip, AP_MODE_IP, sizeof(g_current_ip) - 1);
    
    ESP_LOGI(TAG, "WiFi AP Mode started:");
    ESP_LOGI(TAG, "  SSID: %s", AP_MODE_SSID);
    ESP_LOGI(TAG, "  Password: %s", AP_MODE_PASSWORD);
    ESP_LOGI(TAG, "  IP: %s", AP_MODE_IP);
}

static void wifi_init_sta_mode(void) {
    ESP_LOGI(TAG, "Initializing WiFi STA mode");
    
    g_wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_sta, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler_sta, NULL, &instance_got_ip));
    
    // Configure STA
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi STA initialization finished");
    
    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(g_wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE, pdFALSE, portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID: %s", CONFIG_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID: %s", CONFIG_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Unexpected WiFi event");
    }
    
    // Cleanup
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(g_wifi_event_group);
}

static void wifi_event_handler_ap(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station connected: " MACSTR " AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station disconnected: " MACSTR " AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void wifi_event_handler_sta(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (g_retry_count < MAX_RETRY_COUNT) {
            esp_wifi_connect();
            g_retry_count++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(g_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t*) event_data;
        snprintf(g_current_ip, sizeof(g_current_ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Got IP: %s", g_current_ip);
        g_retry_count = 0;
        xEventGroupSetBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}



esp_err_t wifi_manager_set_sta_config(const char *ssid, const char *password) {
    if (ssid == NULL || password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(ssid) >= MAX_SSID_LEN || strlen(password) >= MAX_PASSWORD_LEN) {
        ESP_LOGE(TAG, "SSID or password too long");
        return ESP_ERR_INVALID_SIZE;
    }
    
    strncpy(g_wifi_config.sta_ssid, ssid, MAX_SSID_LEN - 1);
    g_wifi_config.sta_ssid[MAX_SSID_LEN - 1] = '\0';
    
    strncpy(g_wifi_config.sta_password, password, MAX_PASSWORD_LEN - 1);
    g_wifi_config.sta_password[MAX_PASSWORD_LEN - 1] = '\0';
    
    ESP_LOGI(TAG, "STA config updated: SSID=%s", ssid);
    return ESP_OK;
}

esp_err_t wifi_manager_set_ap_config(const char *ssid, const char *password, uint8_t channel) {
    if (ssid == NULL || password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(ssid) >= MAX_SSID_LEN || strlen(password) >= MAX_PASSWORD_LEN) {
        ESP_LOGE(TAG, "SSID or password too long");
        return ESP_ERR_INVALID_SIZE;
    }
    
    if (channel < 1 || channel > 13) {
        ESP_LOGE(TAG, "Invalid channel: %d (must be 1-13)", channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    strncpy(g_wifi_config.ap_ssid, ssid, MAX_SSID_LEN - 1);
    g_wifi_config.ap_ssid[MAX_SSID_LEN - 1] = '\0';
    
    strncpy(g_wifi_config.ap_password, password, MAX_PASSWORD_LEN - 1);
    g_wifi_config.ap_password[MAX_PASSWORD_LEN - 1] = '\0';
    
    g_wifi_config.ap_channel = channel;
    
    ESP_LOGI(TAG, "AP config updated: SSID=%s, Channel=%d", ssid, channel);
    return ESP_OK;
}

esp_err_t wifi_manager_get_config(emg8x_wifi_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(config, &g_wifi_config, sizeof(emg8x_wifi_config_t));
    return ESP_OK;
}

esp_err_t wifi_manager_apply_config(void) {
    ESP_LOGI(TAG, "wifi_manager_apply_config: Not yet implemented (stub)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t wifi_manager_save_config(void) {
    ESP_LOGI(TAG, "wifi_manager_save_config: Not yet implemented (stub)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t wifi_manager_load_config(void) {
    ESP_LOGI(TAG, "wifi_manager_load_config: Not yet implemented (stub)");
    return ESP_OK;
}

esp_err_t wifi_manager_reset_config(void) {
    ESP_LOGI(TAG, "wifi_manager_reset_config: Not yet implemented (stub)");
    
    // Reset to default values
    strncpy(g_wifi_config.sta_ssid, CONFIG_WIFI_SSID, MAX_SSID_LEN - 1);
    strncpy(g_wifi_config.sta_password, CONFIG_WIFI_PASSWORD, MAX_PASSWORD_LEN - 1);
    strncpy(g_wifi_config.ap_ssid, AP_MODE_SSID, MAX_SSID_LEN - 1);
    strncpy(g_wifi_config.ap_password, AP_MODE_PASSWORD, MAX_PASSWORD_LEN - 1);
    g_wifi_config.ap_mode = false;
    g_wifi_config.ap_channel = 1;
    g_wifi_config.ap_max_connections = 4;
    
    return ESP_OK;
}

esp_err_t wifi_manager_scan_networks(void) {
    ESP_LOGI(TAG, "wifi_manager_scan_networks: Not yet implemented (stub)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t wifi_manager_get_scan_results(char *buffer, size_t buffer_size) {
    if (buffer == NULL || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    strncpy(buffer, "No scan results available (stub implementation)\n"
                   "Use the enhanced wifi_manager.c for full functionality.\n", 
            buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
    
    return ESP_OK;
}

esp_err_t wifi_manager_get_connection_info(char *buffer, size_t buffer_size) {
    if (buffer == NULL || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    char current_ip[16];
    wifi_manager_get_ip(current_ip, sizeof(current_ip));
    
    snprintf(buffer, buffer_size,
             "Mode: %s\r\n"
             "SSID: %s\r\n"
             "IP: %s\r\n"
             "Status: Stub implementation\r\n",
             g_wifi_config.ap_mode ? "AP (Access Point)" : "STA (Client)",
             g_wifi_config.ap_mode ? g_wifi_config.ap_ssid : g_wifi_config.sta_ssid,
             current_ip);
             
    return ESP_OK;
}