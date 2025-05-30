#include "telnet_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

// ==============================================
// MODULE CONFIGURATION AND CONSTANTS
// ==============================================

static const char *TAG = "TELNET_CONSOLE";

// ==============================================
// MODULE STATE VARIABLES
// ==============================================

// Client connection management
static telnet_client_t g_telnet_clients[MAX_CLIENTS];
static int g_telnet_server_socket = -1;
static TaskHandle_t g_telnet_task_handle = NULL;
static bool g_console_running = false;

// Callback interface to main application
static const console_callbacks_t *g_callbacks = NULL;

// ==============================================
// FORWARD DECLARATIONS
// ==============================================

static void telnet_send_response(int client_socket, const char *response);
static void send_prompt(int socket);

// Basic commands
static esp_err_t cmd_help(int socket, int argc, char **argv);
static esp_err_t cmd_status(int socket, int argc, char **argv);
static esp_err_t cmd_info(int socket, int argc, char **argv);
static esp_err_t cmd_reboot(int socket, int argc, char **argv);

// ADC commands  
static esp_err_t cmd_set_channel(int socket, int argc, char **argv);
static esp_err_t cmd_read_reg(int socket, int argc, char **argv);
static esp_err_t cmd_write_reg(int socket, int argc, char **argv);
static esp_err_t cmd_reset_adc(int socket, int argc, char **argv);
static esp_err_t cmd_start_stop(int socket, int argc, char **argv);

static esp_err_t cmd_wifi_info(int socket, int argc, char **argv);
static esp_err_t cmd_wifi_switch(int socket, int argc, char **argv);
static esp_err_t cmd_wifi_set_sta(int socket, int argc, char **argv);
static esp_err_t cmd_wifi_set_ap(int socket, int argc, char **argv);
static esp_err_t cmd_wifi_config(int socket, int argc, char **argv);
static esp_err_t cmd_wifi_apply(int socket, int argc, char **argv);
static esp_err_t cmd_wifi_save(int socket, int argc, char **argv);
static esp_err_t cmd_wifi_reset(int socket, int argc, char **argv);
static esp_err_t cmd_wifi_scan(int socket, int argc, char **argv);
static esp_err_t cmd_wifi_scan_results(int socket, int argc, char **argv);
static esp_err_t cmd_wifi_status(int socket, int argc, char **argv);

// Utility functions
static int parse_command(char *cmd_line, char **argv, int max_args);
static void execute_command(int socket, char *cmd_line);
static void handle_client_data(telnet_client_t *client, char *data, int len);

// ==============================================
// COMMAND TABLE
// ==============================================

static const console_command_t telnet_commands[] = {
    // Basic system commands
    {"help", cmd_help, "Show available commands"},
    {"status", cmd_status, "Show device status"},
    {"info", cmd_info, "Show system information"},
    {"reboot", cmd_reboot, "Reboot the device"},
    
    // ADC commands
    {"setch", cmd_set_channel, "setch <ch> <gain> <mode> - Set channel parameters"},
    {"rreg", cmd_read_reg, "rreg <addr> - Read register"},
    {"wreg", cmd_write_reg, "wreg <addr> <value> - Write register"},
    {"reset", cmd_reset_adc, "Reset ADC"},
    {"start", cmd_start_stop, "start/stop - Start/stop data acquisition"},
    {"stop", cmd_start_stop, "start/stop - Start/stop data acquisition"},
    
    // WiFi commands (legacy)
    {"wifi", cmd_wifi_info, "Show WiFi information and modes"},
    {"wifi_switch", cmd_wifi_switch, "Switch WiFi mode (STA/AP)"},
    
    // WiFi commands (enhanced)
    {"wifi_set_sta", cmd_wifi_set_sta, "wifi_set_sta <ssid> <password> - Configure STA mode"},
    {"wifi_set_ap", cmd_wifi_set_ap, "wifi_set_ap <ssid> <password> [channel] - Configure AP mode"},
    {"wifi_config", cmd_wifi_config, "Show current WiFi configuration"},
    {"wifi_apply", cmd_wifi_apply, "Apply WiFi configuration without reboot"},
    {"wifi_save", cmd_wifi_save, "Save WiFi configuration to persistent storage"},
    {"wifi_reset", cmd_wifi_reset, "Reset WiFi configuration to defaults"},
    {"wifi_scan", cmd_wifi_scan, "Scan for available WiFi networks"},
    {"wifi_scan_results", cmd_wifi_scan_results, "Show WiFi scan results"},
    {"wifi_status", cmd_wifi_status, "Show detailed WiFi connection status"},
    
    // Special commands
    {"exit", NULL, "Disconnect from console"},
    {NULL, NULL, NULL}  // End of table marker
};

// ==============================================
// UTILITY FUNCTIONS
// ==============================================

static void telnet_send_response(int client_socket, const char *response)
{
    if (client_socket >= 0 && response != NULL) {
        int len = strlen(response);
        int sent = send(client_socket, response, len, 0);
        if (sent != len) {
            ESP_LOGW(TAG, "Failed to send complete response to client");
        }
    }
}

static void send_prompt(int socket)
{
    telnet_send_response(socket, "EMG8x> ");
}

static int parse_command(char *cmd_line, char **argv, int max_args)
{
    int argc = 0;
    char *token = strtok(cmd_line, " \r\n");
    
    while (token != NULL && argc < max_args) {
        argv[argc++] = token;
        token = strtok(NULL, " \r\n");
    }
    
    return argc;
}

// ==============================================
// BASIC COMMAND HANDLERS
// ==============================================

static esp_err_t cmd_help(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    snprintf(response, sizeof(response),
             "\r\n=== EMG8x Console Commands ===\r\n"
             "\r\nBasic Commands:\r\n");
    telnet_send_response(socket, response);
    
    for (int i = 0; telnet_commands[i].cmd != NULL; i++) {
        if (strncmp(telnet_commands[i].cmd, "wifi_", 5) != 0 && 
            strcmp(telnet_commands[i].cmd, "wifi") != 0) {
            snprintf(response, sizeof(response),
                     "  %-12s - %s\r\n", telnet_commands[i].cmd, telnet_commands[i].help);
            telnet_send_response(socket, response);
        }
    }
    
    telnet_send_response(socket, "\r\nWiFi Commands:\r\n");
    for (int i = 0; telnet_commands[i].cmd != NULL; i++) {
        if (strncmp(telnet_commands[i].cmd, "wifi", 4) == 0) {
            snprintf(response, sizeof(response),
                     "  %-16s - %s\r\n", telnet_commands[i].cmd, telnet_commands[i].help);
            telnet_send_response(socket, response);
        }
    }
    
    telnet_send_response(socket, 
             "\r\nUsage Examples:\r\n"
             "  wifi_set_sta MyWiFi password123\r\n"
             "  wifi_set_ap EMG8x_Device mypassword 6\r\n"
             "  wifi_apply\r\n"
             "  wifi_save\r\n"
             "\r\n");
    return ESP_OK;
}

static esp_err_t cmd_status(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (g_callbacks == NULL) {
        telnet_send_response(socket, "\r\nError: Console not properly initialized\r\n\r\n");
        return ESP_ERR_INVALID_STATE;
    }
    
    snprintf(response, sizeof(response),
             "\r\nEMG8x Status:\r\n"
             "=============\r\n"
             "Block Counter: %" PRIu32 "\r\n"
             "Sample Count: %d\r\n"
             "Queue Head: %d\r\n"
             "Queue Tail: %d\r\n"
             "Free Heap: %" PRIu32 " bytes\r\n"
             "Min Free Heap: %" PRIu32 " bytes\r\n"
             "\r\n",
             g_callbacks->get_block_counter ? g_callbacks->get_block_counter() : 0,
             g_callbacks->get_sample_count ? g_callbacks->get_sample_count() : 0,
             g_callbacks->get_queue_head ? g_callbacks->get_queue_head() : 0,
             g_callbacks->get_queue_tail ? g_callbacks->get_queue_tail() : 0,
             g_callbacks->get_free_heap_size ? g_callbacks->get_free_heap_size() : 0,
             g_callbacks->get_min_free_heap_size ? g_callbacks->get_min_free_heap_size() : 0);
    
    telnet_send_response(socket, response);
    return ESP_OK;
}

static esp_err_t cmd_info(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    const char *chip_model = "Unknown";
    switch (chip_info.model) {
        case CHIP_ESP32: chip_model = "ESP32"; break;
        case CHIP_ESP32S2: chip_model = "ESP32-S2"; break;
        case CHIP_ESP32S3: chip_model = "ESP32-S3"; break;
        case CHIP_ESP32C3: chip_model = "ESP32-C3"; break;
        default: chip_model = "Unknown"; break;
    }
    
    uint64_t uptime = 0;
    if (g_callbacks && g_callbacks->get_uptime_ms) {
        uptime = g_callbacks->get_uptime_ms();
    }
    
    snprintf(response, sizeof(response),
             "\r\nSystem Information:\r\n"
             "==================\r\n"
             "IDF Version: %s\r\n"
             "Chip Model: %s\r\n"
             "Cores: %d\r\n"
             "Free Heap: %" PRIu32 " bytes\r\n"
             "Min Free Heap: %" PRIu32 " bytes\r\n"
             "Uptime: %" PRIu64 " ms (%.1f hours)\r\n"
#if CONFIG_EMG8X_BOARD_EMULATION == 0
             "Mode: Hardware\r\n"
#else
             "Mode: Emulation\r\n"
#endif
             "\r\n",
             esp_get_idf_version(),
             chip_model,
             chip_info.cores,
             g_callbacks && g_callbacks->get_free_heap_size ? g_callbacks->get_free_heap_size() : 0,
             g_callbacks && g_callbacks->get_min_free_heap_size ? g_callbacks->get_min_free_heap_size() : 0,
             uptime,
             uptime / 3600000.0);
    
    telnet_send_response(socket, response);
    return ESP_OK;
}

static esp_err_t cmd_reboot(int socket, int argc, char **argv)
{
    telnet_send_response(socket, "\r\nRebooting device in 3 seconds...\r\n\r\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    esp_restart();
    return ESP_OK;
}

// ==============================================
// ADC COMMAND HANDLERS
// ==============================================

static esp_err_t cmd_set_channel(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (argc != 4) {
        snprintf(response, sizeof(response),
                 "\r\nUsage: setch <channel> <gain> <mode>\r\n"
                 "channel: 1-8\r\n"
                 "gain: 0=x1, 1=x2, 2=x4, 3=x6, 4=x8, 5=x12, 6=x24\r\n"
                 "mode: 0=normal, 1=short, 3=test, 4=rld_drp, 5=rld_drn, 6=rld_drl, 7=mvdd\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_INVALID_ARG;
    }
    
    int channel = atoi(argv[1]);
    int gain = atoi(argv[2]);
    int mode = atoi(argv[3]);
    
    if (channel < 1 || channel > 8 || gain < 0 || gain > 6 || mode < 0 || mode > 7) {
        snprintf(response, sizeof(response), "\r\nInvalid parameters\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_callbacks == NULL || g_callbacks->adc_set_channel == NULL) {
        snprintf(response, sizeof(response), "\r\nError: ADC control not available\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    esp_err_t ret = g_callbacks->adc_set_channel(channel, gain, mode);
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response),
                 "\r\nChannel %d configured: gain=%d, mode=%d\r\n\r\n",
                 channel, gain, mode);
    } else {
        snprintf(response, sizeof(response),
                 "\r\nFailed to configure channel %d\r\n\r\n", channel);
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_read_reg(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (argc != 2) {
        snprintf(response, sizeof(response), "\r\nUsage: rreg <addr>\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_INVALID_ARG;
    }
    
    int addr = strtol(argv[1], NULL, 16);
    if (addr < 0 || addr > 0xFF) {
        snprintf(response, sizeof(response), "\r\nInvalid address\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_callbacks == NULL || g_callbacks->adc_read_reg == NULL) {
        snprintf(response, sizeof(response), "\r\nError: ADC control not available\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    uint8_t value = 0;
    esp_err_t ret = g_callbacks->adc_read_reg((uint8_t)addr, &value);
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response),
                 "\r\nRegister 0x%02X = 0x%02X\r\n\r\n", addr, value);
    } else {
        snprintf(response, sizeof(response),
                 "\r\nFailed to read register 0x%02X\r\n\r\n", addr);
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_write_reg(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (argc != 3) {
        snprintf(response, sizeof(response), "\r\nUsage: wreg <addr> <value>\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_INVALID_ARG;
    }
    
    int addr = strtol(argv[1], NULL, 16);
    int value = strtol(argv[2], NULL, 16);
    
    if (addr < 0 || addr > 0xFF || value < 0 || value > 0xFF) {
        snprintf(response, sizeof(response), "\r\nInvalid parameters\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_callbacks == NULL || g_callbacks->adc_write_reg == NULL) {
        snprintf(response, sizeof(response), "\r\nError: ADC control not available\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    esp_err_t ret = g_callbacks->adc_write_reg((uint8_t)addr, (uint8_t)value);
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response),
                 "\r\nRegister 0x%02X = 0x%02X written\r\n\r\n", addr, value);
    } else {
        snprintf(response, sizeof(response),
                 "\r\nFailed to write register 0x%02X\r\n\r\n", addr);
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_reset_adc(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (g_callbacks == NULL || g_callbacks->adc_reset == NULL) {
        snprintf(response, sizeof(response), "\r\nError: ADC control not available\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    esp_err_t ret = g_callbacks->adc_reset();
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response), "\r\nADC reset completed\r\n\r\n");
    } else {
        snprintf(response, sizeof(response), "\r\nADC reset failed\r\n\r\n");
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_start_stop(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (g_callbacks == NULL || g_callbacks->adc_start_stop == NULL) {
        snprintf(response, sizeof(response), "\r\nError: ADC control not available\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    bool start = (strcmp(argv[0], "start") == 0);
    esp_err_t ret = g_callbacks->adc_start_stop(start);
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response), 
                 "\r\nData acquisition %s\r\n\r\n", start ? "started" : "stopped");
    } else {
        snprintf(response, sizeof(response),
                 "\r\nFailed to %s data acquisition\r\n\r\n", start ? "start" : "stop");
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_wifi_info(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    char current_ip[16] = "N/A";
    
    if (g_callbacks && g_callbacks->wifi_get_ip) {
        g_callbacks->wifi_get_ip(current_ip, sizeof(current_ip));
    }
    
    bool is_ap_mode = false;
    if (g_callbacks && g_callbacks->wifi_is_ap_mode) {
        is_ap_mode = g_callbacks->wifi_is_ap_mode();
    }
    
    snprintf(response, sizeof(response),
             "\r\nWiFi Information:\r\n"
             "=================\r\n"
             "Current Mode: %s\r\n"
             "IP Address: %s\r\n"
             "\r\n"
             "Quick Commands:\r\n"
             "- wifi_config     : Show current configuration\r\n"
             "- wifi_scan       : Scan for networks\r\n"
             "- wifi_status     : Detailed connection status\r\n"
             "- wifi_set_sta <ssid> <pass> : Configure STA mode\r\n"
             "- wifi_set_ap <ssid> <pass>  : Configure AP mode\r\n"
             "- wifi_apply      : Apply config without reboot\r\n"
             "- wifi_save       : Save config permanently\r\n"
             "\r\n",
             is_ap_mode ? "AP (Access Point)" : "STA (Client)",
             current_ip);
    
    telnet_send_response(socket, response);
    return ESP_OK;
}

static esp_err_t cmd_wifi_switch(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (g_callbacks == NULL || g_callbacks->wifi_switch_mode == NULL) {
        telnet_send_response(socket, "\r\nError: WiFi switch not available\r\n\r\n");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    bool is_ap_mode = g_callbacks->wifi_is_ap_mode ? g_callbacks->wifi_is_ap_mode() : false;
    
    snprintf(response, sizeof(response),
             "\r\nSwitching WiFi mode from %s to %s\r\n"
             "Device will reboot in 3 seconds...\r\n\r\n",
             is_ap_mode ? "AP" : "STA",
             is_ap_mode ? "STA" : "AP");
    
    telnet_send_response(socket, response);
    
    esp_err_t ret = g_callbacks->wifi_switch_mode();
    return ret;
}

static esp_err_t cmd_wifi_set_sta(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (argc < 3) {
        snprintf(response, sizeof(response),
                 "\r\nUsage: wifi_set_sta <ssid> <password>\r\n"
                 "Example: wifi_set_sta MyWiFi mypassword123\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_callbacks == NULL || g_callbacks->wifi_set_sta_config == NULL) {
        telnet_send_response(socket, "\r\nError: WiFi configuration not available\r\n\r\n");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    esp_err_t ret = g_callbacks->wifi_set_sta_config(argv[1], argv[2]);
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response),
                 "\r\nSTA configuration updated:\r\n"
                 "SSID: %s\r\n"
                 "Password: %s\r\n"
                 "\r\n"
                 "Use 'wifi_apply' to apply without reboot or 'wifi_save' to save permanently.\r\n\r\n",
                 argv[1], argv[2]);
    } else {
        snprintf(response, sizeof(response), "\r\nFailed to set STA configuration\r\n\r\n");
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_wifi_set_ap(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (argc < 3) {
        snprintf(response, sizeof(response),
                 "\r\nUsage: wifi_set_ap <ssid> <password> [channel]\r\n"
                 "Example: wifi_set_ap EMG8x_Device mypassword 6\r\n"
                 "Channel range: 1-13 (default: 1)\r\n\r\n");
        telnet_send_response(socket, response);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_callbacks == NULL || g_callbacks->wifi_set_ap_config == NULL) {
        telnet_send_response(socket, "\r\nError: WiFi configuration not available\r\n\r\n");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    uint8_t channel = 1;  // Default channel
    if (argc >= 4) {
        int ch = atoi(argv[3]);
        if (ch >= 1 && ch <= 13) {
            channel = (uint8_t)ch;
        } else {
            snprintf(response, sizeof(response), "\r\nInvalid channel: %d (must be 1-13)\r\n\r\n", ch);
            telnet_send_response(socket, response);
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    esp_err_t ret = g_callbacks->wifi_set_ap_config(argv[1], argv[2], channel);
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response),
                 "\r\nAP configuration updated:\r\n"
                 "SSID: %s\r\n"
                 "Password: %s\r\n"
                 "Channel: %d\r\n"
                 "\r\n"
                 "Use 'wifi_apply' to apply without reboot or 'wifi_save' to save permanently.\r\n\r\n",
                 argv[1], argv[2], channel);
    } else {
        snprintf(response, sizeof(response), "\r\nFailed to set AP configuration\r\n\r\n");
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_wifi_config(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (g_callbacks == NULL || g_callbacks->wifi_get_config == NULL) {
        telnet_send_response(socket, "\r\nError: WiFi configuration not available\r\n\r\n");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    emg8x_wifi_config_t config;
    esp_err_t ret = g_callbacks->wifi_get_config(&config);
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response),
                 "\r\nCurrent WiFi Configuration:\r\n"
                 "===========================\r\n"
                 "Active Mode: %s\r\n"
                 "\r\n"
                 "STA (Client) Configuration:\r\n"
                 "  SSID: %s\r\n"
                 "  Password: %s\r\n"
                 "\r\n"
                 "AP (Access Point) Configuration:\r\n"
                 "  SSID: %s\r\n"
                 "  Password: %s\r\n"
                 "  Channel: %d\r\n"
                 "  Max Connections: %d\r\n"
                 "\r\n",
                 config.ap_mode ? "AP (Access Point)" : "STA (Client)",
                 config.sta_ssid,
                 config.sta_password,
                 config.ap_ssid,
                 config.ap_password,
                 config.ap_channel,
                 config.ap_max_connections);
    } else {
        snprintf(response, sizeof(response), "\r\nFailed to get WiFi configuration\r\n\r\n");
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_wifi_apply(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (g_callbacks == NULL || g_callbacks->wifi_apply_config == NULL) {
        telnet_send_response(socket, "\r\nError: WiFi apply not available\r\n\r\n");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    telnet_send_response(socket, "\r\nApplying WiFi configuration...\r\n");
    
    esp_err_t ret = g_callbacks->wifi_apply_config();
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response), "\r\nWiFi configuration applied successfully\r\n\r\n");
    } else {
        snprintf(response, sizeof(response), "\r\nFailed to apply WiFi configuration\r\n\r\n");
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_wifi_save(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (g_callbacks == NULL || g_callbacks->wifi_save_config == NULL) {
        telnet_send_response(socket, "\r\nError: WiFi save not available\r\n\r\n");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    esp_err_t ret = g_callbacks->wifi_save_config();
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response), "\r\nWiFi configuration saved permanently\r\n\r\n");
    } else {
        snprintf(response, sizeof(response), "\r\nFailed to save WiFi configuration\r\n\r\n");
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_wifi_reset(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (g_callbacks == NULL || g_callbacks->wifi_reset_config == NULL) {
        telnet_send_response(socket, "\r\nError: WiFi reset not available\r\n\r\n");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    esp_err_t ret = g_callbacks->wifi_reset_config();
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response), 
                 "\r\nWiFi configuration reset to defaults\r\n"
                 "Use 'wifi_config' to see current settings\r\n\r\n");
    } else {
        snprintf(response, sizeof(response), "\r\nFailed to reset WiFi configuration\r\n\r\n");
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_wifi_scan(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (g_callbacks == NULL || g_callbacks->wifi_scan_networks == NULL) {
        telnet_send_response(socket, "\r\nError: WiFi scan not available\r\n\r\n");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    telnet_send_response(socket, "\r\nScanning for WiFi networks...\r\n");
    
    esp_err_t ret = g_callbacks->wifi_scan_networks();
    
    if (ret == ESP_OK) {
        snprintf(response, sizeof(response), 
                 "\r\nScan completed. Use 'wifi_scan_results' to see networks.\r\n\r\n");
    } else {
        snprintf(response, sizeof(response), "\r\nWiFi scan failed\r\n\r\n");
    }
    
    telnet_send_response(socket, response);
    return ret;
}

static esp_err_t cmd_wifi_scan_results(int socket, int argc, char **argv)
{
    if (g_callbacks == NULL || g_callbacks->wifi_get_scan_results == NULL) {
        telnet_send_response(socket, "\r\nError: WiFi scan results not available\r\n\r\n");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    char *scan_buffer = malloc(2048);  // Larger buffer for scan results
    if (scan_buffer == NULL) {
        telnet_send_response(socket, "\r\nError: Out of memory\r\n\r\n");
        return ESP_ERR_NO_MEM;
    }
    
    telnet_send_response(socket, "\r\nAvailable WiFi Networks:\r\n========================\r\n");
    
    esp_err_t ret = g_callbacks->wifi_get_scan_results(scan_buffer, 2048);
    
    if (ret == ESP_OK) {
        telnet_send_response(socket, scan_buffer);
        telnet_send_response(socket, "\r\n");
    } else {
        telnet_send_response(socket, "\r\nFailed to get scan results\r\n\r\n");
    }
    
    free(scan_buffer);
    return ret;
}

static esp_err_t cmd_wifi_status(int socket, int argc, char **argv)
{
    char response[RESPONSE_BUFFER_SIZE];
    
    if (g_callbacks == NULL || g_callbacks->wifi_get_connection_info == NULL) {
        telnet_send_response(socket, "\r\nError: WiFi status not available\r\n\r\n");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    telnet_send_response(socket, "\r\nWiFi Connection Status:\r\n=======================\r\n");
    
    esp_err_t ret = g_callbacks->wifi_get_connection_info(response, sizeof(response));
    
    if (ret == ESP_OK) {
        telnet_send_response(socket, response);
        telnet_send_response(socket, "\r\n");
    } else {
        telnet_send_response(socket, "\r\nFailed to get connection info\r\n\r\n");
    }
    
    return ret;
}

// ==============================================
// COMMAND PROCESSING
// ==============================================

static void execute_command(int socket, char *cmd_line)
{
    char *argv[16];
    int argc = parse_command(cmd_line, argv, 16);
    
    if (argc == 0) {
        send_prompt(socket);
        return;
    }
    
    // Special handling for exit command
    if (strcmp(argv[0], "exit") == 0) {
        telnet_send_response(socket, "\r\nGoodbye!\r\n");
        close(socket);
        return;
    }
    
    // Search for command in table
    for (int i = 0; telnet_commands[i].cmd != NULL; i++) {
        if (strcmp(argv[0], telnet_commands[i].cmd) == 0) {
            if (telnet_commands[i].handler != NULL) {
                esp_err_t result = telnet_commands[i].handler(socket, argc, argv);
                if (result != ESP_OK) {
                    ESP_LOGW(TAG, "Command '%s' returned error: %s", 
                             argv[0], esp_err_to_name(result));
                }
            }
            send_prompt(socket);
            return;
        }
    }
    
    // Unknown command
    char response[RESPONSE_BUFFER_SIZE];
    snprintf(response, sizeof(response),
             "\r\nUnknown command: %s\r\nType 'help' for available commands\r\n\r\n", 
             argv[0]);
    telnet_send_response(socket, response);
    send_prompt(socket);
}

static void handle_client_data(telnet_client_t *client, char *data, int len)
{
    for (int i = 0; i < len; i++) {
        char c = data[i];
        
        if (c == '\r' || c == '\n') {
            if (client->cmd_pos > 0) {
                client->cmd_buffer[client->cmd_pos] = '\0';
                telnet_send_response(client->socket, "\r\n");
                execute_command(client->socket, client->cmd_buffer);
                client->cmd_pos = 0;
            } else {
                send_prompt(client->socket);
            }
        } else if (c == '\b' || c == 127) { // Backspace
            if (client->cmd_pos > 0) {
                client->cmd_pos--;
                telnet_send_response(client->socket, "\b \b");
            }
        } else if (c == 3) { // Ctrl+C
            telnet_send_response(client->socket, "\r\n^C\r\n");
            client->cmd_pos = 0;
            send_prompt(client->socket);
        } else if (c == 27) { // ESC sequence (arrows, etc.)
            // Ignore escape sequences
            continue;
        } else if (c >= 32 && c < 127) { // Printable characters
            if (client->cmd_pos < CMD_BUFFER_SIZE - 1) {
                client->cmd_buffer[client->cmd_pos++] = c;
                send(client->socket, &c, 1, 0);
            }
        }
        // Ignore all other characters (including telnet control sequences)
    }
}

// ==============================================
// MAIN TELNET SERVER TASK
// ==============================================

void telnet_console_task(void *pvParameter)
{
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    fd_set read_fds;
    int max_fd;
    
    ESP_LOGI(TAG, "Telnet console task started");
    
    // Create server socket
    g_telnet_server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (g_telnet_server_socket < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        g_console_running = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Set socket options
    int opt = 1;
    setsockopt(g_telnet_server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Bind to address
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(TELNET_PORT);
    
    if (bind(g_telnet_server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Bind failed");
        close(g_telnet_server_socket);
        g_console_running = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Start listening
    if (listen(g_telnet_server_socket, MAX_CLIENTS) < 0) {
        ESP_LOGE(TAG, "Listen failed");
        close(g_telnet_server_socket);
        g_console_running = false;
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Telnet console listening on port %d", TELNET_PORT);
    g_console_running = true;
    
    // Main server loop
    while (g_console_running) {
        FD_ZERO(&read_fds);
        FD_SET(g_telnet_server_socket, &read_fds);
        max_fd = g_telnet_server_socket;
        
        // Add active client sockets to select set
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (g_telnet_clients[i].active && g_telnet_clients[i].socket >= 0) {
                FD_SET(g_telnet_clients[i].socket, &read_fds);
                if (g_telnet_clients[i].socket > max_fd) {
                    max_fd = g_telnet_clients[i].socket;
                }
            }
        }
        
        struct timeval timeout = {1, 0}; // 1 second timeout
        int activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        
        if (activity < 0) {
            ESP_LOGE(TAG, "Select error");
            continue;
        }
        
        // Handle new connections
        if (FD_ISSET(g_telnet_server_socket, &read_fds)) {
            int new_socket = accept(g_telnet_server_socket, 
                                   (struct sockaddr *)&client_addr, &client_addr_len);
            if (new_socket >= 0) {
                // Find free client slot
                int client_idx = -1;
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (!g_telnet_clients[i].active) {
                        client_idx = i;
                        break;
                    }
                }
                
                if (client_idx >= 0) {
                    g_telnet_clients[client_idx].socket = new_socket;
                    g_telnet_clients[client_idx].active = true;
                    g_telnet_clients[client_idx].cmd_pos = 0;
                    
                    ESP_LOGI(TAG, "New telnet client connected: %s", 
                             inet_ntoa(client_addr.sin_addr));
                    
                    // Send telnet negotiation to disable echo and line mode
                    char telnet_init[] = {255, 251, 1,  // IAC WILL ECHO
                                         255, 251, 3,  // IAC WILL SUPPRESS-GO-AHEAD
                                         255, 253, 1,  // IAC DO ECHO
                                         255, 253, 3}; // IAC DO SUPPRESS-GO-AHEAD
                    send(new_socket, telnet_init, sizeof(telnet_init), 0);
                    
                    telnet_send_response(new_socket,
                                       "\r\n=== Welcome to EMG8x Console! ===\r\n"
                                       "Type 'help' for available commands.\r\n"
                                       "Type 'wifi' for WiFi quick commands.\r\n");
                    send_prompt(new_socket);
                } else {
                    telnet_send_response(new_socket,
                                       "\r\nSorry, maximum clients connected.\r\n");
                    close(new_socket);
                }
            }
        }
        
        // Handle client data
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (g_telnet_clients[i].active && 
                FD_ISSET(g_telnet_clients[i].socket, &read_fds)) {
                
                char buffer[256];
                int bytes_read = recv(g_telnet_clients[i].socket, buffer, 
                                    sizeof(buffer) - 1, 0);
                
                if (bytes_read <= 0) {
                    ESP_LOGI(TAG, "Client disconnected");
                    close(g_telnet_clients[i].socket);
                    g_telnet_clients[i].active = false;
                    g_telnet_clients[i].socket = -1;
                } else {
                    buffer[bytes_read] = '\0';
                    handle_client_data(&g_telnet_clients[i], buffer, bytes_read);
                }
            }
        }
    }
    
    // Cleanup
    close(g_telnet_server_socket);
    g_telnet_server_socket = -1;
    ESP_LOGI(TAG, "Telnet console task stopped");
    vTaskDelete(NULL);
}

// ==============================================
// PUBLIC API IMPLEMENTATION
// ==============================================

esp_err_t telnet_console_init(const console_callbacks_t *callbacks)
{
    if (callbacks == NULL) {
        ESP_LOGE(TAG, "Callbacks cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    g_callbacks = callbacks;
    
    // Initialize client array
    memset(g_telnet_clients, 0, sizeof(g_telnet_clients));
    for (int i = 0; i < MAX_CLIENTS; i++) {
        g_telnet_clients[i].socket = -1;
        g_telnet_clients[i].active = false;
    }
    
    g_console_running = false;
    g_telnet_server_socket = -1;
    g_telnet_task_handle = NULL;
    
    ESP_LOGI(TAG, "Telnet console initialized with enhanced WiFi commands");
    return ESP_OK;
}

esp_err_t telnet_console_start(void)
{
    if (g_callbacks == NULL) {
        ESP_LOGE(TAG, "Console not initialized - call telnet_console_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_console_running) {
        ESP_LOGW(TAG, "Console already running");
        return ESP_OK;
    }
    
    // Create console task
    BaseType_t result = xTaskCreatePinnedToCore(
        &telnet_console_task,
        "telnet_console_task",
        6144,  // Increased stack size for enhanced features
        NULL,
        tskIDLE_PRIORITY + 3,
        &g_telnet_task_handle,
        0  // Pin to core 0
    );
    
    if (result != pdPASS || g_telnet_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create telnet console task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Enhanced telnet console started successfully");
    return ESP_OK;
}

esp_err_t telnet_console_stop(void)
{
    if (!g_console_running) {
        ESP_LOGW(TAG, "Console not running");
        return ESP_OK;
    }
    
    g_console_running = false;
    
    // Close all client connections
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (g_telnet_clients[i].active) {
            close(g_telnet_clients[i].socket);
            g_telnet_clients[i].active = false;
            g_telnet_clients[i].socket = -1;
        }
    }
    
    // Task will cleanup and delete itself
    if (g_telnet_task_handle != NULL) {
        // Wait a bit for task to cleanup
        vTaskDelay(100 / portTICK_PERIOD_MS);
        g_telnet_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Telnet console stopped");
    return ESP_OK;
}

bool telnet_console_is_running(void)
{
    return g_console_running;
}

int telnet_console_get_client_count(void)
{
    int count = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (g_telnet_clients[i].active) {
            count++;
        }
    }
    return count;
}