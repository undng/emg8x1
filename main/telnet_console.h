#ifndef TELNET_CONSOLE_H
#define TELNET_CONSOLE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==============================================
// TELNET CONSOLE CONFIGURATION
// ==============================================

#define TELNET_PORT                     23
#define MAX_CLIENTS                     2
#define CMD_BUFFER_SIZE                 256
#define RESPONSE_BUFFER_SIZE            512

// ==============================================
// DATA STRUCTURES
// ==============================================

/**
 * @brief Telnet client connection state
 */
typedef struct {
    int socket;                         ///< Client socket descriptor
    bool active;                        ///< Connection active flag
    char cmd_buffer[CMD_BUFFER_SIZE];   ///< Command input buffer
    int cmd_pos;                        ///< Current position in command buffer
} telnet_client_t;

/**
 * @brief Console command handler function type
 * @param socket Client socket for sending responses
 * @param argc Number of command arguments
 * @param argv Array of command arguments
 * @return ESP_OK on success, error code on failure
 */
typedef esp_err_t (*console_cmd_handler_t)(int socket, int argc, char **argv);

/**
 * @brief Console command definition
 */
typedef struct {
    const char *cmd;                    ///< Command name
    console_cmd_handler_t handler;      ///< Command handler function
    const char *help;                   ///< Help text for the command
} console_command_t;

// ==============================================
// CALLBACK INTERFACE FOR MAIN APPLICATION
// ==============================================

/**
 * @brief Callback functions for accessing main application data and functionality
 * 
 * This structure provides a clean interface between the console and the main
 * application, allowing the console to be independent and reusable.
 */
typedef struct {
    // ADC/Hardware operations
    /**
     * @brief Read ADC register
     * @param addr Register address
     * @param value Pointer to store read value
     * @return ESP_OK on success
     */
    esp_err_t (*adc_read_reg)(uint8_t addr, uint8_t *value);
    
    /**
     * @brief Write ADC register
     * @param addr Register address
     * @param value Value to write
     * @return ESP_OK on success
     */
    esp_err_t (*adc_write_reg)(uint8_t addr, uint8_t value);
    
    /**
     * @brief Reset ADC
     * @return ESP_OK on success
     */
    esp_err_t (*adc_reset)(void);
    
    /**
     * @brief Start/stop data acquisition
     * @param start true to start, false to stop
     * @return ESP_OK on success
     */
    esp_err_t (*adc_start_stop)(bool start);
    
    /**
     * @brief Configure ADC channel
     * @param channel Channel number (1-8)
     * @param gain Gain setting (0-6)
     * @param mode Channel mode (0-7)
     * @return ESP_OK on success
     */
    esp_err_t (*adc_set_channel)(int channel, int gain, int mode);
    
    // System status information
    /**
     * @brief Get current block counter
     * @return Block counter value
     */
    uint32_t (*get_block_counter)(void);
    
    /**
     * @brief Get current sample count
     * @return Sample count value
     */
    int (*get_sample_count)(void);
    
    /**
     * @brief Get queue head position
     * @return Queue head index
     */
    int (*get_queue_head)(void);
    
    /**
     * @brief Get queue tail position
     * @return Queue tail index
     */
    int (*get_queue_tail)(void);
    
    // WiFi operations
    /**
     * @brief Check if WiFi is in AP mode
     * @return true if in AP mode, false if in STA mode
     */
    bool (*wifi_is_ap_mode)(void);
    
    /**
     * @brief Get current IP address
     * @param buffer Buffer to store IP address string
     * @param size Buffer size
     * @return ESP_OK on success
     */
    esp_err_t (*wifi_get_ip)(char *buffer, size_t size);
    
    /**
     * @brief Switch WiFi mode (STA <-> AP)
     * @return ESP_OK on success
     */
    esp_err_t (*wifi_switch_mode)(void);
    
    // System operations
    /**
     * @brief Get free heap size
     * @return Free heap size in bytes
     */
    uint32_t (*get_free_heap_size)(void);
    
    /**
     * @brief Get minimum free heap size
     * @return Minimum free heap size in bytes
     */
    uint32_t (*get_min_free_heap_size)(void);
    
    /**
     * @brief Get system uptime
     * @return Uptime in milliseconds
     */
    uint64_t (*get_uptime_ms)(void);
    
} console_callbacks_t;

// ==============================================
// PUBLIC API
// ==============================================

/**
 * @brief Initialize telnet console
 * 
 * This function initializes the telnet console subsystem and registers
 * callback functions for accessing main application functionality.
 * 
 * @param callbacks Pointer to callback structure (must remain valid)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t telnet_console_init(const console_callbacks_t *callbacks);

/**
 * @brief Start telnet console server
 * 
 * Creates and starts the telnet server task. The console will begin
 * accepting connections on the configured port.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t telnet_console_start(void);

/**
 * @brief Stop telnet console server
 * 
 * Stops the telnet server task and closes all active connections.
 * 
 * @return ESP_OK on success, error code on failure
 */
esp_err_t telnet_console_stop(void);

/**
 * @brief Check if telnet console is running
 * 
 * @return true if console is active, false otherwise
 */
bool telnet_console_is_running(void);

/**
 * @brief Get number of active telnet connections
 * 
 * @return Number of active client connections
 */
int telnet_console_get_client_count(void);

// ==============================================
// INTERNAL TASK FUNCTION (for FreeRTOS)
// ==============================================

/**
 * @brief Main telnet console task function
 * 
 * This is the main task function for the telnet console. It should not
 * be called directly - use telnet_console_start() instead.
 * 
 * @param pvParameter Task parameter (unused)
 */
void telnet_console_task(void *pvParameter);

// ==============================================
// CONFIGURATION VALIDATION
// ==============================================

#if TELNET_PORT < 1 || TELNET_PORT > 65535
#error "Invalid TELNET_PORT: must be between 1 and 65535"
#endif

#if MAX_CLIENTS < 1 || MAX_CLIENTS > 10
#error "Invalid MAX_CLIENTS: must be between 1 and 10"
#endif

#if CMD_BUFFER_SIZE < 64 || CMD_BUFFER_SIZE > 1024
#error "Invalid CMD_BUFFER_SIZE: must be between 64 and 1024"
#endif

#if RESPONSE_BUFFER_SIZE < 128 || RESPONSE_BUFFER_SIZE > 2048
#error "Invalid RESPONSE_BUFFER_SIZE: must be between 128 and 2048"
#endif

#endif // TELNET_CONSOLE_H