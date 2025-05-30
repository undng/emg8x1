//
// Copyright 2019-2020 rf-lab.org
// (MIREA KB-2( frmly. MIREA KB-3, MGUPI IT-6 "Control and simulation in technical systems"))
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// EMG-8x/app_main.c - main source file for EMG-8x hardware platform
// Please refer to:

// EMG platform
//      https://github.com/RF-Lab/emg_platform

// How to install SDK for ESP32 under Windows(r)
//      http://rf-lab.org/news/2020/04/04/esp-idf.html

// List of modifications:
//      https://github.com/RF-Lab/emg_platform/commits/master/source/esp32/emg8x/main/app_main.c
//
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "hal/gpio_types.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"

#include  "esp_chip_info.h"
#include "esp_timer.h"  // Для esp_timer_get_time()


spi_device_handle_t         spi_dev ;

#define TELNET_PORT 23
#define MAX_CLIENTS 2
#define CMD_BUFFER_SIZE 256
#define RESPONSE_BUFFER_SIZE 512

esp_err_t telnet_console_init(void);
void telnet_console_task(void *pvParameter);
// Структура клиента telnet
typedef struct {
    int socket;
    bool active;
    char cmd_buffer[CMD_BUFFER_SIZE];
    int cmd_pos;
} telnet_client_t;

// Структура команды консоли
typedef struct {
    const char* cmd;
    esp_err_t (*handler)(int socket, int argc, char** argv);
    const char* help;
} console_command_t;

// Глобальные переменные telnet
static telnet_client_t telnet_clients[MAX_CLIENTS];
static int telnet_server_socket = -1;
static const char *TELNET_TAG = "TELNET_CONSOLE";

// Функции telnet консоли
static void telnet_send_response(int client_socket, const char* response);
static void send_prompt(int socket);
static esp_err_t cmd_help(int socket, int argc, char** argv);
static esp_err_t cmd_status(int socket, int argc, char** argv);
static esp_err_t cmd_set_channel(int socket, int argc, char** argv);
static esp_err_t cmd_read_reg(int socket, int argc, char** argv);
static esp_err_t cmd_write_reg(int socket, int argc, char** argv);
static esp_err_t cmd_reset_adc(int socket, int argc, char** argv);
static esp_err_t cmd_start_stop(int socket, int argc, char** argv);
static esp_err_t cmd_reboot(int socket, int argc, char** argv);
static esp_err_t cmd_info(int socket, int argc, char** argv);

// Константы для переключения режимов
#define WIFI_MODE_BUTTON_PIN    GPIO_NUM_0      // BOOT кнопка на NodeMCU
#define BUTTON_PRESS_TIME_MS    3000            // 3 секунды удержания для переключения
#define AP_MODE_IP              "192.168.4.1"  // Фиксированный IP в режиме AP
#define AP_MODE_SSID            "EMG8x_Setup"   // Имя точки доступа
#define AP_MODE_PASSWORD        "emg8x123"      // Пароль точки доступа

// Глобальные переменные для WiFi
static bool wifi_ap_mode = false;               // false = STA, true = AP
static int button_press_start_time = 0;
static bool button_was_pressed = false;
static TaskHandle_t wifiModeTaskHandle;



static const char *TAG                          = "EMG8x" ;

// Pinout mapping
// Use pinout rules from here: (https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
// NodeMCU 32-s2 pinout: (https://www.instructables.com/id/ESP32-Internal-Details-and-Pinout/)

// DEBUG purpose pins
//static const gpio_num_t     DEBUG_PIN1          = GPIO_NUM_27 ;         // Debug pin #1
static const gpio_num_t         ESP_LED1          = GPIO_NUM_2 ;         // Debug pin #1

#if CONFIG_EMG8X_BOARD_EMULATION == 0

#if CONFIG_EMG8X_BOARD_REV == 1

static const gpio_num_t     AD1299_PWDN_PIN     = GPIO_NUM_22 ;         // ADS<--ESP Power down pin (active low)
static const gpio_num_t     AD1299_RESET_PIN    = GPIO_NUM_32 ;         // ADS<--ESP Reset pin (active low)
static const gpio_num_t     AD1299_DRDY_PIN     = GPIO_NUM_33 ;         // ADS-->ESP DRDY pin (active low)
static const gpio_num_t     AD1299_START_PIN    = GPIO_NUM_21 ;         // ADS<--ESP Start data conversion pint (active high)

#elif CONFIG_EMG8X_BOARD_REV == 2

static const gpio_num_t     AD1299_PWDN_PIN     = GPIO_NUM_22 ;         // ADS<--ESP Power down pin (active low)
static const gpio_num_t     AD1299_RESET_PIN    = GPIO_NUM_32 ;         // ADS<--ESP Reset pin (active low)
static const gpio_num_t     AD1299_DRDY_PIN     = GPIO_NUM_4 ;          // ADS-->ESP DRDY pin (active low)
static const gpio_num_t     AD1299_START_PIN    = GPIO_NUM_21 ;         // ADS<--ESP Start data conversion pint (active high)

#elif CONFIG_EMG8X_BOARD_REV == 3

// Same as Rev 2
static const gpio_num_t     AD1299_PWDN_PIN     = GPIO_NUM_22 ;         // ADS<--ESP Power down pin (active low)
static const gpio_num_t     AD1299_RESET_PIN    = GPIO_NUM_32 ;         // ADS<--ESP Reset pin (active low)
static const gpio_num_t     AD1299_DRDY_PIN     = GPIO_NUM_4 ;          // ADS-->ESP DRDY pin (active low)
static const gpio_num_t     AD1299_START_PIN    = GPIO_NUM_21 ;         // ADS<--ESP Start data conversion pint (active high)

#elif CONFIG_EMG8X_BOARD_REV == 4

// Same as Rev 2,3
static const gpio_num_t     AD1299_PWDN_PIN     = GPIO_NUM_22 ;         // ADS<--ESP Power down pin (active low)
static const gpio_num_t     AD1299_RESET_PIN    = GPIO_NUM_32 ;         // ADS<--ESP Reset pin (active low)
static const gpio_num_t     AD1299_DRDY_PIN     = GPIO_NUM_4 ;          // ADS-->ESP DRDY pin (active low)
static const gpio_num_t     AD1299_START_PIN    = GPIO_NUM_21 ;         // ADS<--ESP Start data conversion pint (active high)
static const gpio_num_t     BOARD_LED1          = GPIO_NUM_25 ;         // Extra led1
static const gpio_num_t     BOARD_LED2          = GPIO_NUM_26 ;         // Extra led2
static const gpio_num_t     BOARD_LED3          = GPIO_NUM_27 ;         // Extra led3

#else

#endif

// SPI comands (see https://www.ti.com/lit/ds/symlink/ads1299.pdf?ts=1599826124971)
static const uint8_t        AD1299_CMD_RREG     = 0x20 ;                // Read register
static const uint8_t        AD1299_CMD_WREG     = 0x40 ;                // Write to register
static const uint8_t        AD1299_CMD_RDATAC   = 0x10 ;                // Start continouous mode
static const uint8_t        AD1299_CMD_SDATAC   = 0x11 ;                // Stop continuous mode

// Register addresses available through SPI (see https://www.ti.com/lit/ds/symlink/ads1299.pdf?ts=1599826124971)
static const uint8_t        AD1299_ADDR_ID      = 0x00 ;                // ID register
static const uint8_t        AD1299_ADDR_CONFIG1 = 0x01 ;                // CONFIG1 register
static const uint8_t        AD1299_ADDR_CONFIG2 = 0x02 ;                // CONFIG2 register
static const uint8_t        AD1299_ADDR_CONFIG3 = 0x03 ;                // CONFIG3 register
static const uint8_t        AD1299_ADDR_LEADOFF = 0x04 ;
static const uint8_t        AD1299_ADDR_CH1SET  = 0x05 ;
static const uint8_t        AD1299_ADDR_CH2SET  = 0x06 ;
static const uint8_t        AD1299_ADDR_CH3SET  = 0x07 ;
static const uint8_t        AD1299_ADDR_CH4SET  = 0x08 ;
static const uint8_t        AD1299_ADDR_CH5SET  = 0x09 ;
static const uint8_t        AD1299_ADDR_CH6SET  = 0x0a ;
static const uint8_t        AD1299_ADDR_CH7SET  = 0x0b ;
static const uint8_t        AD1299_ADDR_CH8SET  = 0x0c ;

#endif

// AD1299 constants (see https://www.ti.com/lit/ds/symlink/ads1299.pdf?ts=1599826124971)
// Number of analog channels
#define                     AD1299_NUM_CH                   8

// Transport protocol constants

// Transport header size (bytes), use 'idf.py menuconfig' to change this constant
// CONFIG_EMG8X_TRANSPORT_BLOCK_HEADER_SIZE =               16

// Offset to packet counter in 32 bit words, use 'idf.py menuconfig' to change this constant
// CONFIG_EMG8X_PKT_COUNT_OFFSET =                          2

// Number of 32bit samples per transport block, use 'idf.py menuconfig' to change this constant
// CONFIG_EMG8X_SAMPLES_PER_TRANSPORT_BLOCK =               64

// Device thread to transport queue size, use 'idf.py menuconfig' to change this constant
// Number of transport blocks to queue
// CONFIG_EMG8X_TRANSPORT_QUE_SIZE =                        2

// TCP server port to listen, use 'idf.py menuconfig' to change this constant
// CONFIG_EMG8X_TCP_SERVER_PORT =                           3000


IRAM_ATTR void spi_data_pump_task( void* pvParameter ) ;

DRAM_ATTR spi_device_handle_t         spi_dev ;

DRAM_ATTR TaskHandle_t      tcpTaskHandle ;
DRAM_ATTR TaskHandle_t      datapumpTaskHandle ;
DRAM_ATTR TaskHandle_t      telnetTaskHandle ;    

// DRDY signal interrupt context
typedef struct
{
    SemaphoreHandle_t       xSemaphore ;
} type_drdy_isr_context ;
DRAM_ATTR type_drdy_isr_context       drdy_isr_context ;

// ADC data variables combined into the structure
typedef struct
{

    uint8_t                 spiReadBuffer [27] ;            // Buffer to receive raw data from ADS1299 th SPI transaction

    uint32_t                adcStat32 ;                     // STAT field of last transaction in 32bit form (only 24bits used)

    // ISR to thread data queue
    int                     head ;          // 0 to CONFIG_EMG8X_TRANSPORT_QUE_SIZE-1
    int                     tail ;          // 0 to CONFIG_EMG8X_TRANSPORT_QUE_SIZE-1

    int                     sampleCount ;   // 0 to CONFIG_EMG8X_SAMPLES_PER_TRANSPORT_BLOCK-1

    uint32_t                blockCounter ;  // Number of received from ADC data blocks (num of full transport blocks)

    // read by thread, written by isr
    int32_t                 adcDataQue[CONFIG_EMG8X_TRANSPORT_QUE_SIZE][(CONFIG_EMG8X_TRANSPORT_BLOCK_HEADER_SIZE)/sizeof(int32_t)+(AD1299_NUM_CH+1)*(CONFIG_EMG8X_SAMPLES_PER_TRANSPORT_BLOCK)] ;

    // Counting semaphore represents number of transport blocks queued for
    // TCP delivery task
    SemaphoreHandle_t       xDataQueueSemaphore ;

} type_drdy_thread_context ;
DRAM_ATTR type_drdy_thread_context    drdy_thread_context ;

// TCP Server context
typedef struct
{

    // Pointer to ADC data
    type_drdy_thread_context*   pDrdyThreadContext ;
    char                        serverIpAddr[128] ;

} type_tcp_server_thread_context ;
DRAM_ATTR type_tcp_server_thread_context tcp_server_thread_context ;

// #if CONFIG_WIFI_MODE_SOFTAP

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station %02x:%02x:%02x:%02x:%02x:%02x join, AID=%d", 
                 event->mac[0], event->mac[1], event->mac[2], 
                 event->mac[3], event->mac[4], event->mac[5], 
                 event->aid);
    } 
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) 
    {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station %02x:%02x:%02x:%02x:%02x:%02x leave, AID=%d", 
                 event->mac[0], event->mac[1], event->mac[2], 
                 event->mac[3], event->mac[4], event->mac[5], 
                 event->aid);
    }
}
 esp_netif_ip_info_t ap_ip_info ;
void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init()) ;
    ESP_ERROR_CHECK(esp_event_loop_create_default()) ;
    esp_netif_create_default_wifi_ap() ;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT() ;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)) ;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL)) ;

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CONFIG_WIFI_SSID,
            .ssid_len = strlen(CONFIG_WIFI_SSID),
            .channel = CONFIG_WIFI_CHANNEL,
            .password = CONFIG_WIFI_PASSWORD,
            .max_connection = CONFIG_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    } ;

    if (strlen(CONFIG_WIFI_PASSWORD) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN ;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP)) ;
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config)) ;
    ESP_ERROR_CHECK(esp_wifi_start()) ;

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
            CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD, CONFIG_WIFI_CHANNEL) ;


    // Get default netif object
    esp_netif_t *esp_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF") ;
    if (esp_netif)
    {
        ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif,&ap_ip_info)) ;
        strncpy(tcp_server_thread_context.serverIpAddr, ipaddr_ntoa((const ip_addr_t *)&ap_ip_info.ip), 127) ;
        ESP_LOGI(TAG, "local AP address is: %s\n", tcp_server_thread_context.serverIpAddr ) ;
    }

    //strncpy(tcp_server_thread_context.serverIpAddr, ipaddr_ntoa(&gnetif.ip_addr), 127) ;
    //ESP_LOGI(TAG, ipaddr_ntoa(gnetif.ip_addr.addr) )
    //ESP_LOGI(TAG, "local address: %s\n", tcp_server_thread_context.serverIpAddr ) ;

}

// #else


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group ;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define EXAMPLE_ESP_MAXIMUM_RETRY 10

static int s_retry_num = 0 ;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect() ;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP") ;
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail") ;
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        strncpy(tcp_server_thread_context.serverIpAddr, ipaddr_ntoa((const ip_addr_t *)&event->ip_info.ip), 127) ;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT) ;
    }
}

// static void wifi_init(void)
// {
//     s_wifi_event_group = xEventGroupCreate() ;

//     ESP_ERROR_CHECK(esp_netif_init()) ;

//     ESP_ERROR_CHECK(esp_event_loop_create_default()) ;
//     esp_netif_create_default_wifi_sta() ;

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT() ;
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg)) ;

//     esp_event_handler_instance_t instance_any_id ;
//     esp_event_handler_instance_t instance_got_ip ;
//     ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
//                                                         ESP_EVENT_ANY_ID,
//                                                         &event_handler,
//                                                         NULL,
//                                                         &instance_any_id)) ;
//     ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
//                                                         IP_EVENT_STA_GOT_IP,
//                                                         &event_handler,
//                                                         NULL,
//                                                         &instance_got_ip)) ;

//     wifi_config_t wifi_config = {
//         .sta = {
//             .ssid = CONFIG_WIFI_SSID,
//             .password = CONFIG_WIFI_PASSWORD,
//             /* Setting a password implies station will connect to all security modes including WEP/WPA.
//              * However these modes are deprecated and not advisable to be used. Incase your Access point
//              * doesn't support WPA2, these mode can be enabled by commenting below line */
// 	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

//             .pmf_cfg = {
//                 .capable = true,
//                 .required = false
//             },
//         },
//     };
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
//     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
//     ESP_ERROR_CHECK(esp_wifi_start() );

//     ESP_LOGI(TAG, "wifi_init_sta finished.");

//     /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
//      * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
//     EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
//             WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
//             pdFALSE,
//             pdFALSE,
//             portMAX_DELAY);

//     /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
//      * happened. */
//     if (bits & WIFI_CONNECTED_BIT) {
//         ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
//                  CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD ) ;
// #if CONFIG_EMG8X_BOARD_REV == 4
//         gpio_set_level( BOARD_LED1,     1 ) ;
// #endif

//     } else if (bits & WIFI_FAIL_BIT) {
//         ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
//                  CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD ) ;
//     } else {
//         ESP_LOGE(TAG, "UNEXPECTED EVENT") ;
//     }

//     /* The event will not be processed after unregister */
//     ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
//     ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
//     vEventGroupDelete(s_wifi_event_group);
// }
// #endif


#if CONFIG_EMG8X_BOARD_EMULATION == 0
// send 8bit command
IRAM_ATTR void ad1299_send_cmd8(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret ;
    spi_transaction_t t ;
    memset(&t, 0, sizeof(spi_transaction_t)) ;      //Zero out the transaction
    t.flags         = SPI_TRANS_USE_TXDATA ;        //Bitwise OR of SPI_TRANS_* flags
    t.length        = 8 ;                           //Total data length, in bits
    t.user          = (void*)0 ;                    //User-defined variable. Can be used to store eg transaction ID.
    t.tx_data[0]    = cmd ;                         //Pointer to transmit buffer, or NULL for no MOSI phase
    t.rx_buffer     = NULL ;                        //Pointer to receive buffer, or NULL for no MISO phase. Written by 4 bytes-unit if DMA is used.

    int icmd = cmd ;
    ESP_LOGI( TAG, "ad1299_cmd8: send command:0x%02X", icmd ) ;

    ret = spi_device_polling_transmit( spi, &t ) ;  // send command
    if (ret==ESP_OK)
    {
        ESP_LOGI( TAG, "Sent successfuly" ) ;
    }
    ESP_ERROR_CHECK(ret) ;            //Should have had no issues.
}

// Write to ad1299 register
IRAM_ATTR void ad1299_wreg(spi_device_handle_t spi, const uint8_t addr, const uint8_t value)
{
    esp_err_t ret ;
    spi_transaction_t t ;
    memset(&t, 0, sizeof(spi_transaction_t)) ;      //Zero out the transaction
    t.flags         = SPI_TRANS_USE_TXDATA ;        //Bitwise OR of SPI_TRANS_* flags
    t.length        = 8*3 ;                         //Total data length, in bits
    t.user          = (void*)0 ;                    //User-defined variable. Can be used to store eg transaction ID.
    t.tx_data[0]    = AD1299_CMD_WREG|addr ;
    t.tx_data[1]    = 0 ;                           // 1 register to read
    t.tx_data[2]    = value ;                       // value to write
    t.tx_data[3]    = 0 ;                           // NOP
    t.rx_buffer     = NULL ;                        //Pointer to receive buffer, or NULL for no MISO phase. Written by 4 bytes-unit if DMA is used.

    ESP_LOGI( TAG, "ad1299_wreg :0x%02X to REG:0x%02X", value, addr ) ;
    ret = spi_device_polling_transmit(spi, &t) ;    //Transmit!
    if (ret==ESP_OK)
    {
        ESP_LOGI( TAG, "Sent successfuly" ) ;
    }
    ESP_ERROR_CHECK(ret) ;            //Should have had no issues.
}

IRAM_ATTR uint8_t ad1299_rreg(spi_device_handle_t spi, const uint8_t addr)
{
    esp_err_t ret ;
    spi_transaction_t t ;
    memset( &t, 0, sizeof(spi_transaction_t) ) ;    //Zero out the transaction
    t.flags         = SPI_TRANS_USE_RXDATA|
                      SPI_TRANS_USE_TXDATA ;        //Bitwise OR of SPI_TRANS_* flags
    t.length        = 8*2 ;                         //Total data length, in bits
    t.rxlength      = 8 ;                           //Total data length received, should be not greater than ``length`` in full-duplex mode (0 defaults this to the value of ``length``)
    t.user          = (void*)0 ;                    //User-defined variable. Can be used to store eg transaction ID.

    t.tx_data[0]    = AD1299_CMD_RREG|addr ;
    t.tx_data[1]    = 0 ;                           // 1 register to read
    t.tx_data[2]    = 0 ;                           // NOP
    t.tx_data[3]    = 0 ;                           // NOP

    ESP_LOGI( TAG, "ad1299_rreg: read from REG:0x%02X", addr ) ;
    ret = spi_device_polling_transmit(spi, &t) ;
    if (ret==ESP_OK)
    {
        //ESP_LOGI( TAG, "Read successfuly: 0x%02X", t.rx_data[0] ) ;
        //ESP_LOGI( TAG, "Read successfuly: 0x%02X", t.rx_data[1] ) ;
        //ESP_LOGI( TAG, "Read successfuly: 0x%02X", t.rx_data[2] ) ;
        //ESP_LOGI( TAG, "Read successfuly: 0x%02X", t.rx_data[3] ) ;
    }
    ESP_ERROR_CHECK(ret) ;                          //Should have had no issues.

    return (t.rx_data[0]) ;

}

/*
    ad1299_read_data_block216(spi_device_handle_t spi, uint8_t* data216)
    spi     - spi handle
    data216 - pointer to array of 27 bytes length (216 bits=24bit * 9)
*/
IRAM_ATTR void ad1299_read_data_block216(spi_device_handle_t spi, uint8_t* data216)
{
    esp_err_t ret ;
    spi_transaction_t t ;
    memset( &t, 0, sizeof(spi_transaction_t) ) ;    //Zero out the transaction
    t.flags         = 0 ;                           //Bitwise OR of SPI_TRANS_* flags
    t.length        = 216 ;                         //Total data length, in bits
    t.rxlength      = 216 ;                         //Total data length received, should be not greater than ``length`` in full-duplex mode (0 defaults this to the value of ``length``)
    t.user          = (void*)0 ;                    //User-defined variable. Can be used to store eg transaction ID.
    t.tx_buffer     = NULL ;                        // There are No tx data to send
    t.rx_buffer     = data216 ;                     // Pointer to Rx buffer

    ret = spi_device_polling_transmit(spi, &t) ;
    ESP_ERROR_CHECK(ret) ;                          //Should have had no issues.

}

// Rx data buffer for 1 transaction (9*24 bits)
uint8_t rxDataBuf [27] ;

// DRDY signal ISR
// DRDY becomes low when ADS1299 collect 8 samples (1 sample of 24bit for each channel )
// and ready to transfer these data to ESP32 using 1 SPI transaction with 9*24 bits length
IRAM_ATTR static void drdy_gpio_isr_handler(void* arg)
{
    //type_drdy_isr_context* pContext = (type_drdy_isr_context*) arg ;
    static BaseType_t high_task_wakeup = pdFALSE ;

    //xSemaphoreGiveFromISR( pContext->xSemaphore, &high_task_wakeup ) ;

    vTaskNotifyGiveFromISR( datapumpTaskHandle, &high_task_wakeup ) ;

    /* If high_task_wakeup was set to true you
    should yield.  The actual macro used here is
    port specific. */
    if ( high_task_wakeup!=pdFALSE )
    {
        portYIELD_FROM_ISR( ) ;
    }

}

#endif

//
// TCP server task
// This routine is waiting for data appears in ADC queue and
// then send to to connected tcp client(s)
char addr_str[128] ;
void tcp_server_task( void* pvParameter )
{

    // This routine is based on https://github.com/sankarcheppali/esp_idf_esp32_posts/blob/master/tcp_server/sta_mode/main/esp_sta_tcp_server.c

    type_tcp_server_thread_context* pTcpServerContext   = (type_tcp_server_thread_context*) pvParameter ;

    struct sockaddr_in tcpServerAddr ;
    tcpServerAddr.sin_addr.s_addr                       = htonl( INADDR_ANY ) ;
    tcpServerAddr.sin_family                            = AF_INET ;
    tcpServerAddr.sin_port                              = htons( CONFIG_EMG8X_TCP_SERVER_PORT ) ;
    int s ;
    static struct sockaddr_in remote_addr ;
    static long unsigned int socklen ;
    socklen = sizeof(remote_addr) ;
    int cs ; //client socket

    //xEventGroupWaitBits(wifi_event_group,CONNECTED_BIT,false,true,portMAX_DELAY) ;

    while(1)
    {
        s = socket( AF_INET, SOCK_STREAM, 0) ;
        if(s < 0)
        {
            ESP_LOGE(TAG, "TCP_SERVER: Failed to allocate socket.\n") ;
            vTaskDelay(1000 / portTICK_PERIOD_MS) ;
            continue ;
        }

        if(bind(s, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr)) != 0)
        {
            ESP_LOGE(TAG, "TCP_SERVER: socket bind failed errno=%d \n", errno) ;
            close(s) ;
            vTaskDelay(4000 / portTICK_PERIOD_MS) ;
            continue ;
        }

        if(listen (s, 1024) != 0)
        {
            ESP_LOGE(TAG, "TCP_SERVER: socket listen failed errno=%d \n", errno) ;
            close(s) ;
            vTaskDelay(4000 / portTICK_PERIOD_MS) ;
            continue ;
        }

        ESP_LOGI(TAG,"TCP server started at %s, listen on the port: %d, at CpuCore%1d\n", pTcpServerContext->serverIpAddr, CONFIG_EMG8X_TCP_SERVER_PORT, xPortGetCoreID() ) ;

        // data transfer cycle
        while(1)
        {
            cs  =   accept(s,(struct sockaddr *)&remote_addr, &socklen) ;

            inet_ntoa_r(remote_addr.sin_addr.s_addr, addr_str, sizeof(addr_str) - 1) ;
            ESP_LOGI(TAG,"New incoming connection from: %s\n", addr_str ) ;

#if CONFIG_EMG8X_BOARD_REV == 4
        gpio_set_level( BOARD_LED2,     1 ) ;
#endif


            while(1)
            {

                if (drdy_thread_context.tail!=drdy_thread_context.head)
                {
#if CONFIG_EMG8X_BOARD_REV == 4
                    gpio_set_level( BOARD_LED3,     1 ) ;
#endif

                    if( write(cs , drdy_thread_context.adcDataQue[drdy_thread_context.tail],
                        ((CONFIG_EMG8X_TRANSPORT_BLOCK_HEADER_SIZE)/sizeof(int32_t)+(AD1299_NUM_CH+1)*(CONFIG_EMG8X_SAMPLES_PER_TRANSPORT_BLOCK))*sizeof(int32_t)) < 0)
                    {
                        ESP_LOGE(TAG, "TCP_SERVER: Send failed\n") ;
                        break ;
                    }

#if CONFIG_EMG8X_BOARD_REV == 4
                    gpio_set_level( BOARD_LED3,     0 ) ;
#endif
                    ESP_LOGI(TAG, "TCP_SERVER: Block %d Sent.", (int)drdy_thread_context.adcDataQue[drdy_thread_context.tail][CONFIG_EMG8X_PKT_COUNT_OFFSET] ) ;

                    // move queue tail forward
                    drdy_thread_context.tail        = (drdy_thread_context.tail+1)%(CONFIG_EMG8X_TRANSPORT_QUE_SIZE) ;
                }

                //taskYIELD() ;

                // Notify or/and wait for data appears in the queue
                //if ( xSemaphoreTake( drdy_thread_context.xDataQueueSemaphore, 0xFFFF ) != pdTRUE )
                if (!ulTaskNotifyTake( pdFALSE, 0xffff )) // Wait or/and decrement count
                {
                    // still waiting for data from the ADC
                    ESP_LOGE(TAG, "TCP_SERVER: There are no data from ADC for a long time...\n") ;
                    continue ;
                }

            }

            inet_ntoa_r(remote_addr.sin_addr.s_addr, addr_str, sizeof(addr_str) - 1) ;
            ESP_LOGI(TAG, "Close connection to %s\n", addr_str ) ;

            close(cs) ;

#if CONFIG_EMG8X_BOARD_REV == 4
        gpio_set_level( BOARD_LED2,     0 ) ;
        gpio_set_level( BOARD_LED3,     0 ) ;
#endif

            vTaskDelay( 500 / portTICK_PERIOD_MS) ;
        }
    }

}

static void emg8x_app_start(void)
{
#if CONFIG_EMG8X_BOARD_EMULATION==0
//    int dmaChan             = 0 ;  // disable dma
    esp_err_t ret           = 0 ;

    ESP_LOGI(TAG, "Initialize GPIO lines") ;

    // Initialize GPIO pins

    

    gpio_reset_pin( AD1299_PWDN_PIN ) ;
    gpio_set_direction( AD1299_PWDN_PIN, GPIO_MODE_OUTPUT ) ;

    gpio_reset_pin( AD1299_RESET_PIN ) ;
    gpio_set_direction( AD1299_RESET_PIN, GPIO_MODE_OUTPUT ) ;

    gpio_reset_pin( AD1299_START_PIN ) ;
    gpio_set_direction( AD1299_START_PIN, GPIO_MODE_OUTPUT ) ;

    gpio_reset_pin( AD1299_DRDY_PIN ) ;
    gpio_set_direction( AD1299_DRDY_PIN, GPIO_MODE_INPUT ) ;
    gpio_set_intr_type( AD1299_DRDY_PIN, GPIO_INTR_NEGEDGE ) ;
    gpio_intr_enable( AD1299_DRDY_PIN ) ;

    // Debug pin
    //gpio_reset_pin( DEBUG_PIN1 ) ;
    //gpio_set_direction( DEBUG_PIN1, GPIO_MODE_OUTPUT ) ;
    //gpio_set_level( DEBUG_PIN1,     0 ) ;

    // See 10.1.2 Setting the Device for Basic Data Capture (ADS1299 Datasheet)
    ESP_LOGI(TAG, "Set PWDN & RESET to 1") ;
    gpio_set_level(AD1299_PWDN_PIN,     1 ) ;
    gpio_set_level(AD1299_RESET_PIN,    1 ) ;
    gpio_set_level(AD1299_START_PIN,    0 ) ;

    ESP_LOGI(TAG, "Wait for 20 tclk") ;

    vTaskDelay( 100 / portTICK_PERIOD_MS ) ;
    //ets_delay_us( 15*10 ) ; //~30 clock periods @2MHz

    // Reset pulse
    ESP_LOGI(TAG, "Reset ad1299") ;
    gpio_set_level(AD1299_RESET_PIN, 0 ) ;
    vTaskDelay( 100 / portTICK_PERIOD_MS ) ;
    //ets_delay_us( 15*10 ) ;  //~20 clock periods @2MHz - This delay does not works, it is not enough for reset
    gpio_set_level(AD1299_RESET_PIN, 1 ) ;

    vTaskDelay( 500 / portTICK_PERIOD_MS ) ;

    ESP_LOGI(TAG, "Initialize SPI driver...") ;
    // SEE esp-idf/components/driver/include/driver/spi_common.h
    spi_bus_config_t buscfg = {
        .miso_io_num        = GPIO_NUM_19,
        .mosi_io_num        = GPIO_NUM_23,
        .sclk_io_num        = GPIO_NUM_18,
        .quadwp_io_num      = -1,
        .quadhd_io_num      = -1,
        .flags              = 0,                        // Abilities of bus to be checked by the driver. Or-ed value of ``SPICOMMON_BUSFLAG_*`` flags.
        .intr_flags         = ESP_INTR_FLAG_IRAM,
        .max_transfer_sz    = 0                         // maximum data size in bytes, 0 means 4094
    } ;

    // see esp-idf/components/driver/include/driver/spi_master.h
    spi_device_interface_config_t devcfg = {
        .command_bits       = 0,                        // 0-16
        .address_bits       = 0,                        // 0-64
        .dummy_bits         = 0,                        // Amount of dummy bits to insert between address and data phase
        .clock_speed_hz     = 1000000,                  // Clock speed, divisors of 80MHz, in Hz. See ``SPI_MASTER_FREQ_*``.
        .mode               = 1,                        // SPI mode 0
        .flags              = SPI_DEVICE_HALFDUPLEX,    // Bitwise OR of SPI_DEVICE_* flags
        .input_delay_ns     = 0,                        // The time required between SCLK and MISO
        .spics_io_num       = GPIO_NUM_5,               // CS pin
        .queue_size         = 1,                        // No queued transactions
        .cs_ena_pretrans    = 1,                        // 0 not used
        .cs_ena_posttrans   = 0,                        // 0 not used
    } ;

    //Initialize the SPI bus
    ret                     = spi_bus_initialize(VSPI_HOST, &buscfg, 0 ) ;
    ESP_ERROR_CHECK(ret) ;
    //Attach the LCD to the SPI bus
    ret                     = spi_bus_add_device(VSPI_HOST, &devcfg, &spi_dev ) ;
    ESP_ERROR_CHECK(ret) ;
    ESP_LOGI(TAG, "SPI driver initialized, Freq limit is:%dHz", (int)spi_get_freq_limit( false, 0 ) ) ;


    // Send SDATAC / Stop Read Data Continuously Mode
    ESP_LOGI(TAG, "Send SDATAC") ;

    ad1299_send_cmd8( spi_dev, AD1299_CMD_SDATAC ) ;
    vTaskDelay( 100 / portTICK_PERIOD_MS ) ;

    /*

    ESP_LOGI(TAG, "Read chip Id from Reg#0: 0x%02x", ad1299_rreg( spi_dev, AD1299_ADDR_ID ) ) ;

    ad1299_send_cmd8( spi_dev, AD1299_CMD_RDATAC ) ;

    gpio_set_level(AD1299_START_PIN, 1 ) ;

    // Install ISR for all GPIOs
    gpio_install_isr_service(0) ;

    // Configure ISR for DRDY signal
    drdy_isr_context.xSemaphore                     = xSemaphoreCreateBinary() ;
    gpio_isr_handler_add( AD1299_DRDY_PIN, drdy_gpio_isr_handler, &drdy_isr_context ) ;

    while(1)
    {

        // Wait for DRDY
        //while( gpio_get_level(AD1299_DRDY_PIN)==1 ) { vTaskDelay( 1  ) ;}
        if( xSemaphoreTake( drdy_isr_context.xSemaphore, 0xffff ) == pdTRUE )
        {

            // DRDY goes down - data ready to read
            ad1299_read_data_block216( spi_dev, drdy_thread_context.spiReadBuffer ) ;
        }
    }
    */


    // RREG id
    ESP_LOGI(TAG, "Read chip Id from Reg#0:") ;
    uint8_t valueu8             = ad1299_rreg( spi_dev, AD1299_ADDR_ID ) ;

    uint8_t ad1299_rev_id       = valueu8>>5 ;
    uint8_t ad1299_check_bit    = (valueu8>>4) & 0x01 ;
    uint8_t ad1299_dev_id       = (valueu8>>2) & 0x03 ;
    uint8_t ad1299_num_ch       = (valueu8) & 0x03 ;

    if (ad1299_check_bit && ad1299_dev_id==0x03 )
    {
        ESP_LOGI(TAG,"ads1299 found:") ;
        ESP_LOGI(TAG, "!-->ad1299_rev_id:       0x%02X",  ad1299_rev_id ) ;
        ESP_LOGI(TAG, "!-->ad1299_check_bit:    %1d (should be 1)",  ad1299_check_bit ) ;
        ESP_LOGI(TAG, "!-->ad1299_dev_id:       0x%02X",  ad1299_dev_id ) ;
        ESP_LOGI(TAG, "!-->ad1299_num_ch:       0x%02X",  ad1299_num_ch ) ;
    }
    else
    {
        ESP_LOGE(TAG, "error: ads1299 not found!" ) ;
        return ;
    }

    gpio_set_level( ESP_LED1,     1 ) ;


    // Set internal reference
    // WREG CONFIG3 E0h
    ESP_LOGI(TAG, "Set internal reference" ) ;
    ad1299_wreg( spi_dev, AD1299_ADDR_CONFIG3, 0xEE ) ;
    vTaskDelay( 100 / portTICK_PERIOD_MS ) ;

 /*   while(1)
    {
        uint8_t reg_value = ad1299_rreg( spi_dev, AD1299_ADDR_CONFIG3 ) ;
        ESP_LOGI(TAG, "check CONF3:       0x%02X",  reg_value ) ;
        vTaskDelay( 500 / portTICK_PERIOD_MS ) ;
    }*/

    //Set device for DR=fmod/4096
    // Enable clk output
    ESP_LOGI(TAG, "Set sampling rate" ) ;
    ad1299_wreg( spi_dev, AD1299_ADDR_CONFIG1, 0xf4 ) ;     // Default 0x96 (see power up sequence)
    vTaskDelay( 100 / portTICK_PERIOD_MS ) ;

    // Configure test signal parameters
    ad1299_wreg( spi_dev, AD1299_ADDR_CONFIG2, 0xb5 ) ;
    vTaskDelay( 100 / portTICK_PERIOD_MS ) ;

    // Configure test signal parameters
    ad1299_wreg( spi_dev, AD1299_ADDR_LEADOFF, 0x0e ) ;
    vTaskDelay( 100 / portTICK_PERIOD_MS ) ;

    // Set All Channels to Input Short
    for (int i=0;i<8;i++)
    {
        ad1299_wreg( spi_dev, AD1299_ADDR_CH1SET+i, 0x01 ) ;
        //vTaskDelay( 50 / portTICK_PERIOD_MS ) ;
    }
    vTaskDelay( 50 / portTICK_PERIOD_MS ) ;

    // Activate Conversion
    // After This Point DRDY Toggles at
    // fCLK / 8192
    gpio_set_level(AD1299_START_PIN, 1 ) ;
    vTaskDelay( 50 / portTICK_PERIOD_MS ) ;

    // Put the Device Back in RDATAC Mode
    // RDATAC
    ad1299_send_cmd8( spi_dev, AD1299_CMD_RDATAC ) ;
    vTaskDelay( 50 / portTICK_PERIOD_MS ) ;

    // Capture data and check for noise level
    for(int nSample=0;nSample<CONFIG_EMG8X_SAMPLES_PER_TRANSPORT_BLOCK;nSample++)
    {

        // Wait for DRDY
        while( gpio_get_level(AD1299_DRDY_PIN)==1 ) { vTaskDelay( 1  ) ;}

        // DRDY goes down - data ready to read
        ad1299_read_data_block216( spi_dev, rxDataBuf ) ;

        ESP_LOGI(TAG, "DATA: STAT:0x%02X%02X%02X DATA1:0x%02X%02X%02X",  rxDataBuf[0],rxDataBuf[1],rxDataBuf[2], rxDataBuf[3],rxDataBuf[4],rxDataBuf[5] ) ;

        // Place here code to collect samples and analyse noise level for
        // shorted analog inputs

        vTaskDelay( 1 ) ;

    }

    // Stop all the channels
    ad1299_send_cmd8( spi_dev, AD1299_CMD_SDATAC ) ;
    vTaskDelay( 100 / portTICK_PERIOD_MS ) ;

    // Configure channels
    ad1299_wreg( spi_dev, AD1299_ADDR_CH1SET, 0x40 ) ;      // CH1: Test signal,    PGA_Gain=1
    ad1299_wreg( spi_dev, AD1299_ADDR_CH2SET, 0x30 ) ;      // CH2: Measure VDD,    PGA_Gain=1
    ad1299_wreg( spi_dev, AD1299_ADDR_CH3SET, 0x30 ) ;      // CH3: Normal,         PGA_Gain=1
    ad1299_wreg( spi_dev, AD1299_ADDR_CH4SET, 0x00 ) ;      // CH4: Normal,         PGA_Gain=24
    ad1299_wreg( spi_dev, AD1299_ADDR_CH5SET, 0x30 ) ;      // CH5: Normal,         PGA_Gain=24
    ad1299_wreg( spi_dev, AD1299_ADDR_CH6SET, 0x50 ) ;      // CH6: Normal,         PGA_Gain=24
    ad1299_wreg( spi_dev, AD1299_ADDR_CH7SET, 0x30 ) ;      // CH7: Normal,         PGA_Gain=24
    ad1299_wreg( spi_dev, AD1299_ADDR_CH8SET, 0x30 ) ;      // CH8: Normal,         PGA_Gain=24
    vTaskDelay( 50 / portTICK_PERIOD_MS ) ;

    ESP_LOGI(TAG, "Put device in RDATAC mode" ) ;

    // Put the Device Back in RDATAC Mode
    // RDATAC
    ad1299_send_cmd8( spi_dev, AD1299_CMD_RDATAC ) ;
    vTaskDelay( 50 / portTICK_PERIOD_MS ) ;

#endif // Emulation

    // Initialize drdy_thread_context
    drdy_thread_context.head                        = 0 ;
    drdy_thread_context.tail                        = 0 ;
    drdy_thread_context.sampleCount                 = 0 ;
    drdy_thread_context.blockCounter                = 0 ;

    // Initialize counting semaphore which represents number of queued blocks
    drdy_thread_context.xDataQueueSemaphore         = xSemaphoreCreateCounting( CONFIG_EMG8X_TRANSPORT_QUE_SIZE, 0 ) ;

    for (int iQue=0;iQue<CONFIG_EMG8X_TRANSPORT_QUE_SIZE;iQue++)
    {

        unsigned char* pTransportBlockHeader        = (unsigned char*) drdy_thread_context.adcDataQue[iQue] ;

        // Fill in each header
        memset( pTransportBlockHeader, 0, CONFIG_EMG8X_TRANSPORT_BLOCK_HEADER_SIZE ) ;

        // Sync 5 bytes
        strncpy( (char*)pTransportBlockHeader, "EMG8x", 6 ) ;

        // Initialize packet counter
        drdy_thread_context.adcDataQue[iQue][CONFIG_EMG8X_PKT_COUNT_OFFSET]     = 0 ;

    }

    // Initialize tcp_server_thread_context
    tcp_server_thread_context.pDrdyThreadContext    = &drdy_thread_context ;

    // Start TCP server task
    //xTaskCreatePinnedToCore( &tcp_server_task, "tcp_server_task", 4096, &tcp_server_thread_context, 5, NULL, 0 ) ;
    /*https://github.com/espressif/esp-idf/blob/0bbc721a633f9afe4f3ebe648b9ca99e2f2f6d5f/components/freertos/include/freertos/task.h*/
    //xTaskCreate( &tcp_server_task, "tcp_server_task", 4096, &tcp_server_thread_context, tskIDLE_PRIORITY+5, &tcpTaskHandle ) ;

    // !!!
    // Data pump task and TCP task should be pinned to different cores to prevent influence of TCP to realtime data pump procedures

    // Start TCP task on Core#0
    xTaskCreatePinnedToCore( &tcp_server_task, "tcp_server_task", 4096, &tcp_server_thread_context, tskIDLE_PRIORITY+5, &tcpTaskHandle, 0 ) ;
    configASSERT( tcpTaskHandle ) ;

    // Start Data pump task on Core#1
    xTaskCreatePinnedToCore( &spi_data_pump_task, "spi_data_pump_task", 4096, &tcp_server_thread_context, tskIDLE_PRIORITY+5, &datapumpTaskHandle, 1 ) ;
    configASSERT( datapumpTaskHandle ) ;

#if CONFIG_EMG8X_BOARD_EMULATION==0
    // ISR uses datapumpTaskHandle to deliver notification

    // Install ISR for all GPIOs
    gpio_install_isr_service(0) ;

    // Configure ISR for DRDY signal
    drdy_isr_context.xSemaphore                     = xSemaphoreCreateBinary() ;
    gpio_isr_handler_add( AD1299_DRDY_PIN, drdy_gpio_isr_handler, &drdy_isr_context ) ;

#endif

telnet_console_init();
    xTaskCreatePinnedToCore( &telnet_console_task,  "telnet_console_task", 4096, NULL, tskIDLE_PRIORITY + 3, &telnetTaskHandle, 0);
    configASSERT(telnetTaskHandle);
    
    ESP_LOGI(TAG, "Telnet console started on port 23");

}

#if CONFIG_EMG8X_BOARD_EMULATION
int numSamplesToMilliseconds(int numSamples)
{
    int sampleRate  = 16000/(1<<CONFIG_EMG8X_ADC_SAMPLING_FREQUENCY) ;
    return (1000*numSamples/sampleRate) ;
}
#endif

//float my_data_array[4096] ;
//int32_t sampleCounter = 0 ;

IRAM_ATTR void spi_data_pump_task( void* pvParameter )
{
    ESP_LOGI(TAG,"spi_data_pump_task started at CpuCore%1d\n", xPortGetCoreID() ) ;

    // Queue full flag
    int queueFull                                   = 0 ;

    // Continuous capture the data
    while(1)
    {

        // Wait for DRDY
        //while( gpio_get_level(AD1299_DRDY_PIN)==1 ) { vTaskDelay( 1  ) ;}
#if CONFIG_EMG8X_BOARD_EMULATION
        if (drdy_thread_context.sampleCount%60==0)
        {
            vTaskDelay( numSamplesToMilliseconds(60)/ portTICK_PERIOD_MS ) ;
        }
#else

        // Wait for DRDY signal becomes low
        if (gpio_get_level(AD1299_DRDY_PIN)!=0)
        {
            if (!ulTaskNotifyTake( pdTRUE, 0xffff ))
            {
                // Data is not ready
                continue ;
            }
        }

        // Wait using semaphore
        //if( gpio_get_level(AD1299_DRDY_PIN)==0 xSemaphoreTake( drdy_isr_context.xSemaphore, 0xffff )==pdTRUE )

#endif

        //gpio_set_level( DEBUG_PIN1,     1 ) ;
#if CONFIG_EMG8X_BOARD_EMULATION
        drdy_thread_context.spiReadBuffer[0] = 0xc0 ;
#else
        // Read data from device
        ad1299_read_data_block216( spi_dev, drdy_thread_context.spiReadBuffer ) ;
#endif
        //gpio_set_level( DEBUG_PIN1,     0 ) ;

        // Check for stat 0xCx presence
        if ((drdy_thread_context.spiReadBuffer[0]&0xf0)!=0xc0)
        {
            ESP_LOGE(TAG, "raw_REad:[%6d][%6d] 0xc0 not found!", (int)drdy_thread_context.blockCounter, (int)drdy_thread_context.sampleCount ) ;
            continue ;
        }

        //gpio_set_level( DEBUG_PIN1,     1 ) ;
        // get STAT field
        // transform 24 bit field to 32 bit integer
        drdy_thread_context.adcStat32   = (uint32_t) (
                                                        (((uint32_t)drdy_thread_context.spiReadBuffer[0])<<16) |
                                                        (((uint32_t)drdy_thread_context.spiReadBuffer[1])<<8) |
                                                        ((uint32_t) drdy_thread_context.spiReadBuffer[2])
                                                    ) ;

        // save STAT field to transport block
        drdy_thread_context.adcDataQue[drdy_thread_context.head][
            (CONFIG_EMG8X_TRANSPORT_BLOCK_HEADER_SIZE)/sizeof(int32_t) +
            drdy_thread_context.sampleCount ] = drdy_thread_context.adcStat32 ;

        // transform 24 bit twos-complement samples to 32 bit signed integers
        // save 32 bit samples into thread queue drdy_thread_context
        for(int chNum=0;chNum<AD1299_NUM_CH;chNum++)
        {

            uint32_t signExtBits        = 0x00000000 ;

            int byteOffsetCh            = (chNum+1)*3 ; // offset in raw data to first byte of 24 twos complement field

            if (drdy_thread_context.spiReadBuffer[ byteOffsetCh ] & 0x80)
            {
                // extend sign bit to 31..24
                signExtBits             = 0xff000000 ;
            }

            int32_t value_i32           =
                                        (int32_t) (
                                                        signExtBits |
                                                        (((uint32_t)drdy_thread_context.spiReadBuffer[byteOffsetCh])<<16) |
                                                        (((uint32_t)drdy_thread_context.spiReadBuffer[byteOffsetCh+1])<<8) |
                                                        ((uint32_t) drdy_thread_context.spiReadBuffer[byteOffsetCh+2])
                                                    ) ;
#if CONFIG_EMG8X_BOARD_EMULATION
            value_i32                   = (drdy_thread_context.sampleCount%20)<10 ;
#endif

            drdy_thread_context.adcDataQue[drdy_thread_context.head][
                (CONFIG_EMG8X_TRANSPORT_BLOCK_HEADER_SIZE)/sizeof(int32_t) +
                (CONFIG_EMG8X_SAMPLES_PER_TRANSPORT_BLOCK)*(chNum+1) +
                drdy_thread_context.sampleCount
                ]    = value_i32 ;
        } //for(int chNum=0
        //gpio_set_level( DEBUG_PIN1,     0 ) ;

        drdy_thread_context.sampleCount++ ;

        if (drdy_thread_context.sampleCount>=CONFIG_EMG8X_SAMPLES_PER_TRANSPORT_BLOCK)
        {
            // Transport block collection done

            // reset sample counter of transport block
            drdy_thread_context.sampleCount     = 0 ;

            // put packet counter to transport header
            drdy_thread_context.adcDataQue[drdy_thread_context.head][CONFIG_EMG8X_PKT_COUNT_OFFSET] = drdy_thread_context.blockCounter ;

            //gpio_set_level( DEBUG_PIN1,     1 ) ;

            if (!queueFull)
            {
                // Send notification to TCP server task and increment taskCounter
                uint32_t taskCounterValueBeforeIncrement  = 0 ;
                xTaskNotifyAndQuery( tcpTaskHandle, 0, eIncrement, &taskCounterValueBeforeIncrement ) ;
                if ((taskCounterValueBeforeIncrement+1)==CONFIG_EMG8X_TRANSPORT_QUE_SIZE)
                {
                    // Queue is over!
                    // Do not move head forward - wait for TCP server sends data to host
                    ESP_LOGE(TAG, "ADC_READ_CYCLE: TCP FIFO Full @ block %d\n", (int)drdy_thread_context.blockCounter ) ;
                    queueFull = 1 ;
                }
                else
                {
                    // There is empty space in the queue
                    // move queue head forward
                    drdy_thread_context.head        = (drdy_thread_context.head+1)%(CONFIG_EMG8X_TRANSPORT_QUE_SIZE) ;
                }

            }
            else
            {
                // check for queue state
                uint32_t taskCounterValue  = 0 ;
                xTaskNotifyAndQuery( tcpTaskHandle, 0, eNoAction, &taskCounterValue ) ;
                if ((taskCounterValue+1)<CONFIG_EMG8X_TRANSPORT_QUE_SIZE)
                {
                    ESP_LOGI(TAG, "ADC_READ_CYCLE: TCP FIFO ok @ block %d\n", (int)drdy_thread_context.blockCounter ) ;
                    queueFull = 0 ;
                }
            }

/*
            //if (xSemaphoreGive( drdy_thread_context.xDataQueueSemaphore )==pdTRUE)
            if ( !queueFull )
            {

                // move queue head forward
                drdy_thread_context.head        = (drdy_thread_context.head+1)%(CONFIG_EMG8X_TRANSPORT_QUE_SIZE) ;

                if (queueFull)
                {
                    ESP_LOGI(TAG, "ADC_READ_CYCLE: TCP FIFO ok @ block %d\n", drdy_thread_context.blockCounter ) ;
                    queueFull = 0 ;
                }
            }
            else
            {
                // There is no space in queue
                // skip data
                if (!queueFull)
                {
                    ESP_LOGE(TAG, "ADC_READ_CYCLE: TCP FIFO Full @ block %d\n", drdy_thread_context.blockCounter ) ;
                    queueFull = 1 ;
                }
            }
            */
            //gpio_set_level( DEBUG_PIN1,     0 ) ;

            // Increment received packet counter
            drdy_thread_context.blockCounter += 1 ;

            //ESP_LOGI(TAG, "head: %d",  drdy_thread_context.head ) ;
        }
    }
}

// ==============================================
// ИСПРАВЛЕНИЯ ОШИБОК КОМПИЛЯЦИИ
// ==============================================

// 1. ДОБАВИТЬ В НАЧАЛО main.c (после существующих includes):
#include "esp_timer.h"  // Для esp_timer_get_time()

// 2. ИСПРАВЛЕННЫЕ ФУНКЦИИ (заменить в коде):

// Функция для сохранения режима WiFi в NVS
static void save_wifi_mode(bool ap_mode) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("emg8x", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_set_u8(nvs_handle, "wifi_ap_mode", ap_mode ? 1 : 0);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI("EMG8x", "WiFi mode saved: %s", ap_mode ? "AP" : "STA");  // Используем константу вместо TAG
    }
}

// Функция для загрузки режима WiFi из NVS
static bool load_wifi_mode(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("emg8x", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        uint8_t mode = 0;
        err = nvs_get_u8(nvs_handle, "wifi_ap_mode", &mode);
        nvs_close(nvs_handle);
        if (err == ESP_OK) {
            ESP_LOGI("EMG8x", "WiFi mode loaded: %s", mode ? "AP" : "STA");
            return mode != 0;
        }
    }
    ESP_LOGI("EMG8x", "WiFi mode not found, using default: STA");
    return false; // По умолчанию STA режим
}

// Функция мигания светодиодом для индикации режима
static void indicate_wifi_mode(bool ap_mode) {
    // AP режим: быстрое мигание (3 раза)
    // STA режим: медленное мигание (1 раз)
    int blink_count = ap_mode ? 3 : 1;
    int blink_delay = ap_mode ? 200 : 500;
    
    for (int i = 0; i < blink_count; i++) {
        gpio_set_level(GPIO_NUM_2, 1);  // Используем GPIO_NUM_2 вместо ESP_LED1
        vTaskDelay(blink_delay / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(blink_delay / portTICK_PERIOD_MS);
    }
}

// WiFi для AP режима
static void wifi_init_ap_mode(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    // Настройка IP адреса для AP
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

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

    strncpy(tcp_server_thread_context.serverIpAddr, AP_MODE_IP, 127);
    
    ESP_LOGI("EMG8x", "WiFi AP Mode started:");
    ESP_LOGI("EMG8x", "  SSID: %s", AP_MODE_SSID);
    ESP_LOGI("EMG8x", "  Password: %s", AP_MODE_PASSWORD);
    ESP_LOGI("EMG8x", "  IP: %s", AP_MODE_IP);
    ESP_LOGI("EMG8x", "  Telnet: telnet %s", AP_MODE_IP);
}

// Объявление функции wifi_init (добавить перед использованием)
#if CONFIG_WIFI_MODE_SOFTAP
void wifi_init_softap(void);  // Объявление существующей функции
#define wifi_init_sta_mode wifi_init_softap
#else
static void wifi_init_sta_mode(void);  // Объявление для STA режима
#endif

// WiFi для STA режима
#if !CONFIG_WIFI_MODE_SOFTAP
static void wifi_init_sta_mode(void) {
    // Используем существующий код wifi_init() из main.c
    // Копируем содержимое функции wifi_init() сюда
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

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

    ESP_LOGI("EMG8x", "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("EMG8x", "connected to ap SSID:%s password:%s",
                 CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
#if CONFIG_EMG8X_BOARD_REV == 4
        gpio_set_level(BOARD_LED1, 1);
#endif
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI("EMG8x", "Failed to connect to SSID:%s, password:%s",
                 CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
    } else {
        ESP_LOGE("EMG8x", "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}
#endif

// Задача мониторинга кнопки
void wifi_mode_button_task(void *pvParameter) {
    ESP_LOGI("EMG8x", "WiFi mode button task started");
    
    while (1) {
        // Проверяем состояние кнопки (активный уровень - LOW)
        if (gpio_get_level(WIFI_MODE_BUTTON_PIN) == 0) {
            // Кнопка нажата
            if (!button_was_pressed) {
                button_was_pressed = true;
                button_press_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                ESP_LOGI("EMG8x", "Button pressed, waiting for %d ms...", BUTTON_PRESS_TIME_MS);
            } else {
                // Проверяем время удержания
                int current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if ((current_time - button_press_start_time) >= BUTTON_PRESS_TIME_MS) {
                    // Кнопка удерживалась достаточно долго
                    ESP_LOGI("EMG8x", "Button held for %d ms - switching WiFi mode", BUTTON_PRESS_TIME_MS);
                    
                    // Переключаем режим
                    wifi_ap_mode = !wifi_ap_mode;
                    save_wifi_mode(wifi_ap_mode);
                    
                    // Индикация переключения
                    indicate_wifi_mode(wifi_ap_mode);
                    
                    ESP_LOGI("EMG8x", "WiFi mode will be: %s after reboot", 
                             wifi_ap_mode ? "AP (Access Point)" : "STA (Client)");
                    
                    // Перезагружаемся для применения нового режима
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    esp_restart();
                }
            }
        } else {
            // Кнопка отпущена
            if (button_was_pressed) {
                int press_duration = xTaskGetTickCount() * portTICK_PERIOD_MS - button_press_start_time;
                if (press_duration < BUTTON_PRESS_TIME_MS) {
                    ESP_LOGI("EMG8x", "Button released after %d ms (too short)", press_duration);
                }
                button_was_pressed = false;
            }
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS); // Проверяем каждые 100ms
    }
}

// Инициализация кнопки и WiFi режима
static void init_wifi_mode_system(void) {
    // Настройка кнопки
    gpio_reset_pin(WIFI_MODE_BUTTON_PIN);
    gpio_set_direction(WIFI_MODE_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(WIFI_MODE_BUTTON_PIN, GPIO_PULLUP_ONLY);
    
    // Загружаем сохраненный режим WiFi
    wifi_ap_mode = load_wifi_mode();
    
    ESP_LOGI("EMG8x", "WiFi mode: %s", wifi_ap_mode ? "AP (Access Point)" : "STA (Client)");
    
    // Индикация текущего режима при загрузке
    indicate_wifi_mode(wifi_ap_mode);
    
    // Запускаем соответствующий WiFi режим
    if (wifi_ap_mode) {
        wifi_init_ap_mode();
    } else {
#if CONFIG_WIFI_MODE_SOFTAP
        wifi_init_softap();
#else
        wifi_init_sta_mode();
#endif
    }
    
    // Запускаем задачу мониторинга кнопки
    xTaskCreatePinnedToCore(
        &wifi_mode_button_task,
        "wifi_mode_button_task",
        2048,
        NULL,
        tskIDLE_PRIORITY + 2,
        &wifiModeTaskHandle,
        0
    );
    configASSERT(wifiModeTaskHandle);
}

// ==============================================
// РЕАЛИЗАЦИЯ ФУНКЦИЙ TELNET КОНСОЛИ
// ==============================================

static void telnet_send_response(int client_socket, const char* response) {
    if (client_socket >= 0) {
        send(client_socket, response, strlen(response), 0);
    }
}

static void send_prompt(int socket) {
    telnet_send_response(socket, "EMG8x> ");
}



static esp_err_t cmd_status(int socket, int argc, char** argv) {
    char response[RESPONSE_BUFFER_SIZE];
    
    snprintf(response, sizeof(response),
        "\r\nEMG8x Status:\r\n"
        "=============\r\n"
        "Block Counter: %u\r\n"
        "Sample Count: %d\r\n"
        "Queue Head: %d\r\n"
        "Queue Tail: %d\r\n"
        "Free Heap: %u bytes\r\n"
        "\r\n",
        (unsigned)drdy_thread_context.blockCounter,
        drdy_thread_context.sampleCount,
        drdy_thread_context.head,
        drdy_thread_context.tail,
        (unsigned)esp_get_free_heap_size()
    );
    
    telnet_send_response(socket, response);
    return ESP_OK;
}

static esp_err_t cmd_set_channel(int socket, int argc, char** argv) {
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
    
#if CONFIG_EMG8X_BOARD_EMULATION == 0
    // Остановим сбор данных
    ad1299_send_cmd8(spi_dev, AD1299_CMD_SDATAC);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Установим параметры канала
    uint8_t ch_set = (mode << 4) | gain;
    ad1299_wreg(spi_dev, AD1299_ADDR_CH1SET + (channel - 1), ch_set);
    
    // Возобновим сбор данных
    ad1299_send_cmd8(spi_dev, AD1299_CMD_RDATAC);
    vTaskDelay(10 / portTICK_PERIOD_MS);
#endif
    
    snprintf(response, sizeof(response),
        "\r\nChannel %d configured: gain=%d, mode=%d\r\n\r\n",
        channel, gain, mode);
    telnet_send_response(socket, response);
    
    return ESP_OK;
}

static esp_err_t cmd_read_reg(int socket, int argc, char** argv) {
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
    
#if CONFIG_EMG8X_BOARD_EMULATION == 0
    // Остановим сбор данных
    ad1299_send_cmd8(spi_dev, AD1299_CMD_SDATAC);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    uint8_t value = ad1299_rreg(spi_dev, addr);
    
    // Возобновим сбор данных
    ad1299_send_cmd8(spi_dev, AD1299_CMD_RDATAC);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    snprintf(response, sizeof(response),
        "\r\nRegister 0x%02X = 0x%02X\r\n\r\n", addr, value);
#else
    snprintf(response, sizeof(response),
        "\r\nEMULATION: Register 0x%02X = 0x00 (emulated)\r\n\r\n", addr);
#endif
    
    telnet_send_response(socket, response);
    return ESP_OK;
}

static esp_err_t cmd_write_reg(int socket, int argc, char** argv) {
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
    
#if CONFIG_EMG8X_BOARD_EMULATION == 0
    // Остановим сбор данных
    ad1299_send_cmd8(spi_dev, AD1299_CMD_SDATAC);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    ad1299_wreg(spi_dev, addr, value);
    
    // Возобновим сбор данных
    ad1299_send_cmd8(spi_dev, AD1299_CMD_RDATAC);
    vTaskDelay(10 / portTICK_PERIOD_MS);
#endif
    
    snprintf(response, sizeof(response),
        "\r\nRegister 0x%02X = 0x%02X written\r\n\r\n", addr, value);
    telnet_send_response(socket, response);
    
    return ESP_OK;
}

static esp_err_t cmd_reset_adc(int socket, int argc, char** argv) {
    char response[RESPONSE_BUFFER_SIZE];
    
#if CONFIG_EMG8X_BOARD_EMULATION == 0
    // Выполним reset ADC
    gpio_set_level(AD1299_RESET_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(AD1299_RESET_PIN, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    snprintf(response, sizeof(response), "\r\nADC reset completed\r\n\r\n");
#else
    snprintf(response, sizeof(response), "\r\nEMULATION: ADC reset (emulated)\r\n\r\n");
#endif
    
    telnet_send_response(socket, response);
    return ESP_OK;
}

static esp_err_t cmd_start_stop(int socket, int argc, char** argv) {
    char response[RESPONSE_BUFFER_SIZE];
    
#if CONFIG_EMG8X_BOARD_EMULATION == 0
    if (strcmp(argv[0], "start") == 0) {
        gpio_set_level(AD1299_START_PIN, 1);
        ad1299_send_cmd8(spi_dev, AD1299_CMD_RDATAC);
        snprintf(response, sizeof(response), "\r\nData acquisition started\r\n\r\n");
    } else {
        gpio_set_level(AD1299_START_PIN, 0);
        ad1299_send_cmd8(spi_dev, AD1299_CMD_SDATAC);
        snprintf(response, sizeof(response), "\r\nData acquisition stopped\r\n\r\n");
    }
#else
    if (strcmp(argv[0], "start") == 0) {
        snprintf(response, sizeof(response), "\r\nEMULATION: Data acquisition started\r\n\r\n");
    } else {
        snprintf(response, sizeof(response), "\r\nEMULATION: Data acquisition stopped\r\n\r\n");
    }
#endif
    
    telnet_send_response(socket, response);
    return ESP_OK;
}

static esp_err_t cmd_reboot(int socket, int argc, char** argv) {
    telnet_send_response(socket, "\r\nRebooting device in 3 seconds...\r\n\r\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    esp_restart();
    return ESP_OK;
}

// static esp_err_t cmd_info(int socket, int argc, char** argv) {
//     char response[RESPONSE_BUFFER_SIZE];
//     esp_chip_info_t chip_info;
//     esp_chip_info(&chip_info);
    
//     snprintf(response, sizeof(response),
//         "\r\nSystem Information:\r\n"
//         "==================\r\n"
//         "IDF Version: %s\r\n"
//         "Chip Model: %s\r\n"
//         "Cores: %d\r\n"
//         "Free Heap: %u bytes\r\n"
//         "Min Free Heap: %u bytes\r\n"
//         "Uptime: %llu ms\r\n"
// #if CONFIG_EMG8X_BOARD_EMULATION == 0
//         "Mode: Hardware\r\n"
// #else
//         "Mode: Emulation\r\n"
// #endif
//         "\r\n",
//         esp_get_idf_version(),
//         (chip_info.model == CHIP_ESP32) ? "ESP32" : 
//         (chip_info.model == CHIP_ESP32S2) ? "ESP32-S2" : "Unknown",
//         chip_info.cores,
//         (unsigned)esp_get_free_heap_size(),
//         (unsigned)esp_get_minimum_free_heap_size(),
//         esp_timer_get_time() / 1000
//     );
    
//     telnet_send_response(socket, response);
//     return ESP_OK;
// }

static int parse_command(char* cmd_line, char** argv, int max_args) {
    int argc = 0;
    char* token = strtok(cmd_line, " \r\n");
    
    while (token != NULL && argc < max_args) {
        argv[argc++] = token;
        token = strtok(NULL, " \r\n");
    }
    
    return argc;
}




// ==============================================
// ДОПОЛНИТЕЛЬНЫЕ КОМАНДЫ TELNET ДЛЯ УПРАВЛЕНИЯ WiFi
// ==============================================

static esp_err_t cmd_info(int socket, int argc, char** argv) {
    char response[RESPONSE_BUFFER_SIZE];
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    snprintf(response, sizeof(response),
        "\r\nSystem Information:\r\n"
        "==================\r\n"
        "IDF Version: %s\r\n"
        "Chip Model: %s\r\n"
        "Cores: %d\r\n"
        "Free Heap: %u bytes\r\n"
        "Min Free Heap: %u bytes\r\n"
        "Uptime: %llu ms\r\n"
#if CONFIG_EMG8X_BOARD_EMULATION == 0
        "Mode: Hardware\r\n"
#else
        "Mode: Emulation\r\n"
#endif
        "\r\n",
        esp_get_idf_version(),
        (chip_info.model == CHIP_ESP32) ? "ESP32" : 
        (chip_info.model == CHIP_ESP32S2) ? "ESP32-S2" : "Unknown",
        chip_info.cores,
        (unsigned)esp_get_free_heap_size(),
        (unsigned)esp_get_minimum_free_heap_size(),
        (unsigned long long)(esp_timer_get_time() / 1000)  // Исправлен тип данных
    );
    
    telnet_send_response(socket, response);
    return ESP_OK;
}

// ИСПРАВЛЕННЫЕ WiFi КОМАНДЫ
static esp_err_t cmd_wifi_info(int socket, int argc, char** argv) {
    char response[RESPONSE_BUFFER_SIZE];
    
    snprintf(response, sizeof(response),
        "\r\nWiFi Information:\r\n"
        "=================\r\n"
        "Current Mode: %s\r\n"
        "IP Address: %s\r\n"
        "\r\n"
        "Available Commands:\r\n"
        "- Hold BOOT button for 3 sec to switch mode\r\n"
        "- wifi_switch - Switch WiFi mode via command\r\n"
        "\r\n"
        "Modes:\r\n"
        "- STA (Client): Connects to existing WiFi\r\n"
        "- AP (Access Point): Creates WiFi hotspot\r\n"
        "  SSID: %s\r\n"
        "  Password: %s\r\n"
        "  IP: %s\r\n"
        "\r\n",
        wifi_ap_mode ? "AP (Access Point)" : "STA (Client)",
        tcp_server_thread_context.serverIpAddr,
        AP_MODE_SSID,
        AP_MODE_PASSWORD,
        AP_MODE_IP
    );
    
    telnet_send_response(socket, response);
    return ESP_OK;
}

static esp_err_t cmd_wifi_switch(int socket, int argc, char** argv) {
    wifi_ap_mode = !wifi_ap_mode;
    save_wifi_mode(wifi_ap_mode);
    
    char response[RESPONSE_BUFFER_SIZE];
    snprintf(response, sizeof(response),
        "\r\nWiFi mode switched to: %s\r\n"
        "Device will reboot in 3 seconds...\r\n\r\n",
        wifi_ap_mode ? "AP (Access Point)" : "STA (Client)"
    );
    
    telnet_send_response(socket, response);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    esp_restart();
    return ESP_OK;
}

static const console_command_t telnet_commands[] = {
    {"help", cmd_help, "Show available commands"},
    {"status", cmd_status, "Show device status"},
    {"info", cmd_info, "Show system information"},
    {"wifi", cmd_wifi_info, "Show WiFi information and modes"},
    {"wifi_switch", cmd_wifi_switch, "Switch WiFi mode (STA/AP)"},
    {"setch", cmd_set_channel, "setch <ch> <gain> <mode> - Set channel parameters"},
    {"rreg", cmd_read_reg, "rreg <addr> - Read register"},
    {"wreg", cmd_write_reg, "wreg <addr> <value> - Write register"},
    {"reset", cmd_reset_adc, "Reset ADC"},
    {"start", cmd_start_stop, "start/stop - Start/stop data acquisition"},
    {"stop", cmd_start_stop, "start/stop - Start/stop data acquisition"},
    {"reboot", cmd_reboot, "Reboot the device"},
    {"exit", NULL, "Disconnect from console"},
    {NULL, NULL, NULL}
};

static esp_err_t cmd_help(int socket, int argc, char** argv) {
    char response[RESPONSE_BUFFER_SIZE];
    snprintf(response, sizeof(response), 
        "\r\nEMG8x Console Commands:\r\n"
        "======================\r\n");
    telnet_send_response(socket, response);

    for (int i = 0; telnet_commands[i].cmd != NULL; i++) {
        snprintf(response, sizeof(response), 
            "%-10s - %s\r\n", telnet_commands[i].cmd, telnet_commands[i].help);
        telnet_send_response(socket, response);
    }
    telnet_send_response(socket, "\r\n");
    return ESP_OK;
}

static void execute_command(int socket, char* cmd_line) {
    char* argv[16];
    int argc = parse_command(cmd_line, argv, 16);
    
    if (argc == 0) {
        send_prompt(socket);
        return;
    }
    
    // Специальная обработка команды exit
    if (strcmp(argv[0], "exit") == 0) {
        telnet_send_response(socket, "\r\nGoodbye!\r\n");
        close(socket);
        return;
    }
    
    for (int i = 0; telnet_commands[i].cmd != NULL; i++) {
        if (strcmp(argv[0], telnet_commands[i].cmd) == 0) {
            if (telnet_commands[i].handler != NULL) {
                telnet_commands[i].handler(socket, argc, argv);
            }
            send_prompt(socket);
            return;
        }
    }
    
    char response[RESPONSE_BUFFER_SIZE];
    snprintf(response, sizeof(response), 
        "\r\nUnknown command: %s\r\nType 'help' for available commands\r\n\r\n", argv[0]);
    telnet_send_response(socket, response);
    send_prompt(socket);
}
static void handle_client_data(telnet_client_t* client, char* data, int len) {
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
        } else if (c == 27) { // ESC sequence (стрелки и т.д.)
            // Игнорируем escape последовательности
            continue;
        } else if (c >= 32 && c < 127) { // Printable characters
            if (client->cmd_pos < CMD_BUFFER_SIZE - 1) {
                client->cmd_buffer[client->cmd_pos++] = c;
                send(client->socket, &c, 1, 0);
            }
        }
        // Игнорируем все остальные символы (включая спецсимволы telnet)
    }
}

esp_err_t telnet_console_init(void) {
    memset(telnet_clients, 0, sizeof(telnet_clients));
    for (int i = 0; i < MAX_CLIENTS; i++) {
        telnet_clients[i].socket = -1;
    }
    
    ESP_LOGI(TELNET_TAG, "Telnet console initialized");
    return ESP_OK;
}

void telnet_console_task(void *pvParameter) {
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    fd_set read_fds;
    int max_fd;
    
    // Создаем серверный сокет
    telnet_server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (telnet_server_socket < 0) {
        ESP_LOGE(TELNET_TAG, "Failed to create socket");
        vTaskDelete(NULL);
        return;
    }
    
    int opt = 1;
    setsockopt(telnet_server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(TELNET_PORT);
    
    if (bind(telnet_server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TELNET_TAG, "Bind failed");
        close(telnet_server_socket);
        vTaskDelete(NULL);
        return;
    }
    
    if (listen(telnet_server_socket, MAX_CLIENTS) < 0) {
        ESP_LOGE(TELNET_TAG, "Listen failed");
        close(telnet_server_socket);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TELNET_TAG, "Telnet console listening on port %d", TELNET_PORT);
    
    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(telnet_server_socket, &read_fds);
        max_fd = telnet_server_socket;
        
        // Добавляем активные клиентские сокеты
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (telnet_clients[i].active && telnet_clients[i].socket >= 0) {
                FD_SET(telnet_clients[i].socket, &read_fds);
                if (telnet_clients[i].socket > max_fd) {
                    max_fd = telnet_clients[i].socket;
                }
            }
        }
        
        struct timeval timeout = {1, 0}; // 1 секунда таймаут
        int activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
        
        if (activity < 0) {
            ESP_LOGE(TELNET_TAG, "Select error");
            continue;
        }
        
        // Новое подключение
        if (FD_ISSET(telnet_server_socket, &read_fds)) {
            int new_socket = accept(telnet_server_socket, (struct sockaddr*)&client_addr, &client_addr_len);
            if (new_socket >= 0) {
                // Найдем свободное место для клиента
                int client_idx = -1;
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (!telnet_clients[i].active) {
                        client_idx = i;
                        break;
                    }
                }
                
                if (client_idx >= 0) {
                    telnet_clients[client_idx].socket = new_socket;
                    telnet_clients[client_idx].active = true;
                    telnet_clients[client_idx].cmd_pos = 0;
                    
                    ESP_LOGI(TELNET_TAG, "New telnet client connected: %s", 
                        inet_ntoa(client_addr.sin_addr));
                    
                    // Отправляем Telnet negotiation для отключения echo и line mode
                    char telnet_init[] = {255, 251, 1,    // IAC WILL ECHO (сервер будет делать echo)
                                         255, 251, 3,    // IAC WILL SUPPRESS-GO-AHEAD
                                         255, 253, 1,    // IAC DO ECHO (клиент не должен делать echo)
                                         255, 253, 3};   // IAC DO SUPPRESS-GO-AHEAD
                    send(new_socket, telnet_init, sizeof(telnet_init), 0);
                    
                    telnet_send_response(new_socket, 
                        "\r\nWelcome to EMG8x Console!\r\nType 'help' for available commands.\r\n");
                    send_prompt(new_socket);
                } else {
                    telnet_send_response(new_socket, 
                        "\r\nSorry, maximum clients connected.\r\n");
                    close(new_socket);
                }
            }
        }
        
        // Обработка данных от клиентов
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (telnet_clients[i].active && FD_ISSET(telnet_clients[i].socket, &read_fds)) {
                char buffer[256];
                int bytes_read = recv(telnet_clients[i].socket, buffer, sizeof(buffer) - 1, 0);
                
                if (bytes_read <= 0) {
                    ESP_LOGI(TELNET_TAG, "Client disconnected");
                    close(telnet_clients[i].socket);
                    telnet_clients[i].active = false;
                    telnet_clients[i].socket = -1;
                } else {
                    buffer[bytes_read] = '\0';
                    handle_client_data(&telnet_clients[i], buffer, bytes_read);
                }
            }
        }
    }
    
    close(telnet_server_socket);
    vTaskDelete(NULL);
}



void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..") ;
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", (int)esp_get_free_heap_size()) ;
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version()) ;

    esp_chip_info_t chip_info ;
    memset(&chip_info,0,sizeof(esp_chip_info_t)) ;
    esp_chip_info(&chip_info) ;
    switch(chip_info.model)
    {
        case CHIP_ESP32:
            ESP_LOGI(TAG, "[APP] Processor model: ESP32") ;
            break ;
        case CHIP_ESP32S2:
            ESP_LOGI(TAG, "[APP] Processor model: ESP32-S2") ;
            break ;
/*
        case CHIP_ESP32S3:
            ESP_LOGI(TAG, "[APP] Processor model: ESP32-S3") ;
            break ;
        case CHIP_ESP32C3:
            ESP_LOGI(TAG, "[APP] Processor model: ESP32-C3") ;
            break ;
*/
        default:
            ESP_LOGI(TAG, "[APP] Processor model: Unknown(%d)",(int)chip_info.model) ;
            break ;
    }
    ESP_LOGI(TAG, "[APP] Processor num cores: %d",(int)chip_info.cores) ;

    //esp_log_level_set("*", ESP_LOG_INFO);
    //esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    //esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    //esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    //esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init() ;
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      ESP_ERROR_CHECK(nvs_flash_erase()) ;
      ret = nvs_flash_init() ;
    }
    ESP_ERROR_CHECK(ret) ;

#if CONFIG_EMG8X_BOARD_REV == 4
    gpio_reset_pin( BOARD_LED1 ) ;
    gpio_reset_pin( BOARD_LED2 ) ;
    gpio_reset_pin( BOARD_LED3 ) ;

    gpio_set_direction( BOARD_LED1, GPIO_MODE_OUTPUT ) ;
    gpio_set_direction( BOARD_LED2, GPIO_MODE_OUTPUT ) ;
    gpio_set_direction( BOARD_LED3, GPIO_MODE_OUTPUT ) ;

    gpio_set_level( BOARD_LED1,     0 ) ;
    gpio_set_level( BOARD_LED2,     0 ) ;
    gpio_set_level( BOARD_LED3,     0 ) ;
#endif

    gpio_reset_pin( ESP_LED1 ) ;
    gpio_set_direction( ESP_LED1, GPIO_MODE_OUTPUT ) ;
    gpio_set_level( ESP_LED1,     0 ) ;

// #if CONFIG_WIFI_MODE_SOFTAP
//     wifi_init_softap();
// #else  
//     wifi_init();
// #endif

    init_wifi_mode_system();
    emg8x_app_start() ;
}
