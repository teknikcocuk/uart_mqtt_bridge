// main/common_defs.h
#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Structure to pass data from UART to MQTT Task
typedef struct
{
    char *topic;   // Dynamically allocated topic string
    char *payload; // Dynamically allocated payload string
} uart_data_t;

// Queue handles (declared here, defined and created in main.c)
extern QueueHandle_t uart_to_mqtt_queue;
extern QueueHandle_t mqtt_to_uart_queue;
extern QueueHandle_t led_command_queue;

// LED Commands Enum
typedef enum
{
    LED_CMD_OFF,
    LED_CMD_WIFI_CONNECTING, // Trying to connect WiFi
    LED_CMD_WIFI_CONNECTED,  // WiFi Connected, MQTT might be connecting/disconnected
    LED_CMD_MQTT_CONNECTED,  // WiFi and MQTT are connected
    LED_CMD_UART_RX_RECEIVED,
    LED_CMD_MQTT_RX_RECEIVED
} led_command_t;

// --- Configuration (Hardcoded - Replace with your details!) ---
// WiFi
#define APP_WIFI_SSID "gundogdu's_plus" // <<< CHANGE THIS
#define APP_WIFI_PASS "neerd3444"       // <<< CHANGE THIS
#define APP_WIFI_RETRY_DELAY_MS 1000    // Delay before retrying WiFi connection (ms)

// MQTT
#define APP_MQTT_BROKER_URI "mqtt://mqtt.eclipseprojects.io" // <<< CHANGE OR CONFIRM
#define APP_MQTT_PUB_BASE_TOPIC "pub/data/"                  // Base for publishing from UART
#define APP_MQTT_SUB_BASE_TOPIC "sub/data/"                  // Base for subscribing

// UART
#define APP_UART_NUM UART_NUM_2 // Or UART_NUM_2 if using external TTL
#define APP_UART_TX_PIN (17)    // Use default for UART0/2
#define APP_UART_RX_PIN (16)    // Use default for UART0/2
#define APP_UART_RX_BUF_SIZE (1024)
#define APP_UART_TASK_STACK (3072)   // JSON parsing needs stack
#define APP_UART_READ_BUF_SIZE (256) // Max expected JSON string length + safety

// LED
#define APP_LED_GPIO (GPIO_NUM_2) // Common built-in LED GPIO
#define APP_LED_TASK_STACK (2048)
// --- End Configuration ---

#endif // COMMON_DEFS_H