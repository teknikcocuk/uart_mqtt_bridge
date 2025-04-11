// main/main.c
#include <stdio.h>
#include <inttypes.h> // For PRIu32 macro
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h" // Still needed for wifi_handler internal use
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h" // For esp_get_free_heap_size() etc.

// Include module headers
#include "common_defs.h"
#include "wifi_handler.h"
#include "mqtt_handler.h"
#include "uart_handler.h"
#include "led_handler.h"

static const char *TAG = "MAIN_APP";

// Define queue handles (declared as extern in common_defs.h)
QueueHandle_t uart_to_mqtt_queue = NULL;
QueueHandle_t mqtt_to_uart_queue = NULL;
QueueHandle_t led_command_queue = NULL;

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    // Set log levels (optional)
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_HANDLER", ESP_LOG_VERBOSE);
    esp_log_level_set("WIFI_HANDLER", ESP_LOG_VERBOSE);
    esp_log_level_set("UART_HANDLER", ESP_LOG_VERBOSE);
    esp_log_level_set("LED_HANDLER", ESP_LOG_VERBOSE);
    esp_log_level_set("MAIN_APP", ESP_LOG_INFO); // Keep main less verbose

    // --- Initialize NVS ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // --- Initialize TCP/IP stack and default event loop ---
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // --- Create Queues ---
    ESP_LOGI(TAG, "Creating FreeRTOS Queues...");
    uart_to_mqtt_queue = xQueueCreate(10, sizeof(uart_data_t));
    mqtt_to_uart_queue = xQueueCreate(10, sizeof(char *));
    led_command_queue = xQueueCreate(15, sizeof(led_command_t));
    if (!uart_to_mqtt_queue || !mqtt_to_uart_queue || !led_command_queue)
    {
        ESP_LOGE(TAG, "Failed to create queues!");
        return; // Fatal error
    }

    // --- Initialize LED Handler ---
    ESP_LOGI(TAG, "Initializing LED Handler...");
    ret = led_init_and_start_task(led_command_queue);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LED handler!");
    }

    // --- Initialize WiFi (Starts persistent connection attempts) ---
    ESP_LOGI(TAG, "Initializing WiFi...");
    ret = wifi_init_sta();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize WiFi stack! Halting.");
        return; // Cannot continue without WiFi stack for MQTT
    }
    // NOTE: We no longer wait here for connection using xEventGroupWaitBits.
    // WiFi handler will keep trying in the background.

    // --- Initialize MQTT Client ---
    // MQTT client will internally wait for network connection before connecting to broker
    ESP_LOGI(TAG, "Initializing MQTT Client...");
    ret = mqtt_app_start(uart_to_mqtt_queue, mqtt_to_uart_queue, led_command_queue);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start MQTT client! UART publishing might not work.");
        // We can still continue to allow UART RX/TX for local tasks if needed
    }

    // --- Initialize UART Handler ---
    // UART can be started regardless of network status
    ESP_LOGI(TAG, "Initializing UART Handler...");
    ret = uart_init_and_start_tasks(uart_to_mqtt_queue, mqtt_to_uart_queue, led_command_queue);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize UART handler!");
    }

    ESP_LOGI(TAG, "Main task finished initialization. Background tasks running.");
    // The app_main task can now idle or be deleted if not needed further.
    // Example: Allow this task to monitor heap occasionally
    // while(1) {
    //     vTaskDelay(pdMS_TO_TICKS(30000)); // Check every 30 seconds
    //     ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    // }
}