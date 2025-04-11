// main/wifi_handler.c
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_handler.h"
#include "common_defs.h" // For hardcoded credentials and LED commands

static const char *TAG = "WIFI_HANDLER";

// Event Group Handle defined here
EventGroupHandle_t wifi_event_group;
// static int s_retry_num = 0; // Retry counter no longer needed

// Event handler for WiFi and IP events
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    led_command_t led_cmd; // LED command variable

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        // Attempt connection when WiFi stack starts
        esp_wifi_connect();
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START: Trying to connect to AP '%s'...", APP_WIFI_SSID);
        led_cmd = LED_CMD_WIFI_CONNECTING;
        xQueueSend(led_command_queue, &led_cmd, pdMS_TO_TICKS(10));
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "WIFI_EVENT_STA_DISCONNECTED: Lost connection to AP '%s'. Retrying...", APP_WIFI_SSID);
        // --- Persistent Retry Logic ---
        // Clear the connected bit in the event group
        if (wifi_event_group)
        { // Ensure event group is created
            xEventGroupClearBits(wifi_event_group, WIFI_EVENT_BIT_CONNECTED);
        }
        // Signal connecting state to LED task
        led_cmd = LED_CMD_WIFI_CONNECTING;
        xQueueSend(led_command_queue, &led_cmd, pdMS_TO_TICKS(10));
        // Delay before attempting reconnect to avoid hammering the AP
        vTaskDelay(pdMS_TO_TICKS(APP_WIFI_RETRY_DELAY_MS));
        esp_wifi_connect(); // Attempt to reconnect indefinitely
        // --- End Persistent Retry Logic ---
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP: Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        // Set the connected bit
        if (wifi_event_group)
        {
            xEventGroupSetBits(wifi_event_group, WIFI_EVENT_BIT_CONNECTED);
        }
        // Signal connected state to LED task
        led_cmd = LED_CMD_WIFI_CONNECTED;
        xQueueSend(led_command_queue, &led_cmd, pdMS_TO_TICKS(10));
    }
}

// Initialize WiFi Station mode
esp_err_t wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
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

    // Set WiFi configuration from defines
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Common default
            .pmf_cfg = {.capable = true, .required = false},
        },
    };
    strncpy((char *)wifi_config.sta.ssid, APP_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    strncpy((char *)wifi_config.sta.password, APP_WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // Start WiFi - connection attempts handled by event loop

    ESP_LOGI(TAG, "wifi_init_sta finished. Background connection attempts started.");

    // No waiting here, main task continues. Connection status managed by event handler.
    return ESP_OK;
}