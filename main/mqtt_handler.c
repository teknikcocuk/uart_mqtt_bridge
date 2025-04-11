// main/mqtt_handler.c
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h> // For malloc, free
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h" // For MAC address

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // For mutex
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "mqtt_handler.h"
#include "common_defs.h"

static const char *TAG = "MQTT_HANDLER";

static esp_mqtt_client_handle_t client = NULL;
static QueueHandle_t local_uart_to_mqtt_queue = NULL;
static QueueHandle_t local_mqtt_to_uart_queue = NULL;
static QueueHandle_t local_led_command_queue = NULL;
static char mac_address_str[18] = {0};

// --- State Tracking for MQTT Connection ---
static bool volatile mqtt_connected_flag = false; // Flag to indicate MQTT connection status
static SemaphoreHandle_t mqtt_state_mutex = NULL; // Mutex to protect the flag
// ---

// Task to handle publishing messages received from the UART queue
static void mqtt_publish_task(void *pvParameters)
{
    uart_data_t received_data;
    bool can_publish = false;

    ESP_LOGI(TAG, "MQTT Publish Task Started.");

    while (1)
    {
        // Wait indefinitely for data from the UART handler queue
        if (xQueueReceive(local_uart_to_mqtt_queue, &received_data, portMAX_DELAY))
        {
            if (received_data.topic == NULL || received_data.payload == NULL)
            {
                ESP_LOGE(TAG, "Received invalid data structure from UART queue.");
                if (received_data.topic)
                    free(received_data.topic);
                if (received_data.payload)
                    free(received_data.payload);
                continue;
            }

            // --- Check MQTT Connection State Before Publishing ---
            can_publish = false; // Assume cannot publish initially
            if (mqtt_state_mutex != NULL)
            {
                if (xSemaphoreTake(mqtt_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                { // Wait briefly for mutex
                    if (client != NULL && mqtt_connected_flag)
                    { // Check client handle and flag
                        can_publish = true;
                    }
                    xSemaphoreGive(mqtt_state_mutex);
                }
                else
                {
                    ESP_LOGE(TAG, "Could not obtain MQTT state mutex to check connection.");
                }
            }
            else
            {
                ESP_LOGE(TAG, "MQTT state mutex not initialized!");
            }
            // --- End Check ---

            if (can_publish)
            {
                ESP_LOGI(TAG, "Publishing - Topic: '%s', Payload: '%s'", received_data.topic, received_data.payload);
                int msg_id = esp_mqtt_client_publish(client,
                                                     received_data.topic,
                                                     received_data.payload,
                                                     strlen(received_data.payload),
                                                     1,  // QoS Level 1
                                                     0); // Retain flag 0
                if (msg_id != -1)
                {
                    ESP_LOGI(TAG, "Publish successful, msg_id=%d", msg_id);
                }
                else
                {
                    ESP_LOGE(TAG, "Publish failed!");
                    // Note: Client might retry automatically depending on QoS and settings
                }
            }
            else
            {
                ESP_LOGW(TAG, "MQTT not connected. Discarding message for topic: %s", received_data.topic);
                // Message is discarded if MQTT is not connected
            }

            // Free the dynamically allocated memory regardless of publish success/failure
            free(received_data.topic);
            free(received_data.payload);
        }
    }
}

// Callback function for MQTT events
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client_local = event->client;
    int msg_id;
    led_command_t led_cmd;

    switch (event->event_id)
    {
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
        // Optionally, update state mutex here if needed for fine-grained control
        break;
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED to %s", APP_MQTT_BROKER_URI);

        // --- Update Connection State ---
        if (mqtt_state_mutex != NULL)
            xSemaphoreTake(mqtt_state_mutex, portMAX_DELAY);
        mqtt_connected_flag = true;
        if (mqtt_state_mutex != NULL)
            xSemaphoreGive(mqtt_state_mutex);
        // ---

        led_cmd = LED_CMD_MQTT_CONNECTED;
        xQueueSend(local_led_command_queue, &led_cmd, pdMS_TO_TICKS(10));

        // Subscribe to device-specific topic
        char sub_topic[64];
        snprintf(sub_topic, sizeof(sub_topic), "%s%s", APP_MQTT_SUB_BASE_TOPIC, mac_address_str);
        ESP_LOGI(TAG, "Subscribing to topic: %s", sub_topic);
        msg_id = esp_mqtt_client_subscribe(client_local, sub_topic, 1);
        if (msg_id != -1)
        {
            ESP_LOGI(TAG, "Sent subscribe successful, msg_id=%d", msg_id);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to send subscribe request for %s", sub_topic);
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED");

        // --- Update Connection State ---
        if (mqtt_state_mutex != NULL)
            xSemaphoreTake(mqtt_state_mutex, portMAX_DELAY);
        mqtt_connected_flag = false;
        if (mqtt_state_mutex != NULL)
            xSemaphoreGive(mqtt_state_mutex);
        // ---

        // Indicate loss of MQTT connection via LED (back to WiFi state)
        led_cmd = LED_CMD_WIFI_CONNECTED;
        xQueueSend(local_led_command_queue, &led_cmd, pdMS_TO_TICKS(10));
        break;

    // Handle SUBSCRIBED, UNSUBSCRIBED, PUBLISHED as before (logging)
    case MQTT_EVENT_SUBSCRIBED: /* ... */
        break;
    case MQTT_EVENT_UNSUBSCRIBED: /* ... */
        break;
    case MQTT_EVENT_PUBLISHED: /* ... */
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        ESP_LOGI(TAG, "Topic: %.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "Data: %.*s", event->data_len, event->data);

        led_cmd = LED_CMD_MQTT_RX_RECEIVED;
        xQueueSend(local_led_command_queue, &led_cmd, pdMS_TO_TICKS(10));

        // Send data to UART handler via queue (allocate memory)
        char expected_sub_topic[64];
        snprintf(expected_sub_topic, sizeof(expected_sub_topic), "%s%s", APP_MQTT_SUB_BASE_TOPIC, mac_address_str);
        if (event->topic_len == strlen(expected_sub_topic) &&
            strncmp(event->topic, expected_sub_topic, event->topic_len) == 0)
        {
            char *payload_copy = malloc(event->data_len + 1);
            if (payload_copy)
            {
                memcpy(payload_copy, event->data, event->data_len);
                payload_copy[event->data_len] = '\0';
                if (xQueueSend(local_mqtt_to_uart_queue, &payload_copy, pdMS_TO_TICKS(50)) != pdPASS)
                {
                    ESP_LOGE(TAG, "Failed to send received MQTT data to UART queue. Freeing memory.");
                    free(payload_copy);
                }
                else
                {
                    ESP_LOGD(TAG, "Sent received payload pointer to UART queue.");
                }
            }
            else
            {
                ESP_LOGE(TAG, "Failed to allocate memory for MQTT payload copy!");
            }
        }
        else
        {
            ESP_LOGW(TAG, "Received data on unexpected topic: %.*s", event->topic_len, event->topic);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        // Update connection state on error? Depends on error type.
        // For simplicity, we primarily rely on DISCONNECTED event.
        if (mqtt_state_mutex != NULL)
            xSemaphoreTake(mqtt_state_mutex, portMAX_DELAY);
        mqtt_connected_flag = false; // Assume disconnected on error
        if (mqtt_state_mutex != NULL)
            xSemaphoreGive(mqtt_state_mutex);
        led_cmd = LED_CMD_WIFI_CONNECTED; // Revert LED
        xQueueSend(local_led_command_queue, &led_cmd, pdMS_TO_TICKS(10));
        break;
    default:
        ESP_LOGI(TAG, "Other MQTT event id:%d", (int)event->event_id);
        break;
    }
    return ESP_OK;
}

// Event handler wrapper (Unchanged)
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ld", base, (long)event_id);
    mqtt_event_handler_cb(event_data);
}

// Get MAC address string (Unchanged)
static void get_mac_address_str()
{
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(mac_address_str, sizeof(mac_address_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Device MAC Address: %s", mac_address_str);
}

// Initialize and start MQTT client
esp_err_t mqtt_app_start(QueueHandle_t uart_queue, QueueHandle_t mqtt_to_uart_q, QueueHandle_t led_q)
{
    if (!uart_queue || !mqtt_to_uart_q || !led_q)
    { /* ... error handling ... */
        return ESP_ERR_INVALID_ARG;
    }
    local_uart_to_mqtt_queue = uart_queue;
    local_mqtt_to_uart_queue = mqtt_to_uart_q;
    local_led_command_queue = led_q;

    // --- Create Mutex for MQTT state ---
    mqtt_state_mutex = xSemaphoreCreateMutex();
    if (mqtt_state_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create MQTT state mutex!");
        return ESP_FAIL;
    }
    // ---

    get_mac_address_str();

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = APP_MQTT_BROKER_URI,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL)
    { /* ... error handling ... */
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client)); // Start client attempts

    // Create the publishing task
    BaseType_t task_created = xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 3072, NULL, 6, NULL);
    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create MQTT publish task");
        esp_mqtt_client_destroy(client);
        client = NULL;
        vSemaphoreDelete(mqtt_state_mutex);
        mqtt_state_mutex = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MQTT client initialized and task started.");
    return ESP_OK;
}