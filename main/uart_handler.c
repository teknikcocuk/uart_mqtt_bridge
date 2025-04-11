// main/uart_handler.c
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For malloc, free
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "cJSON.h" // cJSON library for parsing

#include "uart_handler.h"
#include "common_defs.h" // For defines, structs, queue handles

static const char *TAG = "UART_HANDLER";
static QueueHandle_t local_uart_to_mqtt_queue = NULL;
static QueueHandle_t local_mqtt_to_uart_queue = NULL;
static QueueHandle_t local_led_command_queue = NULL;

// Task to read data from UART, parse JSON, and send to MQTT queue
static void uart_rx_task(void *pvParameters)
{
    uint8_t *rx_buffer = (uint8_t *)malloc(APP_UART_READ_BUF_SIZE);
    if (rx_buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate RX buffer");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "UART RX task started (Port %d).", APP_UART_NUM);

    while (1)
    {
        // Read data from the UART
        int len = uart_read_bytes(APP_UART_NUM, rx_buffer, APP_UART_READ_BUF_SIZE - 1, pdMS_TO_TICKS(100)); // Wait 100ms

        // Process received data
        if (len > 0)
        {
            rx_buffer[len] = '\0'; // Null-terminate the received data
            ESP_LOGI(TAG, "UART Received %d bytes: '%s'", len, (char *)rx_buffer);

            // Send LED command
            led_command_t led_cmd = LED_CMD_UART_RX_RECEIVED;
            xQueueSend(local_led_command_queue, &led_cmd, pdMS_TO_TICKS(10));

            // --- Parse JSON ---
            cJSON *root = cJSON_Parse((const char *)rx_buffer);
            if (root == NULL)
            {
                ESP_LOGE(TAG, "Failed to parse JSON: %s", cJSON_GetErrorPtr());
                uart_write_bytes(APP_UART_NUM, "Error: Invalid JSON\r\n", 21); // Send error back via UART
                continue;                                                      // Skip to next read if parsing failed
            }

            cJSON *topic_item = cJSON_GetObjectItem(root, "topic");
            cJSON *payload_item = cJSON_GetObjectItem(root, "payload");

            // Construct the full topic including the base
            char full_topic[128]; // Adjust size as needed
            if (!cJSON_IsString(topic_item) || (topic_item->valuestring == NULL))
            {
                ESP_LOGE(TAG, "JSON format error: 'topic' not found or not a string.");
                uart_write_bytes(APP_UART_NUM, "Error: Missing/Invalid 'topic'\r\n", 30);
                cJSON_Delete(root);
                continue;
            }
            snprintf(full_topic, sizeof(full_topic), "%s%s", APP_MQTT_PUB_BASE_TOPIC, topic_item->valuestring);

            if (!cJSON_IsString(payload_item) || (payload_item->valuestring == NULL))
            {
                ESP_LOGE(TAG, "JSON format error: 'payload' not found or not a string.");
                uart_write_bytes(APP_UART_NUM, "Error: Missing/Invalid 'payload'\r\n", 32);
                cJSON_Delete(root);
                continue;
            }

            ESP_LOGD(TAG, "JSON Parsed - Full Topic: '%s', Payload: '%s'", full_topic, payload_item->valuestring);

            // --- Prepare data for MQTT queue ---
            uart_data_t data_to_send;
            data_to_send.topic = strdup(full_topic); // Use the constructed full topic
            data_to_send.payload = strdup(payload_item->valuestring);

            if (data_to_send.topic == NULL || data_to_send.payload == NULL)
            {
                ESP_LOGE(TAG, "Failed to allocate memory for topic/payload strings!");
                if (data_to_send.topic)
                    free(data_to_send.topic);
                if (data_to_send.payload)
                    free(data_to_send.payload);
                cJSON_Delete(root);
                continue;
            }

            // Send the data structure to the MQTT queue
            if (xQueueSend(local_uart_to_mqtt_queue, &data_to_send, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to send data to MQTT queue. Freeing memory.");
                free(data_to_send.topic);
                free(data_to_send.payload);
            }
            else
            {
                ESP_LOGD(TAG, "Sent parsed data to MQTT queue.");
                uart_write_bytes(APP_UART_NUM, "OK: Sent to MQTT Queue\r\n", 24); // Send confirmation
            }

            cJSON_Delete(root);
            // --- JSON Handling End ---
        }
        else if (len < 0)
        {
            ESP_LOGE(TAG, "UART read error");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    free(rx_buffer);
    vTaskDelete(NULL);
}

// Task to receive data from MQTT queue and send via UART (Unchanged)
static void uart_tx_task(void *pvParameters)
{
    char *received_payload_ptr = NULL;
    char tx_buffer[APP_UART_READ_BUF_SIZE + 32]; // Buffer for formatted output

    ESP_LOGI(TAG, "UART TX task started.");

    while (1)
    {
        if (xQueueReceive(local_mqtt_to_uart_queue, &received_payload_ptr, portMAX_DELAY))
        {
            if (received_payload_ptr != NULL)
            {
                ESP_LOGI(TAG, "Received payload from MQTT queue: '%s'", received_payload_ptr);
                int len = snprintf(tx_buffer, sizeof(tx_buffer), "MQTT Data: %s\r\n", received_payload_ptr);
                if (len > 0)
                {
                    uart_write_bytes(APP_UART_NUM, tx_buffer, len);
                    ESP_LOGI(TAG, "Sent to UART: %s", tx_buffer);
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to format string for UART TX");
                }
                free(received_payload_ptr);
                received_payload_ptr = NULL;
            }
            else
            {
                ESP_LOGW(TAG, "Received NULL pointer from MQTT queue.");
            }
        }
    }
}

// Initialize UART and start RX/TX tasks (Unchanged Function Body)
esp_err_t uart_init_and_start_tasks(QueueHandle_t uart_to_mqtt_q, QueueHandle_t mqtt_to_uart_q, QueueHandle_t led_q)
{
    if (!uart_to_mqtt_q || !mqtt_to_uart_q || !led_q)
    {
        ESP_LOGE(TAG, "Invalid queue handles provided.");
        return ESP_ERR_INVALID_ARG;
    }
    local_uart_to_mqtt_queue = uart_to_mqtt_q;
    local_mqtt_to_uart_queue = mqtt_to_uart_q;
    local_led_command_queue = led_q;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_LOGI(TAG, "Configuring UART%d...", APP_UART_NUM);
    ESP_ERROR_CHECK(uart_param_config(APP_UART_NUM, &uart_config));
    ESP_LOGI(TAG, "Setting UART%d pins (TX:%d, RX:%d)...", APP_UART_NUM, APP_UART_TX_PIN, APP_UART_RX_PIN);
    ESP_ERROR_CHECK(uart_set_pin(APP_UART_NUM, APP_UART_TX_PIN, APP_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Installing UART%d driver...", APP_UART_NUM);
    ESP_ERROR_CHECK(uart_driver_install(APP_UART_NUM, APP_UART_RX_BUF_SIZE * 2, APP_UART_RX_BUF_SIZE * 2, 0, NULL, 0));

    BaseType_t rx_task_created = xTaskCreate(uart_rx_task, "uart_rx_task", APP_UART_TASK_STACK, NULL, 10, NULL);
    if (rx_task_created != pdPASS)
    { /* ... error handling ... */
        return ESP_FAIL;
    }

    BaseType_t tx_task_created = xTaskCreate(uart_tx_task, "uart_tx_task", APP_UART_TASK_STACK, NULL, 9, NULL);
    if (tx_task_created != pdPASS)
    { /* ... error handling ... */
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UART handler initialized and tasks started.");
    return ESP_OK;
}