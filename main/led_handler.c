// main/led_handler.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_handler.h" // Include own header

static const char *TAG = "LED_HANDLER";

// Task function to control the LED based on commands received via queue
static void led_control_task(void *pvParameters)
{
    QueueHandle_t cmd_queue = (QueueHandle_t)pvParameters;
    led_command_t received_cmd;
    uint8_t current_led_state_indicator = LED_CMD_OFF; // Track logical state

    ESP_LOGI(TAG, "LED control task started.");

    while (1)
    {
        // Wait indefinitely for a command
        if (xQueueReceive(cmd_queue, &received_cmd, portMAX_DELAY))
        {
            ESP_LOGD(TAG, "Received LED command: %d", received_cmd);

            // --- Execute LED Animation ---
            gpio_set_level(APP_LED_GPIO, 0); // Turn off before starting new animation (optional)

            switch (received_cmd)
            {
            case LED_CMD_WIFI_CONNECTING:
                current_led_state_indicator = received_cmd;
                // Slow blink continuously while connecting
                // Simple blocking example - for true continuous, make task non-blocking
                ESP_LOGD(TAG, "LED: WiFi Connecting Blink");
                for (int i = 0; i < 2; ++i)
                { // Just show a couple of blinks as indication
                    gpio_set_level(APP_LED_GPIO, 1);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    gpio_set_level(APP_LED_GPIO, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    if (uxQueueMessagesWaiting(cmd_queue) > 0)
                        break; // Prioritize new commands
                }
                break;

            case LED_CMD_WIFI_CONNECTED:
                current_led_state_indicator = received_cmd;
                // Solid ON briefly then off (indicates WiFi OK, MQTT state pending)
                ESP_LOGD(TAG, "LED: WiFi Connected Indication");
                gpio_set_level(APP_LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(APP_LED_GPIO, 0);
                break;

            case LED_CMD_MQTT_CONNECTED:
                current_led_state_indicator = received_cmd;
                // Solid ON (indicates fully operational)
                ESP_LOGD(TAG, "LED: MQTT Connected - Solid ON");
                gpio_set_level(APP_LED_GPIO, 1);
                // Keep it ON - next state change will turn it off/change it
                break;

            case LED_CMD_UART_RX_RECEIVED:
                // Fast double blink, then return to previous state
                ESP_LOGD(TAG, "LED: UART RX Blink");
                bool prev_state_on = (current_led_state_indicator == LED_CMD_MQTT_CONNECTED);
                if (prev_state_on)
                    gpio_set_level(APP_LED_GPIO, 0); // Turn off if it was solid on
                for (int i = 0; i < 2; i++)
                {
                    gpio_set_level(APP_LED_GPIO, 1);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    gpio_set_level(APP_LED_GPIO, 0);
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                if (prev_state_on)
                    gpio_set_level(APP_LED_GPIO, 1); // Restore solid ON state
                break;

            case LED_CMD_MQTT_RX_RECEIVED:
                // Quick pulse, then return to previous state
                ESP_LOGD(TAG, "LED: MQTT RX Blink");
                prev_state_on = (current_led_state_indicator == LED_CMD_MQTT_CONNECTED);
                if (prev_state_on)
                    gpio_set_level(APP_LED_GPIO, 0); // Turn off if it was solid on
                gpio_set_level(APP_LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(150));
                gpio_set_level(APP_LED_GPIO, 0);
                if (prev_state_on)
                {                                  // Restore solid ON state
                    vTaskDelay(pdMS_TO_TICKS(50)); // Small gap
                    gpio_set_level(APP_LED_GPIO, 1);
                }
                break;

            case LED_CMD_OFF:
            default:
                ESP_LOGD(TAG, "LED: Turning OFF");
                current_led_state_indicator = LED_CMD_OFF;
                gpio_set_level(APP_LED_GPIO, 0); // Turn off
                break;
            }
        }
    }
}

// Initialize LED GPIO and start the control task (Unchanged Function Body)
esp_err_t led_init_and_start_task(QueueHandle_t cmd_queue)
{
    if (cmd_queue == NULL)
    {
        ESP_LOGE(TAG, "Command queue handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing LED GPIO %d", APP_LED_GPIO);
    gpio_reset_pin(APP_LED_GPIO);
    esp_err_t ret = gpio_set_direction(APP_LED_GPIO, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set GPIO direction (%s)", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(APP_LED_GPIO, 0); // Start with LED off

    // Create the LED control task
    BaseType_t task_created = xTaskCreate(led_control_task,
                                          "led_control_task",
                                          APP_LED_TASK_STACK,
                                          (void *)cmd_queue, // Pass queue handle as parameter
                                          5,                 // Task priority
                                          NULL);             // Task handle (optional)

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create LED control task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LED handler initialized and task started.");
    return ESP_OK;
}