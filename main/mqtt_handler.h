// main/mqtt_handler.h
#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include "esp_err.h"
#include "common_defs.h" // For queue handles

/**
 * @brief Initializes and starts the MQTT client.
 *        Requires WiFi to be connected. Attempts to connect to the broker.
 *        Starts a task to handle publishing messages from the UART queue.
 *
 * @param uart_queue Handle to the queue receiving data from UART.
 * @param mqtt_to_uart_q Handle to the queue to send received MQTT data to UART handler.
 * @param led_q Handle to the LED command queue.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t mqtt_app_start(QueueHandle_t uart_queue, QueueHandle_t mqtt_to_uart_q, QueueHandle_t led_q);

#endif // MQTT_HANDLER_H