// main/uart_handler.h
#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "esp_err.h"
#include "common_defs.h" // For queue handles

/**
 * @brief Initializes UART communication and starts the RX/TX tasks.
 *
 * @param uart_to_mqtt_q Queue to send parsed UART data to MQTT handler.
 * @param mqtt_to_uart_q Queue to receive MQTT data to be sent via UART.
 * @param led_q Handle to the LED command queue.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t uart_init_and_start_tasks(QueueHandle_t uart_to_mqtt_q, QueueHandle_t mqtt_to_uart_q, QueueHandle_t led_q);

#endif // UART_HANDLER_H