// main/wifi_handler.h
#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include "esp_err.h"
#include "freertos/event_groups.h" // For event groups

// WiFi connection status event bits
// Only need CONNECTED now, as we retry indefinitely on failure/disconnect
#define WIFI_EVENT_BIT_CONNECTED BIT0
// #define WIFI_EVENT_BIT_FAIL      BIT1 // No longer needed

// External access to the WiFi event group handle
extern EventGroupHandle_t wifi_event_group;

/**
 * @brief Initialize WiFi in Station mode and start persistent connection attempts.
 *        Uses credentials defined in common_defs.h
 *
 * @return esp_err_t ESP_OK on successful initiation, or an error code.
 *         Connection success is signaled via wifi_event_group's WIFI_EVENT_BIT_CONNECTED.
 *         The handler will keep trying to connect if disconnected.
 */
esp_err_t wifi_init_sta(void);

#endif // WIFI_HANDLER_H