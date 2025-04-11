# ESP32 UART-MQTT Bridge with LED Status Indicator

## Overview

This project implements a bridge application on an ESP32 microcontroller that facilitates communication between a UART interface and an MQTT broker. It allows data received via UART (in JSON format) to be published to specific MQTT topics and forwards messages received from a subscribed MQTT topic back to the UART interface. An LED provides visual feedback on the system's connection status (WiFi, MQTT) and data activity.

The application is built using the ESP-IDF framework and leverages FreeRTOS for task management and queues for inter-task communication.

## Features

*   **WiFi Connectivity:** Connects to a specified WiFi network in Station (STA) mode with persistent retry logic on disconnection.
*   **MQTT Client:** Connects to an MQTT broker, publishes messages, and subscribes to a device-specific topic.
*   **UART Interface:**
    *   Listens for incoming data on a configured UART port.
    *   Parses incoming data assuming a specific JSON format: `{"topic": "sub_topic", "payload": "message"}`.
    *   Sends confirmation or error messages back via UART.
    *   Outputs messages received from the subscribed MQTT topic to the UART.
*   **JSON Parsing:** Uses the cJSON library to parse UART input.
*   **MQTT Publishing:** Publishes UART data to `APP_MQTT_PUB_BASE_TOPIC` + `topic` from the JSON.
*   **MQTT Subscribing:** Subscribes to `APP_MQTT_SUB_BASE_TOPIC` + `<DEVICE_MAC_ADDRESS>`.
*   **LED Status Indicator:** Uses a GPIO-controlled LED to indicate:
    *   WiFi Connecting (Slow Blink)
    *   WiFi Connected (Brief ON then OFF - waiting for MQTT)
    *   MQTT Connected (Solid ON - fully operational)
    *   UART Data Received (Fast Double Blink)
    *   MQTT Data Received (Quick Pulse)
*   **Modular Design:** Functionality is split into handlers for WiFi, MQTT, UART, and LED.
*   **Task-Based:** Uses FreeRTOS tasks for concurrent operation (UART RX, UART TX, MQTT Publish, LED Control).
*   **Queue-Based Communication:** Tasks communicate using FreeRTOS queues for decoupling and thread safety.

## Hardware Requirements

*   ESP32 Development Board (e.g., ESP32-DevKitC, NodeMCU-32S)
*   Access Point (Router) with Internet connection for MQTT communication.
*   A computer or device capable of UART communication (e.g., using a USB-to-TTL adapter connected to the configured UART pins).
*   MQTT Broker (public or private). The default is `mqtt://mqtt.eclipseprojects.io`.
*   (Optional) An external LED and current-limiting resistor if the board's built-in LED is not on the configured `APP_LED_GPIO`.

## Software Requirements

*   **ESP-IDF:** Espressif IoT Development Framework. The project should be compatible with recent ESP-IDF versions (e.g., v4.4, v5.x). You can check the required components in `main/CMakeLists.txt`.
*   **MQTT Client Tool:** (Optional, for testing) A tool like MQTT Explorer, mosquitto_pub/sub to interact with the MQTT broker.
*   **Serial Terminal:** A program like `minicom`, `putty`, `screen`, or the `idf.py monitor` command to interact with the ESP32 via UART.

## Project Structure

```text
.
├── main
│   ├── CMakeLists.txt        # Build script for the main component
│   ├── common_defs.h         # Common definitions, configurations, queue declarations
│   ├── led_handler.c         # LED status indicator logic and task
│   ├── led_handler.h         # Header for LED handler
│   ├── main.c                # Main application entry point, initialization
│   ├── mqtt_handler.c        # MQTT connection, publishing, subscribing logic and task
│   ├── mqtt_handler.h        # Header for MQTT handler
│   ├── uart_handler.c        # UART initialization, RX/TX tasks, JSON parsing
│   ├── uart_handler.h        # Header for UART handler
│   ├── wifi_handler.c        # WiFi connection logic and event handling
│   └── wifi_handler.h        # Header for WiFi handler
└── CMakeLists.txt            # Top-level build script         
```

## Configuration

Most configuration options are located in `main/common_defs.h`:

1.  **WiFi Credentials:**
    *   `APP_WIFI_SSID`: Set your WiFi network name.
    *   `APP_WIFI_PASS`: Set your WiFi password.
2.  **MQTT Broker:**
    *   `APP_MQTT_BROKER_URI`: Set the URI of your MQTT broker (e.g., `mqtt://your_broker_ip`). Defaults to `mqtt://mqtt.eclipseprojects.io`.
3.  **MQTT Topics:**
    *   `APP_MQTT_PUB_BASE_TOPIC`: Base topic prefix for publishing data received from UART.
    *   `APP_MQTT_SUB_BASE_TOPIC`: Base topic prefix for subscribing. The device's MAC address will be appended automatically.
4.  **UART Settings:**
    *   `APP_UART_NUM`: UART port number (e.g., `UART_NUM_0`, `UART_NUM_1`, `UART_NUM_2`). Default is `UART_NUM_2`.
    *   `APP_UART_TX_PIN`: GPIO pin for UART TX. Default is `17`.
    *   `APP_UART_RX_PIN`: GPIO pin for UART RX. Default is `16`.
    *   *(Other UART buffer/stack sizes can be adjusted if needed)*
5.  **LED:**
    *   `APP_LED_GPIO`: GPIO pin connected to the status LED. Default is `GPIO_NUM_2` (often the built-in LED on DevKitC boards).

## Building and Flashing

1.  **Clone the Repository:**
    ```bash
    git clone <repository_url>
    cd <repository_directory>
    ```
2.  **Configure:** Open `main/common_defs.h` and modify the configuration values as described above (at minimum, WiFi SSID and Password).
3.  **Set ESP-IDF Target:**
    ```bash
    idf.py set-target esp32 # or esp32s2, esp32c3, etc.
    ```
4.  **Build:**
    ```bash
    idf.py build
    ```
5.  **Flash:** Connect the ESP32 board via USB. Find the correct serial port (e.g., `/dev/ttyUSB0` on Linux, `COM3` on Windows).
    ```bash
    idf.py -p /dev/ttyUSB0 flash # Replace /dev/ttyUSB0 with your port
    ```
6.  **Monitor:** View serial output and interact via UART.
    ```bash
    idf.py -p /dev/ttyUSB0 monitor
    ```
    (Press `Ctrl+]` to exit the monitor).

## Usage

1.  **Power On:** After flashing, the ESP32 will boot. Observe the LED and serial monitor output.
2.  **Connection Status:**
    *   The LED will blink slowly while trying to connect to WiFi.
    *   It will flash briefly ON then OFF when WiFi connects but MQTT is not yet connected.
    *   It will turn solid ON when both WiFi and MQTT are successfully connected.
3.  **Sending Data (UART to MQTT):**
    *   Connect to the ESP32's configured UART port using a serial terminal (Baud rate: 115200, 8N1).
    *   Send data in the following JSON format, terminated by a newline (`\n` or `\r\n`):
        ```json
        {"topic": "sensor/temperature", "payload": "25.5"}
        ```
    *   If the JSON is valid and MQTT is connected:
        *   The LED will perform a fast double blink.
        *   The ESP32 will publish the payload (`"25.5"`) to the MQTT topic `pub/data/sensor/temperature` (assuming default `APP_MQTT_PUB_BASE_TOPIC`).
        *   You will see `OK: Sent to MQTT Queue\r\n` sent back on the UART.
    *   If the JSON is invalid or MQTT is not connected:
        *   The LED will still blink (indicating UART RX).
        *   An error message (e.g., `Error: Invalid JSON\r\n`) might be sent back via UART.
        *   The message will be discarded if MQTT is not connected.
4.  **Receiving Data (MQTT to UART):**
    *   Use an MQTT client to publish a message to the topic the ESP32 subscribed to: `sub/data/<DEVICE_MAC_ADDRESS>`. You can find the device's MAC address in the initial boot logs on the serial monitor.
    *   Example publish using `mosquitto_pub`:
        ```bash
        mosquitto_pub -h mqtt.eclipseprojects.io -t "sub/data/AA:BB:CC:DD:EE:FF" -m "Turn Fan ON"
        ```
        (Replace `AA:BB:CC:DD:EE:FF` with the actual MAC address).
    *   If the message is received:
        *   The LED will perform a quick single pulse.
        *   The following text will appear on the ESP32's serial terminal:
            ```
            MQTT Data: Turn Fan ON

            ```

## How it Works Internally

1.  **Initialization (`main.c`)**: Sets up NVS, networking, creates FreeRTOS queues (`uart_to_mqtt_queue`, `mqtt_to_uart_queue`, `led_command_queue`), and initializes/starts the handler modules (LED, WiFi, MQTT, UART).
2.  **WiFi (`wifi_handler.c`)**: Manages the WiFi connection state using the ESP-IDF event loop. Sends status commands (`LED_CMD_WIFI_CONNECTING`, `LED_CMD_WIFI_CONNECTED`) to the `led_command_queue`. Retries connection indefinitely upon disconnection.
3.  **MQTT (`mqtt_handler.c`)**: Initializes the MQTT client. The client automatically attempts connection when the network (WiFi) is up.
    *   **Publishing**: An `mqtt_publish_task` waits for `uart_data_t` items on the `uart_to_mqtt_queue`. If connected (checked using a mutex and flag), it publishes the data using `esp_mqtt_client_publish`. Frees the memory allocated by the UART task.
    *   **Subscribing**: Upon connection (`MQTT_EVENT_CONNECTED`), it subscribes to the device-specific topic.
    *   **Receiving**: The `mqtt_event_handler_cb` handles incoming messages (`MQTT_EVENT_DATA`). It allocates memory for the payload, sends a pointer to it via `mqtt_to_uart_queue`, and sends `LED_CMD_MQTT_RX_RECEIVED` to the `led_command_queue`.
    *   **State**: Manages connection state (`mqtt_connected_flag` protected by `mqtt_state_mutex`) and sends relevant commands (`LED_CMD_MQTT_CONNECTED`, `LED_CMD_WIFI_CONNECTED`) to the `led_command_queue`.
4.  **UART (`uart_handler.c`)**:
    *   **RX Task (`uart_rx_task`)**: Reads bytes from UART, tries to parse them as JSON. If successful, it dynamically allocates memory for the topic and payload strings, constructs a `uart_data_t` struct, and sends it to `uart_to_mqtt_queue`. Sends `LED_CMD_UART_RX_RECEIVED` to the `led_command_queue`. Sends confirmation/error back via UART TX.
    *   **TX Task (`uart_tx_task`)**: Waits for character pointers (payloads) on the `mqtt_to_uart_queue`. Formats the string and writes it to the UART. Frees the memory allocated by the MQTT task.
5.  **LED (`led_handler.c`)**: A dedicated task (`led_control_task`) waits for commands on the `led_command_queue` and controls the LED GPIO pin according to the received command, performing different blinking/solid patterns.