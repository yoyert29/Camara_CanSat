#include "neo6m.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#define BUFFER 500

// Use UART2 for GPS
#define GPS_UART_NUM UART_NUM_2
// Choose pins for UART2 (change if your wiring is different)
#define GPS_TX_PIN   17   // ESP32 TX -> GPS RX
#define GPS_RX_PIN   16   // ESP32 RX -> GPS TX

static char buf[BUFFER];
static const char *TAG = "GPS_MODULE";

void gps_start(void)
{
    const uart_port_t uart_num = GPS_UART_NUM;

    uart_config_t uart_config = {
        .baud_rate = 9600,                 // NEO-6M default
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Configure UART2 pins
    ESP_ERROR_CHECK(uart_set_pin(
        uart_num,
        GPS_TX_PIN,          // TX
        GPS_RX_PIN,          // RX
        UART_PIN_NO_CHANGE,  // RTS
        UART_PIN_NO_CHANGE   // CTS
    ));

    ESP_ERROR_CHECK(uart_driver_install(
        uart_num,
        BUFFER,     // rx buffer size
        0,          // tx buffer size (0 = no separate buffer)
        0,          // event queue size
        NULL,       // event queue handle
        0           // flags
    ));

    ESP_LOGI(TAG, "GPS UART2 initialized on TX=%d, RX=%d at 9600", GPS_TX_PIN, GPS_RX_PIN);
}

void raw_nmea(void)
{
    memset(buf, 0, BUFFER);
    int len = uart_read_bytes(
        GPS_UART_NUM,
        (uint8_t *)buf,
        BUFFER - 1,
        portMAX_DELAY
    );

    if (len > 0) {
        buf[len] = '\0';
        // ESP_LOGI(TAG, "%s", buf);    // print NMEA line
    } else {
        ESP_LOGW(TAG, "No data read from GPS");
    }
}

const char *gps_get_last_sentence(void)
{
    return buf;
}
