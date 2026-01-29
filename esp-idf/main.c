#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "ptc06.h"

// ==== CONFIGURATION ====
#define PC_UART_NUM      UART_NUM_0
#define PC_BAUD_RATE     115200

#define CAM_UART_NUM     UART_NUM_1
#define CAM_TX_PIN       17  // Connect to Camera RX
#define CAM_RX_PIN       18  // Connect to Camera TX
#define CAM_BAUD_RATE    38400 
// =======================

static const char *TAG = "APP";

void app_main(void)
{
    // 1. Initialize PC UART
    uart_config_t pc_uart_config = {
        .baud_rate = PC_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // FIX: actually apply the config to remove the "unused variable" warning
    uart_param_config(PC_UART_NUM, &pc_uart_config);
    uart_driver_install(PC_UART_NUM, 1024, 0, 0, NULL, 0); 
    
    // 2. Initialize Camera
    ESP_LOGI(TAG, "Initializing Camera...");
    ptc06_config_t cam_cfg = {
        .uart_num = CAM_UART_NUM,
        .tx_pin = CAM_TX_PIN,
        .rx_pin = CAM_RX_PIN,
        .baud_rate = CAM_BAUD_RATE,
        .uart_rx_buf_size = 2048,
        .uart_tx_buf_size = 512,
        .cmd_timeout_ms = 2000
    };

    ptc06_t *cam = NULL;
    esp_err_t err = ptc06_init(&cam, &cam_cfg);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed! Check wiring (TX/RX swapped?)");
        return;
    }

    // FIX: Handle different naming conventions in the header file
    #ifdef PTC06_RES_640x480
        ptc06_set_resolution(cam, PTC06_RES_640x480);
    #else
        // Fallback if your header uses the "VGA" naming convention
        ptc06_set_resolution(cam, PTC06_RES_VGA_640x480);
    #endif

    ptc06_set_compressibility(cam, 0x36); 

    ESP_LOGI(TAG, "Ready! Send 'p' to capture...");

    // 3. Main Loop
    uint8_t cmd_buf[1];
    while (1) {
        int len = uart_read_bytes(PC_UART_NUM, cmd_buf, 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            if (cmd_buf[0] == 'p' || cmd_buf[0] == 'P') {
                ESP_LOGI(TAG, "Capture command received!");

                uint8_t *jpg_buf = NULL;
                size_t jpg_len = 0;

                err = ptc06_capture_jpeg_to_buffer(cam, &jpg_buf, &jpg_len);

                if (err == ESP_OK && jpg_buf != NULL) {
                    ESP_LOGI(TAG, "Captured %d bytes. Sending...", jpg_len);

                    char header[32];
                    int h_len = snprintf(header, sizeof(header), "IMG_START:%u\n", jpg_len);
                    uart_write_bytes(PC_UART_NUM, header, h_len);
                    uart_write_bytes(PC_UART_NUM, (const char*)jpg_buf, jpg_len);
                    
                    free(jpg_buf); 
                    ESP_LOGI(TAG, "Done.");
                } else {
                    ESP_LOGE(TAG, "Capture Failed!");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
