#pragma once
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// Ensure these names match what main.c expects
typedef enum {
    PTC06_RES_160x120 = 0x22,
    PTC06_RES_320x240 = 0x11,
    PTC06_RES_640x480 = 0x00,
    // Add this alias so it compiles even if you use the other name
    PTC06_RES_VGA_640x480 = 0x00 
} ptc06_resolution_t;

typedef struct {
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    int baud_rate;
    int uart_rx_buf_size;
    int uart_tx_buf_size;
    int cmd_timeout_ms;
} ptc06_config_t;

typedef struct ptc06_t ptc06_t;

esp_err_t ptc06_init(ptc06_t **out_cam, const ptc06_config_t *cfg);
esp_err_t ptc06_deinit(ptc06_t *cam);
esp_err_t ptc06_reset(ptc06_t *cam);
esp_err_t ptc06_set_resolution(ptc06_t *cam, ptc06_resolution_t res);
esp_err_t ptc06_set_compressibility(ptc06_t *cam, uint8_t value);

typedef esp_err_t (*ptc06_stream_cb_t)(const uint8_t *data, size_t len, void *ctx);

esp_err_t ptc06_capture_jpeg_stream(ptc06_t *cam, size_t chunk_size, ptc06_stream_cb_t cb, void *ctx, uint32_t *out_len);
esp_err_t ptc06_capture_jpeg_to_buffer(ptc06_t *cam, uint8_t **out_buf, size_t *out_len);

#ifdef __cplusplus
}
#endif