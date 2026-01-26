#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PTC06_RES_VGA_640x480 = 0x00,
    PTC06_RES_QVGA_320x240 = 0x11,
} ptc06_resolution_t;

/**
 * “Compressibility” según datasheet:
 * comando 56 00 31 05 01 01 12 04 XX (default XX=0x36) :contentReference[oaicite:2]{index=2}
 *
 * No es 1:1 con “JPEG quality”, pero en práctica:
 * - valores menores suelen dar más calidad (archivo más grande)
 * - valores mayores suelen comprimir más (archivo más pequeño)
 * OJO: depende del firmware del módulo.
 */
typedef struct {
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    int baud_rate;            // recomendado 115200 (máximo del módulo)
    int uart_rx_buf_size;     // e.g. 8*1024
    int uart_tx_buf_size;     // e.g. 1*1024
    int cmd_timeout_ms;       // e.g. 2000
} ptc06_config_t;

typedef struct ptc06_t ptc06_t;

/**
 * Callback streaming: te llega el JPEG por pedacitos
 * Perfecto para mandar por WiFi, guardar a SD, etc.
 */
typedef esp_err_t (*ptc06_stream_cb_t)(const uint8_t *data, size_t len, void *user_ctx);

esp_err_t ptc06_init(ptc06_t **out_cam, const ptc06_config_t *cfg);
esp_err_t ptc06_deinit(ptc06_t *cam);

esp_err_t ptc06_reset(ptc06_t *cam);
esp_err_t ptc06_set_resolution(ptc06_t *cam, ptc06_resolution_t res);
esp_err_t ptc06_set_compressibility(ptc06_t *cam, uint8_t value);

/**
 * Captura y streamea JPEG.
 * - chunk_size recomendado: 512 (múltiplo de 8) para mantener direcciones alineadas :contentReference[oaicite:3]{index=3}
 * - cb recibe solo bytes JPEG (sin headers del protocolo)
 */
esp_err_t ptc06_capture_jpeg_stream(ptc06_t *cam,
                                   size_t chunk_size,
                                   ptc06_stream_cb_t cb,
                                   void *user_ctx,
                                   uint32_t *out_jpeg_len);

/**
 * Captura y te devuelve buffer con JPEG completo (PSRAM recomendado).
 * Esto es más cómodo pero menos “pro”.
 */
esp_err_t ptc06_capture_jpeg_to_buffer(ptc06_t *cam,
                                      uint8_t **out_buf,
                                      size_t *out_len);

#ifdef __cplusplus
}
#endif
