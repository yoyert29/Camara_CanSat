#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "ptc06.h"

#define PC_UART      UART_NUM_0
#define CAM_UART     UART_NUM_1

#define CAM_TX_PIN   17
#define CAM_RX_PIN   18

static void pc_send_line(const char *s) {
    uart_write_bytes(PC_UART, s, strlen(s));
    uart_write_bytes(PC_UART, "\r\n", 2);
}

static esp_err_t jpeg_to_pc_cb(const uint8_t *data, size_t len, void *user_ctx) {
    (void)user_ctx;
    int w = uart_write_bytes(PC_UART, (const char *)data, len);
    if (w < 0 || (size_t)w != len) return ESP_FAIL;
    return ESP_OK;
}

static void pc_send_img_start(uint32_t jpg_len) {
    char line[64];
    int n = snprintf(line, sizeof(line), "IMG_START:%lu\r\n", (unsigned long)jpg_len);
    uart_write_bytes(PC_UART, line, n);
}

void app_main(void) {
    // Apaga logs para no contaminar el binario del JPG
    esp_log_level_set("*", ESP_LOG_NONE);

    // UART0 hacia PC
    uart_config_t pc_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_param_config(PC_UART, &pc_cfg);
    uart_driver_install(PC_UART, 4 * 1024, 0, 0, NULL, 0);

    pc_send_line("READY_FOR_COMMAND");

    // Init cámara PTC06
    ptc06_t *cam = NULL;
    ptc06_config_t cam_cfg = {
        .uart_num = CAM_UART,
        .tx_pin = CAM_TX_PIN,
        .rx_pin = CAM_RX_PIN,
        .baud_rate = 115200,
        .uart_rx_buf_size = 8 * 1024,
        .uart_tx_buf_size = 1 * 1024,
        .cmd_timeout_ms = 2000,
    };

    if (ptc06_init(&cam, &cam_cfg) != ESP_OK) {
        pc_send_line("Error: Camera Init Failed");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Setup recomendado
    ptc06_reset(cam);
    ptc06_set_resolution(cam, PTC06_RES_QVGA_320x240);
    ptc06_set_compressibility(cam, 0x36);

    while (1) {
        uint8_t ch;
        int r = uart_read_bytes(PC_UART, &ch, 1, pdMS_TO_TICKS(50));
        if (r <= 0) continue;

        if (ch == 'p' || ch == 'P') {
            pc_send_line("Starting_Capture...");

            uint32_t jpg_len = 0;

            // 🚀 chunk grande = menos overhead = más rápido
            // Siempre múltiplo de 8
            size_t chunk_size = 512;

            // Primero capturamos streameando.
            // OJO: El driver obtiene jpg_len INTERNAMENTE, pero nosotros necesitamos mandarlo antes del binario.
            //
            // Solución: llamamos una versión “stream” que nos devuelve out_jpeg_len,
            // y justo antes del primer chunk, mandamos IMG_START.
            //
            // Para eso vamos a hacer la captura en 2 pasos:
            //   - Capturar + obtener length (ya lo hace capture_jpeg_stream)
            //   - Pero el callback se dispara después de saber length, así que podemos mandar IMG_START antes.

            // Truco simple: mandamos IMG_START justo ANTES de llamar al stream,
            // pero necesitamos el length primero. Entonces, hacemos una captura a buffer pequeño NO,
            // mejor: en el driver (ptc06_capture_jpeg_stream) ya te da jpg_len en out_jpeg_len AL INICIO.
            //
            // En la práctica, aquí hacemos: llamar -> devuelve jpg_len -> ya está enviando bytes.
            // Eso no sirve para el protocolo Python.
            //
            // ✅ Solución real: crear una función que capture length primero y luego stream.
            // Para mantenerlo simple, hacemos aquí el “mini driver” de length antes del stream usando la API:
            // te dejo la forma más limpia: usa ptc06_capture_jpeg_to_buffer (PSRAM) y envía.
            //
            // Como tu ESP32-S3 TIENE PSRAM, esto es súper práctico y “sencillo”, como pediste.

            uint8_t *jpg_buf = NULL;
            size_t jpg_size = 0;

            if (ptc06_capture_jpeg_to_buffer(cam, &jpg_buf, &jpg_size) != ESP_OK) {
                pc_send_line("Error: Capture Failed");
                continue;
            }

            // Ahora sí: protocolo idéntico a Arduino
            pc_send_img_start((uint32_t)jpg_size);

            // Mandar binario crudo
            uart_write_bytes(PC_UART, (const char *)jpg_buf, jpg_size);

            // FIN: ya puedes imprimir sin romper la imagen
            pc_send_line("Capture_Finished");

            free(jpg_buf);
        }
    }
}
