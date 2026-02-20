#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "jpeg_decoder.h" 
#include "ptc06.h"        

#define CAM_TX_PIN 17
#define CAM_RX_PIN 18
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define CHANNELS 3        // RGB888

static const char *TAG = "ANAGLYPH_CORE";

// Estructura para almacenar el desplazamiento 2D
typedef struct {
    int x;
    int y;
} offset_2d_t;

// --- ALGORITMO: Alineación de Paralaje Cero 2D ---
offset_2d_t calculate_alignment_offset(uint8_t* img_left, uint8_t* img_right) {
    long min_sad = 2147483647; 
    offset_2d_t best_offset = {0, 0};
    
    int center_x = IMAGE_WIDTH / 2;
    int center_y = IMAGE_HEIGHT / 2;
    int block_size = 64;
    int half_block = block_size / 2;
    
    // Rango de búsqueda: +/- 30 pixeles en X, +/- 15 pixeles en Y (El drift vertical es menor)
    int search_range_x = 30; 
    int search_range_y = 15; 

    for (int offset_y = -search_range_y; offset_y <= search_range_y; offset_y++) {
        for (int offset_x = -search_range_x; offset_x <= search_range_x; offset_x++) {
            long current_sad = 0;

            for (int y = -half_block; y < half_block; y++) {
                for (int x = -half_block; x < half_block; x++) {
                    
                    int lx = center_x + x;
                    int ly = center_y + y;
                    
                    int rx = center_x + x + offset_x;
                    int ry = center_y + y + offset_y;

                    // Bounds check estricto
                    if (rx < 0 || rx >= IMAGE_WIDTH || ry < 0 || ry >= IMAGE_HEIGHT) continue;

                    int idx_L = (ly * IMAGE_WIDTH + lx) * 3;
                    int idx_R = (ry * IMAGE_WIDTH + rx) * 3;

                    // Comparar canal VERDE (Luminancia)
                    int diff = img_left[idx_L + 1] - img_right[idx_R + 1];
                    if(diff < 0) diff = -diff; 
                    
                    current_sad += diff;
                }
            }

            if (current_sad < min_sad) {
                min_sad = current_sad;
                best_offset.x = offset_x;
                best_offset.y = offset_y;
            }
        }
    }
    return best_offset;
}

// --- ALGORITMO: Fusión Anaglifo con corrección 2D ---
void apply_anaglyph_merge(uint8_t* img_target, uint8_t* img_right, offset_2d_t offset) {
    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            
            int idx_out = (y * IMAGE_WIDTH + x) * 3;
            
            int rx = x + offset.x;
            int ry = y + offset.y;
            
            if (rx >= 0 && rx < IMAGE_WIDTH && ry >= 0 && ry < IMAGE_HEIGHT) {
                int idx_R = (ry * IMAGE_WIDTH + rx) * 3;
                img_target[idx_out + 1] = img_right[idx_R + 1]; // Green
                img_target[idx_out + 2] = img_right[idx_R + 2]; // Blue
            } else {
                img_target[idx_out + 1] = 0;
                img_target[idx_out + 2] = 0;
            }
        }
    }
}

// --- Decodificador JPEG a PSRAM Seguro ---
uint8_t* decode_jpeg_to_rgb(uint8_t *jpeg_data, size_t jpeg_len) {
    size_t out_size = (IMAGE_WIDTH * IMAGE_HEIGHT * CHANNELS) + 1024;
    
    uint8_t *out_buffer = (uint8_t*)heap_caps_malloc(out_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!out_buffer) {
        ESP_LOGE(TAG, "FATAL: No hay suficiente PSRAM para decodificar la imagen.");
        return NULL;
    }

    esp_jpeg_image_cfg_t jpeg_cfg = {
            .indata = jpeg_data,
            .indata_size = jpeg_len,
            .outbuf = out_buffer,       
            .outbuf_size = out_size,    
            .out_format = JPEG_IMAGE_FORMAT_RGB888,
            .out_scale = JPEG_IMAGE_SCALE_0,
            .flags = { .swap_color_bytes = 1 } 
        };
    
    esp_jpeg_image_output_t outimg;
    esp_err_t ret = esp_jpeg_decode(&jpeg_cfg, &outimg);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Decodificacion JPEG fallo. Codigo: %d", ret);
        heap_caps_free(out_buffer); // Uso correcto de liberación de PSRAM
        return NULL;
    }
    return out_buffer; 
}

extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);

    // =========================================================
    // FASE 1: INICIALIZACIÓN FÍSICA Y ESCALAMIENTO A 115200 BPS
    // =========================================================
    ESP_LOGI(TAG, "Preparando camara a 38400 bps para escalamiento...");
    uart_config_t temp_uart_config = {
        .baud_rate = 38400, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &temp_uart_config);
    uart_set_pin(UART_NUM_1, CAM_TX_PIN, CAM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    vTaskDelay(pdMS_TO_TICKS(2500)); 

    uint8_t reset_cmd[] = {0x56, 0x00, 0x26, 0x00};
    uart_write_bytes(UART_NUM_1, (const char*)reset_cmd, sizeof(reset_cmd));
    vTaskDelay(pdMS_TO_TICKS(2500)); 
    uart_flush_input(UART_NUM_1); 

    uint8_t set_baud_cmd[] = {0x56, 0x00, 0x24, 0x03, 0x01, 0x0D, 0xA6};
    uart_write_bytes(UART_NUM_1, (const char*)set_baud_cmd, sizeof(set_baud_cmd));
    uint8_t rx_buf[10];
    uart_read_bytes(UART_NUM_1, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(500));

    uart_driver_delete(UART_NUM_1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // =========================================================
    // FASE 2: ENLACE DEL DRIVER LÓGICO
    // =========================================================
    ptc06_config_t cfg = {
        .uart_num = UART_NUM_1,
        .tx_pin = CAM_TX_PIN,
        .rx_pin = CAM_RX_PIN,
        .baud_rate = 115200,      // VELOCIDAD MÁXIMA
        .uart_rx_buf_size = 4096,
        .uart_tx_buf_size = 512,
        .cmd_timeout_ms = 2000
    };
    
    ptc06_t *cam = NULL;
    if (ptc06_init(&cam, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al enlazar la camara PTC06");
        return;
    }
    ptc06_set_resolution(cam, PTC06_RES_320x240);

    // Consola para debug
    uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0);

    uint8_t *jpg1 = NULL, *jpg2 = NULL;
    size_t len1 = 0, len2 = 0;
    uint8_t *rgb1 = NULL, *rgb2 = NULL;

    printf("\n--- ESP32 Sistema de Adquisicion Estereoscopica Listo ---\n");
    printf("Presiona 'p' para iniciar secuencia automatica (simulacion de vuelo).\n");

    while (1) {
        uint8_t cmd = 0;
        int len = uart_read_bytes(UART_NUM_0, &cmd, 1, 50 / portTICK_PERIOD_MS);
        
        // Simulación: Presionar 'p' dispara todo el proceso seguido
        if (len > 0 && cmd == 'p') {
            
            // --- CAPTURA RÁPIDA (El equivalente al ascenso) ---
            printf("\n[1/5] Capturando Foto 1 (Posicion A)...\n");
            if (ptc06_capture_jpeg_to_buffer(cam, &jpg1, &len1) != ESP_OK) continue;
            
            // Simular el tiempo de deriva horizontal (e.g., 3 segundos)
            printf("      Esperando desplazamiento del CanSat (3s)...\n");
            vTaskDelay(pdMS_TO_TICKS(3000));
            
            printf("[2/5] Capturando Foto 2 (Posicion B)...\n");
            if (ptc06_capture_jpeg_to_buffer(cam, &jpg2, &len2) != ESP_OK) continue;

            // --- PROCESAMIENTO ---
            printf("[3/5] Decodificando JPEGs en PSRAM...\n");
            rgb1 = decode_jpeg_to_rgb(jpg1, len1);
            rgb2 = decode_jpeg_to_rgb(jpg2, len2);

            if (rgb1 && rgb2) {
                printf("[4/5] Analisis DSP: Calculando correccion 2D...\n");
                offset_2d_t offset = calculate_alignment_offset(rgb1, rgb2);
                printf("      Desplazamiento detectado: X = %d px, Y = %d px\n", offset.x, offset.y);

                printf("[5/5] Generando Anaglifo 3D...\n");
                apply_anaglyph_merge(rgb1, rgb2, offset);

                // --- TRANSMISIÓN POR CONSOLA (Para debug local) ---
                int pixel_count = IMAGE_WIDTH * IMAGE_HEIGHT;
                size_t total_raw_size = pixel_count * 3;
                printf("IMG_START:%d\n", total_raw_size);
                vTaskDelay(50 / portTICK_PERIOD_MS); 
                uart_write_bytes(UART_NUM_0, (const char*)rgb1, total_raw_size);
                printf("\nProceso exitoso. Sistema reiniciado.\n");
            } else {
                printf("Error critico en decodificacion.\n");
            }

            // --- LIMPIEZA RIGUROSA DE MEMORIA ---
            if (rgb1) { heap_caps_free(rgb1); rgb1 = NULL; }
            if (rgb2) { heap_caps_free(rgb2); rgb2 = NULL; }
            // Asumiendo que la librería usa el malloc estándar de C
            if (jpg1) { free(jpg1); jpg1 = NULL; }
            if (jpg2) { free(jpg2); jpg2 = NULL; }
        }
    }
}
