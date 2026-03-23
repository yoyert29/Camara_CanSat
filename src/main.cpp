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

// --- Función auxiliar para esperar una tecla en la consola ---
void wait_for_keypress(char target_key) {
    uint8_t cmd = 0;
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, &cmd, 1, 100 / portTICK_PERIOD_MS);
        if (len > 0 && cmd == target_key) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Evitar saturar el Watchdog
    }
}

// --- ALGORITMO: Escalado Nearest-Neighbor para compensar la distancia ---
void scale_image_down(uint8_t* src, uint8_t* dst, float scale_factor) {
    int new_w = (int)(IMAGE_WIDTH * scale_factor);
    int new_h = (int)(IMAGE_HEIGHT * scale_factor);
    
    // Calcular el offset para mantener la imagen escalada en el centro
    int offset_x = (IMAGE_WIDTH - new_w) / 2;
    int offset_y = (IMAGE_HEIGHT - new_h) / 2;

    // Llenar el fondo de negro para evitar basura en los bordes
    memset(dst, 0, IMAGE_WIDTH * IMAGE_HEIGHT * 3);

    for (int y = 0; y < new_h; y++) {
        for (int x = 0; x < new_w; x++) {
            // Mapeo inverso al píxel original
            int src_x = (int)(x / scale_factor);
            int src_y = (int)(y / scale_factor);

            // Bounds check de seguridad
            if (src_x >= IMAGE_WIDTH) src_x = IMAGE_WIDTH - 1;
            if (src_y >= IMAGE_HEIGHT) src_y = IMAGE_HEIGHT - 1;

            int src_idx = (src_y * IMAGE_WIDTH + src_x) * 3;
            int dst_idx = ((y + offset_y) * IMAGE_WIDTH + (x + offset_x)) * 3;

            dst[dst_idx] = src[src_idx];         // R
            dst[dst_idx + 1] = src[src_idx + 1]; // G
            dst[dst_idx + 2] = src[src_idx + 2]; // B
        }
    }
}

// --- ALGORITMO: Alineación de Paralaje Cero 2D ---
offset_2d_t calculate_alignment_offset(uint8_t* img_left, uint8_t* img_right) {
    long min_sad = 2147483647; 
    offset_2d_t best_offset = {0, 0};
    
    int center_x = IMAGE_WIDTH / 2;
    int center_y = IMAGE_HEIGHT / 2;
    int block_size = 64;
    int half_block = block_size / 2;
    
    // Rango de búsqueda: +/- 30 pixeles en X, +/- 15 pixeles en Y
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
        heap_caps_free(out_buffer);
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

    // Consola para debug y comandos
    uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0);

    uint8_t *jpg1 = NULL, *jpg2 = NULL;
    size_t len1 = 0, len2 = 0;
    uint8_t *rgb1 = NULL, *rgb2 = NULL;

    printf("\n--- ESP32 Sistema de Adquisicion Estereoscopica Listo ---\n");

    while (1) {
        printf("\n============================================\n");
        printf("Apunta al objeto (Aprox 20m). Presiona 'p' para la FOTO 1...\n");
        wait_for_keypress('p');
        
        printf("[1/6] Capturando Foto 1 (Posicion A)...\n");
        if (ptc06_capture_jpeg_to_buffer(cam, &jpg1, &len1) != ESP_OK) continue;
        
        printf(">> FOTO 1 LISTA. \n");
        printf(">> Ahora da un paso hacia atras y un paso lateral.\n");
        printf(">> Presiona 'p' para la FOTO 2...\n");
        wait_for_keypress('p');
        
        printf("[2/6] Capturando Foto 2 (Posicion B)...\n");
        if (ptc06_capture_jpeg_to_buffer(cam, &jpg2, &len2) != ESP_OK) {
            free(jpg1); jpg1 = NULL;
            continue;
        }

        // --- PROCESAMIENTO ---
        printf("[3/6] Decodificando JPEGs en PSRAM...\n");
        rgb1 = decode_jpeg_to_rgb(jpg1, len1);
        rgb2 = decode_jpeg_to_rgb(jpg2, len2);

        if (rgb1 && rgb2) {
            printf("[4/6] Escalando Foto 1 para compensar alejamiento...\n");
            
            // ¡IMPORTANTE! Ajusta este factor según tu prueba:
            // Si pasas de 20m a 21m, el factor es 20/21 = ~0.95
            // Si pasas de 20m a 22m, el factor es 20/22 = ~0.90
            float scale_factor = 0.95f; 
            
            uint8_t *rgb1_scaled = (uint8_t*)heap_caps_malloc(IMAGE_WIDTH * IMAGE_HEIGHT * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            
            if(rgb1_scaled) {
                scale_image_down(rgb1, rgb1_scaled, scale_factor);
                
                // Liberamos el rgb1 original y redirigimos el puntero al escalado
                heap_caps_free(rgb1);
                rgb1 = rgb1_scaled; 

                printf("[5/6] Analisis DSP: Calculando correccion 2D...\n");
                offset_2d_t offset = calculate_alignment_offset(rgb1, rgb2);
                printf("      Desplazamiento detectado: X = %d px, Y = %d px\n", offset.x, offset.y);

                printf("[6/6] Generando Anaglifo 3D...\n");
                apply_anaglyph_merge(rgb1, rgb2, offset);

                // --- TRANSMISIÓN POR CONSOLA (Raw temporal para PC) ---
                // Nota: Para la misión real, aquí deberás comprimir rgb1 a JPEG nuevamente.
                int pixel_count = IMAGE_WIDTH * IMAGE_HEIGHT;
                size_t total_raw_size = pixel_count * 3;
                printf("IMG_START:%d\n", total_raw_size);
                vTaskDelay(50 / portTICK_PERIOD_MS); 
                uart_write_bytes(UART_NUM_0, (const char*)rgb1, total_raw_size);
                printf("\nProceso exitoso. Listo para nueva prueba.\n");
            } else {
                ESP_LOGE(TAG, "Fallo al asignar memoria para escalado.");
            }
        } else {
            printf("Error critico en decodificacion.\n");
        }

        // --- LIMPIEZA RIGUROSA DE MEMORIA ---
        if (rgb1) { heap_caps_free(rgb1); rgb1 = NULL; }
        if (rgb2) { heap_caps_free(rgb2); rgb2 = NULL; }
        if (jpg1) { free(jpg1); jpg1 = NULL; }
        if (jpg2) { free(jpg2); jpg2 = NULL; }
    }
}
