#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "jpeg_decoder.h" 
#include "ptc06.h"        
#include "jpge.h"         

#define CAM_TX_PIN 4
#define CAM_RX_PIN 5
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
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- ALGORITMO: Escalado Nearest-Neighbor ---
void scale_image_down(uint8_t* src, uint8_t* dst, float scale_factor) {
    int new_w = (int)(IMAGE_WIDTH * scale_factor);
    int new_h = (int)(IMAGE_HEIGHT * scale_factor);
    int offset_x = (IMAGE_WIDTH - new_w) / 2;
    int offset_y = (IMAGE_HEIGHT - new_h) / 2;
    memset(dst, 0, IMAGE_WIDTH * IMAGE_HEIGHT * 3);

    for (int y = 0; y < new_h; y++) {
        for (int x = 0; x < new_w; x++) {
            int src_x = (int)(x / scale_factor);
            int src_y = (int)(y / scale_factor);
            if (src_x >= IMAGE_WIDTH) src_x = IMAGE_WIDTH - 1;
            if (src_y >= IMAGE_HEIGHT) src_y = IMAGE_HEIGHT - 1;
            int src_idx = (src_y * IMAGE_WIDTH + src_x) * 3;
            int dst_idx = ((y + offset_y) * IMAGE_WIDTH + (x + offset_x)) * 3;
            dst[dst_idx] = src[src_idx];         
            dst[dst_idx + 1] = src[src_idx + 1]; 
            dst[dst_idx + 2] = src[src_idx + 2]; 
        }
    }
}

// --- ALGORITMO: Alineación de Paralaje ---
offset_2d_t calculate_alignment_offset(uint8_t* img_left, uint8_t* img_right) {
    long min_sad = 2147483647; 
    offset_2d_t best_offset = {0, 0};
    int center_x = IMAGE_WIDTH / 2;
    int center_y = IMAGE_HEIGHT / 2;
    int half_block = 64 / 2;
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
                    if (rx < 0 || rx >= IMAGE_WIDTH || ry < 0 || ry >= IMAGE_HEIGHT) continue;
                    int idx_L = (ly * IMAGE_WIDTH + lx) * 3;
                    int idx_R = (ry * IMAGE_WIDTH + rx) * 3;
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

// --- ALGORITMO: Fusión Anaglifo ---
void apply_anaglyph_merge(uint8_t* img_target, uint8_t* img_right, offset_2d_t offset) {
    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            int idx_out = (y * IMAGE_WIDTH + x) * 3;
            int rx = x + offset.x;
            int ry = y + offset.y;
            if (rx >= 0 && rx < IMAGE_WIDTH && ry >= 0 && ry < IMAGE_HEIGHT) {
                int idx_R = (ry * IMAGE_WIDTH + rx) * 3;
                img_target[idx_out + 1] = img_right[idx_R + 1]; 
                img_target[idx_out + 2] = img_right[idx_R + 2]; 
            } else {
                img_target[idx_out + 1] = 0;
                img_target[idx_out + 2] = 0;
            }
        }
    }
}

// --- Decodificador JPEG a PSRAM ---
uint8_t* decode_jpeg_to_rgb(uint8_t *jpeg_data, size_t jpeg_len) {
    size_t out_size = (IMAGE_WIDTH * IMAGE_HEIGHT * CHANNELS) + 1024;
    uint8_t *out_buffer = (uint8_t*)heap_caps_malloc(out_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!out_buffer) return NULL;

    esp_jpeg_image_cfg_t jpeg_cfg = {
            .indata = jpeg_data, .indata_size = jpeg_len,
            .outbuf = out_buffer, .outbuf_size = out_size,    
            .out_format = JPEG_IMAGE_FORMAT_RGB888, .out_scale = JPEG_IMAGE_SCALE_0,
            .flags = { .swap_color_bytes = 1 } 
        };
    esp_jpeg_image_output_t outimg;
    if (esp_jpeg_decode(&jpeg_cfg, &outimg) != ESP_OK) {
        heap_caps_free(out_buffer);
        return NULL;
    }
    return out_buffer; 
}

// --- Codificador RGB a JPEG en PSRAM ---
uint8_t* encode_rgb_to_jpeg(uint8_t* rgb_data, int quality, size_t* out_size) {
    int max_jpeg_size = IMAGE_WIDTH * IMAGE_HEIGHT * CHANNELS;
    uint8_t* jpeg_buffer = (uint8_t*)heap_caps_malloc(max_jpeg_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    
    if (!jpeg_buffer) {
        ESP_LOGE(TAG, "FATAL: No hay PSRAM para comprimir a JPEG.");
        return NULL;
    }

    jpge::params params;
    params.m_quality = quality; 
    params.m_subsampling = jpge::H2V2; 
    params.m_two_pass_flag = false;

    int final_size = max_jpeg_size;
    
    bool success = jpge::compress_image_to_jpeg_file_in_memory(
        jpeg_buffer, final_size, IMAGE_WIDTH, IMAGE_HEIGHT, CHANNELS, rgb_data, params);

    if (!success) {
        ESP_LOGE(TAG, "Error durante la compresión a JPEG.");
        heap_caps_free(jpeg_buffer);
        return NULL;
    }

    *out_size = final_size;
    return jpeg_buffer;
}

// =========================================================
// NUEVO: TAREA PRINCIPAL DE LA MISIÓN (STACK GIGANTE)
// =========================================================
void cansat_vision_task(void *pvParameters) {
    ptc06_t *cam = (ptc06_t *)pvParameters; // Recibimos la cámara inicializada
    
    uint8_t *jpg1 = NULL, *jpg2 = NULL;
    size_t len1 = 0, len2 = 0;
    uint8_t *rgb1 = NULL, *rgb2 = NULL;

    while (1) {
        printf("\n============================================\n");
        printf("Apunta al objeto. Presiona 'p' para la FOTO 1...\n");
        wait_for_keypress('p');
        
        printf("[1/7] Capturando Foto 1 (Posicion A)...\n");
        if (ptc06_capture_jpeg_to_buffer(cam, &jpg1, &len1) != ESP_OK) continue;
        
        printf(">> FOTO 1 LISTA. \n");
        printf(">> Ahora da un paso hacia atras y un paso lateral.\n");
        printf(">> Presiona 'p' para la FOTO 2...\n");
        wait_for_keypress('p');
        
        printf("[2/7] Capturando Foto 2 (Posicion B)...\n");
        if (ptc06_capture_jpeg_to_buffer(cam, &jpg2, &len2) != ESP_OK) {
            free(jpg1); jpg1 = NULL;
            continue;
        }

        printf("[3/7] Decodificando JPEGs en PSRAM...\n");
        rgb1 = decode_jpeg_to_rgb(jpg1, len1);
        rgb2 = decode_jpeg_to_rgb(jpg2, len2);

        if (rgb1 && rgb2) {
            printf("[4/7] Escalando Foto 1 para compensar alejamiento...\n");
            float scale_factor = 0.95f; 
            uint8_t *rgb1_scaled = (uint8_t*)heap_caps_malloc(IMAGE_WIDTH * IMAGE_HEIGHT * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            
            if(rgb1_scaled) {
                scale_image_down(rgb1, rgb1_scaled, scale_factor);
                heap_caps_free(rgb1);
                rgb1 = rgb1_scaled; 

                printf("[5/7] Analisis DSP: Calculando correccion 2D...\n");
                offset_2d_t offset = calculate_alignment_offset(rgb1, rgb2);
                printf("      Desplazamiento detectado: X = %d px, Y = %d px\n", offset.x, offset.y);

                printf("[6/7] Generando Anaglifo 3D en RAW...\n");
                apply_anaglyph_merge(rgb1, rgb2, offset);

                printf("[7/7] Comprimiendo Anaglifo a JPEG...\n");
                size_t anaglyph_jpeg_size = 0;
                uint8_t* anaglyph_jpeg = encode_rgb_to_jpeg(rgb1, 70, &anaglyph_jpeg_size);

                if (anaglyph_jpeg) {
                    printf("IMG_START:%d\n", (int)anaglyph_jpeg_size);
                    vTaskDelay(50 / portTICK_PERIOD_MS); 
                    uart_write_bytes(UART_NUM_0, (const char*)anaglyph_jpeg, anaglyph_jpeg_size);
                    printf("\nProceso exitoso. Anaglifo transmitido (%d bytes).\n", (int)anaglyph_jpeg_size);
                    
                    heap_caps_free(anaglyph_jpeg);
                } else {
                    printf("Fallo al comprimir a JPEG.\n");
                }
            } else {
                ESP_LOGE(TAG, "Fallo al asignar memoria para escalado.");
            }
        } else {
            printf("Error critico en decodificacion.\n");
        }

        // LIMPIEZA RIGUROSA
        if (rgb1) { heap_caps_free(rgb1); rgb1 = NULL; }
        if (rgb2) { heap_caps_free(rgb2); rgb2 = NULL; }
        if (jpg1) { free(jpg1); jpg1 = NULL; }
        if (jpg2) { free(jpg2); jpg2 = NULL; }
    }
}

extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Preparando camara a 38400 bps...");
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

    gpio_reset_pin((gpio_num_t)CAM_TX_PIN);
    gpio_reset_pin((gpio_num_t)CAM_RX_PIN);

    ptc06_config_t cfg = {
        .uart_num = UART_NUM_1, .tx_pin = CAM_TX_PIN, .rx_pin = CAM_RX_PIN,
        .baud_rate = 115200, .uart_rx_buf_size = 4096, .uart_tx_buf_size = 512,
        .cmd_timeout_ms = 2000
    };
    
    ptc06_t *cam = NULL;
    if (ptc06_init(&cam, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al enlazar la camara PTC06");
        return;
    }
    ptc06_set_resolution(cam, PTC06_RES_320x240);

    uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0);
    printf("\n--- ESP32 Sistema de Adquisicion Estereoscopica Listo ---\n");

    // =========================================================================
    // MAGIA DE FREERTOS: Lanzamos la tarea con 32,768 bytes de Pila (Stack)
    // =========================================================================
    xTaskCreatePinnedToCore(cansat_vision_task, "Vis_Task", 32768, (void*)cam, 5, NULL, 1);
}
