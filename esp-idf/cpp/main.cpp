#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For abs()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "jpeg_decoder.h" // From espressif/esp_jpeg
#include "ptc06.h"        // Your existing camera driver

#define CAM_TX_PIN 17
#define CAM_RX_PIN 18
#define CAM_BAUD 38400    
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define CHANNELS 3        // RGB888

static const char *TAG = "ANAGLYPH";

// --- ALGORITHM: Zero Parallax Alignment (RGB888 Version) ---
// Returns the horizontal offset (pixels) to shift the Right image
int calculate_alignment_offset(uint8_t* img_left, uint8_t* img_right) {
    long min_sad = 2147483647; // Start with max long
    int best_offset = 0;
    
    // Config: Search center 64x64 block, range +/- 30 pixels
    int center_x = IMAGE_WIDTH / 2;
    int center_y = IMAGE_HEIGHT / 2;
    int block_size = 64;
    int half_block = block_size / 2;
    int search_range = 30; 

    // Scan horizontal shifts
    for (int offset = -search_range; offset <= search_range; offset++) {
        long current_sad = 0;

        // Compare block
        for (int y = -half_block; y < half_block; y++) {
            for (int x = -half_block; x < half_block; x++) {
                
                int lx = center_x + x;
                int ly = center_y + y;
                
                // Right image coordinate is SHIFTED
                int rx = center_x + x + offset;
                int ry = center_y + y;

                // Bounds check
                if (rx < 0 || rx >= IMAGE_WIDTH) continue;

                // Calculate Index for RGB888 [R, G, B, R, G, B...]
                int idx_L = (ly * IMAGE_WIDTH + lx) * 3;
                int idx_R = (ry * IMAGE_WIDTH + rx) * 3;

                // Compare GREEN channel (Index + 1) for brightness match
                int diff = img_left[idx_L + 1] - img_right[idx_R + 1];
                
                // Absolute value (manual to avoid stdlib issues if C)
                if(diff < 0) diff = -diff; 
                
                current_sad += diff;
            }
        }

        // Keep best match
        if (current_sad < min_sad) {
            min_sad = current_sad;
            best_offset = offset;
        }
    }
    return best_offset;
}

// --- ALGORITHM: Merge with Shift ---
// Writes the result directly into the 'img_target' buffer (overwriting Left)
void apply_anaglyph_merge(uint8_t* img_target, uint8_t* img_right, int offset) {
    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            
            int idx_out = (y * IMAGE_WIDTH + x) * 3;
            
            // We keep RED from img_target (Left) automatically because we don't touch it.
            
            // We get CYAN (Green+Blue) from img_right (Shifted)
            int rx = x + offset;
            
            if (rx >= 0 && rx < IMAGE_WIDTH) {
                int idx_R = (y * IMAGE_WIDTH + rx) * 3;
                
                // Copy Green & Blue
                img_target[idx_out + 1] = img_right[idx_R + 1];
                img_target[idx_out + 2] = img_right[idx_R + 2];
            } else {
                // If the shift pushes this pixel off-screen, make the cyan channel black
                img_target[idx_out + 1] = 0;
                img_target[idx_out + 2] = 0;
            }
        }
    }
}

// --- Helper to decode a JPEG buffer into an RGB888 buffer ---
uint8_t* decode_jpeg_to_rgb(uint8_t *jpeg_data, size_t jpeg_len) {
    size_t out_size = (IMAGE_WIDTH * IMAGE_HEIGHT * CHANNELS) + 1024;
    
    uint8_t *out_buffer = (uint8_t*)heap_caps_malloc(out_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!out_buffer) {
        out_buffer = (uint8_t*)malloc(out_size);
    }
    
    if (!out_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RGB buffer!");
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
        ESP_LOGE(TAG, "JPEG Decode Failed: %d", ret);
        free(out_buffer);
        return NULL;
    }
    return out_buffer; 
}

extern "C" void app_main(void) {
    // 1. Initialize Camera
    ptc06_config_t cfg = {
        .uart_num = UART_NUM_1,
        .tx_pin = CAM_TX_PIN,
        .rx_pin = CAM_RX_PIN,
        .baud_rate = CAM_BAUD,
        .uart_rx_buf_size = 2048,
        .uart_tx_buf_size = 512,
        .cmd_timeout_ms = 2000
    };
    
    ptc06_t *cam = NULL;
    if (ptc06_init(&cam, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }
    ptc06_set_resolution(cam, PTC06_RES_320x240);

    // 2. Configure Console
    // Using simple getchar/putchar, so we just ensure driver is installed
    uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0);
    
    uint8_t *jpg1 = NULL, *jpg2 = NULL;
    size_t len1 = 0, len2 = 0;
    uint8_t *rgb1 = NULL, *rgb2 = NULL;

    printf("\n--- ESP32 Anaglyph Maker Ready ---\n");
    printf("Press 'p' to take LEFT picture.\n");

    while (1) {
        // Simple blocking read for command
        uint8_t cmd = 0;
        int len = uart_read_bytes(UART_NUM_0, &cmd, 1, 50 / portTICK_PERIOD_MS);
        
        if (len > 0 && cmd == 'p') {
            
            // --- STEP 1: CAPTURE LEFT ---
            if (jpg1 == NULL) {
                printf("Capturing LEFT image...\n");
                if (ptc06_capture_jpeg_to_buffer(cam, &jpg1, &len1) == ESP_OK) {
                    printf("LEFT captured (%d bytes). Move camera & press 'p'.\n", len1);
                } else {
                    printf("Camera Error on Left!\n");
                }
            } 
            // --- STEP 2: CAPTURE RIGHT ---
            else if (jpg2 == NULL) {
                printf("Capturing RIGHT image...\n");
                if (ptc06_capture_jpeg_to_buffer(cam, &jpg2, &len2) == ESP_OK) {
                    printf("RIGHT captured (%d bytes). Processing...\n", len2);
                    
                    // --- STEP 3: DECODE ---
                    rgb1 = decode_jpeg_to_rgb(jpg1, len1);
                    rgb2 = decode_jpeg_to_rgb(jpg2, len2);

                    if (rgb1 && rgb2) {
                        printf("Decoding success. Calculating Alignment...\n");
                        
                        // --- STEP 4: ALIGNMENT ---
                        int best_offset = calculate_alignment_offset(rgb1, rgb2);
                        printf("Correction Offset: %d pixels\n", best_offset);

                        // --- STEP 5: MERGE ---
                        // 'rgb1' will be modified to hold the final image
                        apply_anaglyph_merge(rgb1, rgb2, best_offset);

                        // --- STEP 6: SEND ---
                        int pixel_count = IMAGE_WIDTH * IMAGE_HEIGHT;
                        size_t total_raw_size = pixel_count * 3;
                        
                        printf("IMG_START:%d\n", total_raw_size);
                        vTaskDelay(50 / portTICK_PERIOD_MS); 
                        
                        uart_write_bytes(UART_NUM_0, (const char*)rgb1, total_raw_size);
                        
                        printf("\nDone! Resetting...\n");
                    } else {
                        printf("Decoding failed (PSRAM full?)\n");
                    }

                    // Cleanup
                    if (rgb1) free(rgb1);
                    if (rgb2) free(rgb2);
                    if (jpg1) free(jpg1);
                    if (jpg2) free(jpg2);
                    jpg1 = NULL; jpg2 = NULL;
                } else {
                    printf("Camera Error on Right!\n");
                }
            }
        }
    }
}
