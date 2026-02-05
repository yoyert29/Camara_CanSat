#include "esp_heap_caps.h"
#include <cmath>
#include <algorithm>

// --- Configuration ---
#define IMG_WIDTH  640
#define IMG_HEIGHT 480
#define SEARCH_RANGE 40  // How many pixels to search left/right for alignment
#define BLOCK_SIZE   64  // Size of the reference block for matching (64x64)

// Helper to extract RGB components from RGB565
// RGB565 format: RRRR RGGG GGGB BBBB
#define GET_R(color) ((color >> 11) & 0x1F)
#define GET_G(color) ((color >> 5) & 0x3F)
#define GET_B(color) (color & 0x1F)
#define MAKE_RGB565(r, g, b) (((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F))

/**
 * 1. Find the Zero-Parallax Offset
 * Compares the center of the Left image with the Right image to find the shift.
 * Returns: The X-offset (negative means shift left, positive means shift right).
 */
int calculate_alignment_offset(uint16_t* img_left, uint16_t* img_right) {
    long min_sad = 2147483647; // Start with max possible value
    int best_offset = 0;

    int center_x = IMG_WIDTH / 2;
    int center_y = IMG_HEIGHT / 2;
    int half_block = BLOCK_SIZE / 2;

    // Scan a horizontal range to find the best match
    for (int offset = -SEARCH_RANGE; offset <= SEARCH_RANGE; offset++) {
        long current_sad = 0;

        // Iterate through the center block
        for (int y = -half_block; y < half_block; y++) {
            for (int x = -half_block; x < half_block; x++) {
                
                // Coordinates in Left Image
                int lx = center_x + x;
                int ly = center_y + y;
                
                // Coordinates in Right Image (shifted by offset)
                int rx = center_x + x + offset;
                int ry = center_y + y;

                // Boundary check
                if (rx < 0 || rx >= IMG_WIDTH) continue;

                // Get pixels
                uint16_t pL = img_left[ly * IMG_WIDTH + lx];
                uint16_t pR = img_right[ry * IMG_WIDTH + rx];

                // Simple luminance approximation (Green channel is dominant)
                int lumL = GET_G(pL);
                int lumR = GET_G(pR);

                // Sum of Absolute Differences (SAD)
                current_sad += abs(lumL - lumR);
            }
        }

        // Did we find a better match?
        if (current_sad < min_sad) {
            min_sad = current_sad;
            best_offset = offset;
        }
    }
    
    return best_offset;
}

/**
 * 2. Generate Anaglyph
 * Combines channels and applies the shift.
 */
void generate_anaglyph(uint16_t* img_left, uint16_t* img_right, uint16_t* img_out, int offset) {
    for (int y = 0; y < IMG_HEIGHT; y++) {
        for (int x = 0; x < IMG_WIDTH; x++) {
            
            // 1. Get Left Pixel (Red Source)
            uint16_t pixel_L = img_left[y * IMG_WIDTH + x];
            uint8_t r = GET_R(pixel_L);

            // 2. Get Right Pixel (Cyan Source) with OFFSET
            int x_right = x + offset;
            uint8_t g = 0;
            uint8_t b = 0;

            // Only fetch if within bounds after shifting
            if (x_right >= 0 && x_right < IMG_WIDTH) {
                uint16_t pixel_R = img_right[y * IMG_WIDTH + x_right];
                g = GET_G(pixel_R);
                b = GET_B(pixel_R);
            } 
            // If out of bounds (edge of image), keep black or replicate edge (using black here)

            // 3. Combine: R from Left, G/B from Right
            img_out[y * IMG_WIDTH + x] = MAKE_RGB565(r, g, b);
        }
    }
}

/**
 * 3. Main wrapper to call from your loop
 */
void process_3d_image() {
    size_t img_size = IMG_WIDTH * IMG_HEIGHT * sizeof(uint16_t);

    // ALLOCATE IN PSRAM (Crucial!)
    uint16_t* left_buffer  = (uint16_t*)heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM);
    uint16_t* right_buffer = (uint16_t*)heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM);
    uint16_t* out_buffer   = (uint16_t*)heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM);

    if (!left_buffer || !right_buffer || !out_buffer) {
        printf("ESP32: Error! Not enough PSRAM.\n");
        return;
    }

    // --- TODO: DECODE YOUR JPEG CAPTURES INTO left_buffer AND right_buffer HERE ---
    // (You would use a library like TJpgDec or esp_jpeg to convert your captured JPEGs 
    // into these raw RGB565 buffers)

    printf("ESP32: Calculating Zero Parallax alignment...\n");
    int offset = calculate_alignment_offset(left_buffer, right_buffer);
    printf("ESP32: Optimal subject offset found: %d pixels\n", offset);

    printf("ESP32: Merging Anaglyph...\n");
    generate_anaglyph(left_buffer, right_buffer, out_buffer, offset);
    
    printf("ESP32: Processing complete. Ready to encode/send.\n");

    // Free memory when done
    heap_caps_free(left_buffer);
    heap_caps_free(right_buffer);
    heap_caps_free(out_buffer);
}
