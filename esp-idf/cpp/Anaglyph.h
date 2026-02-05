#ifndef ANAGLYPH_H
#define ANAGLYPH_H

#include <stdint.h>

// Configuration
#define IMG_WIDTH  640
#define IMG_HEIGHT 480

// Function Declarations
int calculate_alignment_offset(uint16_t* img_left, uint16_t* img_right);
void generate_anaglyph(uint16_t* img_left, uint16_t* img_right, uint16_t* img_out, int offset);

#endif
