#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/pio.h"
#include "ws2812.pio.h"

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 16) |
            ((uint32_t) (g) << 8) |
            (uint32_t) (b);
}

#define RED urgb_u32(0xff, 0, 0)
#define GREEN urgb_u32(0, 0xff, 0)
#define BLUE urgb_u32(0, 0, 0xff)
#define OFF urgb_u32(0, 0, 0)

void neopixel_init();
void neopixel_loop();

void neopixel_solid(uint32_t color);
void neopixel_blink(uint32_t color, int time);