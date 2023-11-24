
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pico/stdlib.h"

#include "neopixel.h"

#define IS_RGBW true
#define WS2812_PIN 16

#define NEOPIXEL_MODE_SOLID 0
#define NEOPIXEL_MODE_BLINK 1

static int neopixel_mode = NEOPIXEL_MODE_SOLID;
static int neopixel_blink_time_ms = 0;
static uint32_t neopixel_color = 0;

static bool neopixel_last_blink_state = false;
static clock_t neopixel_last_blink_time = 0;

clock_t clock()
{
    return (clock_t) time_us_64() / 10000;
}

void neopixel_init()
{
    uint offset = pio_add_program(pio0, &ws2812_program);
    ws2812_program_init(pio0, 0, offset, WS2812_PIN, 800000, IS_RGBW);
    neopixel_last_blink_time = clock();
    put_pixel(OFF);
}

void neopixel_set_mode(int mode)
{
    neopixel_mode = mode;
}

void neopixel_set_color(uint32_t color)
{
    if(neopixel_color != color) {
        neopixel_color = color;
        put_pixel(color);
    }
}

void neopixel_set_blink_time_ms(int time)
{
    neopixel_blink_time_ms = time;
}

void neopixel_solid(uint32_t color)
{
    neopixel_set_mode(NEOPIXEL_MODE_SOLID);
    neopixel_set_color(color);
}

void neopixel_blink(uint32_t color, int time)
{
    neopixel_set_mode(NEOPIXEL_MODE_BLINK);
    neopixel_set_color(color);
    neopixel_set_blink_time_ms(time);

    if(neopixel_mode != NEOPIXEL_MODE_BLINK) {
        neopixel_last_blink_time = clock();
        neopixel_last_blink_state = true;
    }
}

int elapsed_since_blink()
{
    return (clock() - neopixel_last_blink_time) * 1000 / CLOCKS_PER_SEC;
}

void neopixel_loop()
{
    if(neopixel_mode == NEOPIXEL_MODE_BLINK) {
        if(elapsed_since_blink() > neopixel_blink_time_ms) {
            if(neopixel_last_blink_state) {
                put_pixel(OFF);
                neopixel_last_blink_state = false;
            }
            else {
                put_pixel(neopixel_color);
                neopixel_last_blink_state = true;
            }
            neopixel_last_blink_time = clock();
        }
    }
}