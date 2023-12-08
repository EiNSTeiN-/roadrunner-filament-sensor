/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"

#include "neopixel.h"
#include "as5600.h"
#include "ir_sensor.h"
#include "tmcuart.h"
#include "i2c_target.h"

/* leave commented for UART communication with the printer, uncomment for I2C */
//#define IS_I2C_TARGET 1

#define READ_ALL                    0x10
// #define READ_HEALTH                 0x20
#define READ_MAGNET_STATE           0x21
#define READ_FILAMENT_PRESENCE      0x22
#define READ_FULL_TURNS             0x23
#define READ_ANGLE                  0x24

#define MAGNET_STATE_UNKNOWN        0
#define MAGNET_STATE_NOT_DETECTED   1
#define MAGNET_STATE_TOO_WEAK       2
#define MAGNET_STATE_TOO_STRONG     3
#define MAGNET_STATE_DETECTED       4

// #define HEALTH_UNKNOWN                 -1
// #define HEALTH_OK                       0
// #define HEALTH_AS5600_READ_FAILED       1

#define UPDATE_INTERVAL_MS 10

struct state_t {
    // uint8_t health;

    uint8_t magnet_state;
    bool filament_present;

    int32_t full_turns; // number of times the magnet has crossed the zero point
    int32_t angle; // current angle value
} __attribute__((packed)) state;

clock_t last_update;

bool update_magnet_state()
{
    uint8_t detected, too_weak, too_strong;

    if(!as5600_get_status(&detected, &too_weak, &too_strong)) {
        state.magnet_state = MAGNET_STATE_UNKNOWN;
        return false;
    }

    if(detected) {
        if(too_weak) {
            state.magnet_state = MAGNET_STATE_TOO_WEAK;
        }
        else if(too_strong) {
            state.magnet_state = MAGNET_STATE_TOO_STRONG;
        }
        else {
            state.magnet_state = MAGNET_STATE_DETECTED;
        }
    } else {
        state.magnet_state = MAGNET_STATE_NOT_DETECTED;
    }

    return true;
}

bool update_magnet_angle()
{
    uint16_t angle;

    if(!as5600_get_angle(&angle)) {
        return false;
    }

    // check if the change is too large in either direction,
    // meaning the angle looped around across the zero position
    int32_t change = (angle - state.angle);
    if(change > 2048 || change < -2048) {
        if(state.angle > 2048) {
            // previous angle was close to max value (4096)
            // change = (angle + 4096) - state.angle;
            // e.g. state.angle=4000, angle=4, change=100
            state.full_turns += 1;
        }
        else {
            // previous angle was close to min value (0)
            // change = (angle - 4096) + state.angle;
            // e.g. state.angle=4, angle=4000, change=100
            state.full_turns -= 1;
        }
    }
    state.angle = angle;

    return true;
}

void note_last_updated_time()
{
    last_update = (clock_t) time_us_64() / 10000;
}

void update_loop()
{
    clock_t now = (clock_t) time_us_64() / 10000;
    if((now - last_update) * 1000 / CLOCKS_PER_SEC > UPDATE_INTERVAL_MS) {
        note_last_updated_time();

        state.filament_present = ir_sensor_value();

        if(!update_magnet_state()) {
            printf("Update magnet state failed...\n");
            // state.health = HEALTH_AS5600_READ_FAILED;
            return;
        }

        if(!update_magnet_angle()) {
            printf("Update angle failed...\n");
            // state.health = HEALTH_AS5600_READ_FAILED;
            return;
        }

        // state.health = HEALTH_OK;
    }
}

#define MEMCPY_REG_DATA(__buf, __data, __length) \
    { \
        __length = sizeof(__data); \
        memcpy(__buf, (uint8_t *)&__data, sizeof(__data)); \
    }

void prepare_register_data(uint8_t reg, uint8_t *buf, size_t *length)
{
    if(reg == READ_ALL) {
        MEMCPY_REG_DATA(buf, state, *length);
    // } else if(reg == READ_HEALTH) {
    //     MEMCPY_REG_DATA(buf, state.health, *length);
    } else if(reg == READ_MAGNET_STATE) {
        MEMCPY_REG_DATA(buf, state.magnet_state, *length);
    } else if(reg == READ_FILAMENT_PRESENCE) {
        MEMCPY_REG_DATA(buf, state.filament_present, *length);
    } else if(reg == READ_FULL_TURNS) {
        MEMCPY_REG_DATA(buf, state.full_turns, *length);
    } else if(reg == READ_ANGLE) {
        MEMCPY_REG_DATA(buf, state.angle, *length);
    }
}

int main() {
    stdio_init_all();

    neopixel_init();
    sleep_ms(100);
    neopixel_solid(RED);

    printf("Starting...\n");

    // state.health = HEALTH_UNKNOWN;
    state.magnet_state = MAGNET_STATE_UNKNOWN;
    state.filament_present = false;
    state.full_turns = 0;
    state.angle = 0;
    note_last_updated_time();

    as5600_init();
    ir_sensor_init();

#if defined(IS_I2C_TARGET) && IS_I2C_TARGET
    i2c_target_init();
#else
    tmcuart_init();
#endif

    sleep_ms(1000);

    uint16_t min, max, mang;
    as5600_get_zero_position(&min);
    as5600_get_max_position(&max);
    as5600_get_max_angle(&mang);
    printf("AS5600 min position: %u max position: %u, max angle: %u\n", min, max, mang);
    
    printf("Entering main loop...\n");
    neopixel_solid(OFF);

    while (1) {
        update_loop();

        if(state.magnet_state != MAGNET_STATE_DETECTED) {
            neopixel_blink(RED, 100);
        } else if(!state.filament_present) {
            neopixel_solid(BLUE);
        } else {
            neopixel_solid(GREEN);
        }

        neopixel_loop();

#if !defined(IS_I2C_TARGET) || !IS_I2C_TARGET
        tmcuart_loop();
#endif
    }
}
