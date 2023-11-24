#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define ADDR _u(0x36)
#define I2C_INST i2c_get_instance(1)

#define DIR_PIN 9
#define SDA_PIN 10
#define SCL_PIN 11

#define DIR_VALUE 1

// hardware registers
#define REG_ZMCO        _u(0x00)
#define REG_ZPOS        _u(0x01)
#define REG_MPOS        _u(0x03)
#define REG_MANG        _u(0x05)
#define REG_CONF        _u(0x07)
#define REG_RAW_ANGLE   _u(0x0C)
#define REG_ANGLE       _u(0x0E)
#define REG_STATUS      _u(0x0B)
#define REG_AGC         _u(0x1A)
#define REG_MAGNITUDE   _u(0x1B)
#define REG_BURN        _u(0xFF)

/*
# PM(1:0)     1:0     Power Mode      00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3
# HYST(1:0)   3:2     Hysteresis      00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs
# OUTS(1:0)   5:4     Output Stage    00 = analog (full range from 0% to 100% between GND and VDD, 01 = analog (reduced range from 10% to 90% between GND and VDD, 10 = digital PWM
# PWMF(1:0)   7:6     PWM Frequency   00 = 115 Hz; 01 = 230 Hz; 10 = 460 Hz; 11 = 920 Hz
# SF(1:0)     9:8     Slow Filter     00 = 16x (1); 01 = 8x; 10 = 4x; 11 = 2x
# FTH(2:0)    12:10   Fast Filter Threshold   000 = slow filter only, 001 = 6 LSBs, 010 = 7 LSBs, 011 = 9 LSBs,100 = 18 LSBs, 101 = 21 LSBs, 110 = 24 LSBs, 111 = 10 LSBs
# WD          13      Watchdog        0 = OFF, 1 = ON
*/

#define BIT_POWER_MODE          0
#define BIT_HYSTERESIS          2
#define BIT_OUTPUT_STAGE        4
#define BIT_PWM_FREQENCY        6
#define BIT_SLOW_FILTER         8
#define BIT_FAST_FILTER_THRES   10
#define BIT_WATCHDOG            13
#define BIT_MAGNET_OVERFLOW     3
#define BIT_MAGNET_UNDERFLOW    4
#define BIT_MAGNET_DETECTED     5

#define MASK_BITS(n) ((1 << n) - 1)

bool as5600_read_raw(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_write_blocking(I2C_INST, ADDR, &reg, 1, true);  // true to keep master control of bus
    return i2c_read_blocking(I2C_INST, ADDR, buf, len, false) == len;  // false - finished with bus
}

bool as5600_read8(uint8_t reg, uint8_t *val) {
    uint8_t raw;
    return as5600_read_raw(reg, val, 1);
}

bool as5600_read16(uint8_t reg, uint16_t *val) {
    uint16_t raw;
    if(!as5600_read_raw(reg, (uint8_t *)&raw, 2))
        return false;
    *val = ((raw & 0xff) << 8) | ((raw >> 8) & 0xff);
    return true;
}

bool as5600_write_raw(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_write_blocking(I2C_INST, ADDR, &reg, 1, true);  // true to keep master control of bus
    return i2c_write_blocking(I2C_INST, ADDR, buf, len, false) == len;  // false - finished with bus
}

bool as5600_write8(uint8_t reg, uint8_t val) {
    return as5600_write_raw(reg, &val, 1);
}

bool as5600_write16(uint8_t reg, uint16_t val) {
    return as5600_write_raw(reg, (uint8_t *)&val, 2);
}

bool as5600_get_zmco(uint8_t *val)
{
    return as5600_read8(REG_ZMCO, val);
}

bool as5600_get_zero_position(uint16_t *val)
{
    return as5600_read16(REG_ZPOS, val);
}

bool as5600_set_zero_position(uint16_t val)
{
    return as5600_write16(REG_ZPOS, val& MASK_BITS(12));
}

bool as5600_get_max_position(uint16_t *val)
{
    return as5600_read16(REG_MPOS, val);
}

bool as5600_set_max_position(uint16_t val)
{
    return as5600_write16(REG_MPOS, val & MASK_BITS(12));
}

bool as5600_get_max_angle(uint16_t *val)
{
    return as5600_read16(REG_MANG, val);
}

bool as5600_set_max_angle(uint16_t val)
{
    return as5600_write16(REG_MANG, val & MASK_BITS(12));
}

bool as5600_get_config_raw(uint8_t *val, uint8_t start_bit, uint16_t mask)
{
    uint16_t conf;
    if(!as5600_read16(REG_CONF, &conf))
        return false;
    *val = (conf >> start_bit) & mask;
    return true;
}

bool as5600_set_config_raw(uint8_t val, uint8_t start_bit, uint16_t mask)
{
    uint16_t conf;
    if(!as5600_read16(REG_CONF, &conf))
        return false;
    conf &= ~(mask << start_bit);
    return as5600_write16(REG_CONF, conf | ((val & mask) << start_bit));
}

bool as5600_get_config_power_mode(uint8_t *val)
{
    return as5600_get_config_raw(val, BIT_POWER_MODE, MASK_BITS(2));
}

bool as5600_set_config_power_mode(uint8_t val)
{
    return as5600_set_config_raw(val, BIT_POWER_MODE, MASK_BITS(2));
}

bool as5600_get_config_hysteresis(uint8_t *val)
{
    return as5600_get_config_raw(val, BIT_HYSTERESIS, MASK_BITS(2));
}

bool as5600_set_config_hysteresis(uint8_t val)
{
    return as5600_set_config_raw(val, BIT_HYSTERESIS, MASK_BITS(2));
}

bool as5600_get_config_output_stage(uint8_t *val)
{
    return as5600_get_config_raw(val, BIT_OUTPUT_STAGE, MASK_BITS(2));
}

bool as5600_set_config_output_stage(uint8_t val)
{
    return as5600_set_config_raw(val, BIT_OUTPUT_STAGE, MASK_BITS(2));
}

bool as5600_get_config_pwm_frequency(uint8_t *val)
{
    return as5600_get_config_raw(val, BIT_PWM_FREQENCY, MASK_BITS(2));
}

bool as5600_set_config_pwm_frequency(uint8_t val)
{
    return as5600_set_config_raw(val, BIT_PWM_FREQENCY, MASK_BITS(2));
}

bool as5600_get_config_slow_filter(uint8_t *val)
{
    return as5600_get_config_raw(val, BIT_SLOW_FILTER, MASK_BITS(2));
}

bool as5600_set_config_slow_filter(uint8_t val)
{
    return as5600_set_config_raw(val, BIT_SLOW_FILTER, MASK_BITS(2));
}

bool as5600_get_config_fast_filter_threshold(uint8_t *val)
{
    return as5600_get_config_raw(val, BIT_FAST_FILTER_THRES, MASK_BITS(3));
}

bool as5600_set_config_fast_filter_threshold(uint8_t val)
{
    return as5600_set_config_raw(val, BIT_FAST_FILTER_THRES, MASK_BITS(3));
}

bool as5600_get_config_watchdog(uint8_t *val)
{
    return as5600_get_config_raw(val, BIT_WATCHDOG, MASK_BITS(1));
}

bool as5600_set_config_watchdog(uint8_t val)
{
    return as5600_set_config_raw(val, BIT_WATCHDOG, MASK_BITS(1));
}

bool as5600_get_raw_angle(uint16_t *val)
{
    return as5600_read16(REG_RAW_ANGLE, val);
}

bool as5600_get_angle(uint16_t *val)
{
    return as5600_read16(REG_ANGLE, val);
}

bool as5600_get_status_raw(uint8_t *val, uint8_t start_bit, uint16_t mask) {
    uint8_t status;
    if(!as5600_read8(REG_STATUS, &status))
        return false;
    *val = (status >> start_bit) & mask;
    return true;
}

bool as5600_get_status_magnet_too_strong(uint8_t *val)
{
    return as5600_get_status_raw(val, BIT_MAGNET_OVERFLOW, MASK_BITS(1));
}

bool as5600_get_status_magnet_too_weak(uint8_t *val)
{
    return as5600_get_status_raw(val, BIT_MAGNET_UNDERFLOW, MASK_BITS(1));
}

bool as5600_get_status_magnet_detected(uint8_t *val)
{
    return as5600_get_status_raw(val, BIT_MAGNET_DETECTED, MASK_BITS(1));
}

bool as5600_get_status(uint8_t *detected, uint8_t *too_weak, uint8_t *too_strong)
{
    uint8_t status;
    if(!as5600_read8(REG_STATUS, &status))
        return false;
    *detected = (status >> BIT_MAGNET_DETECTED) & MASK_BITS(1);
    *too_weak = (status >> BIT_MAGNET_UNDERFLOW) & MASK_BITS(1);
    *too_strong = (status >> BIT_MAGNET_OVERFLOW) & MASK_BITS(1);
    return true;
}

bool as5600_get_automatic_gain_control(uint8_t *val)
{
    return as5600_read8(REG_AGC, val);
}

bool as5600_get_magnitude(uint16_t *val)
{
    return as5600_read16(REG_MAGNITUDE, val);
}

bool as5600_burn(uint8_t val)
{
    return as5600_write8(REG_BURN, val);
}

int as5600_init() {
    // useful information for picotool
    bi_decl(bi_2pins_with_func(SDA_PIN, SCL_PIN, GPIO_FUNC_I2C));

    i2c_init(I2C_INST, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // initialize DIR pin connected to AS5600
    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 1);

    // configure AS5600
    as5600_set_zero_position(0);
    as5600_set_max_position(4095);
    as5600_set_config_hysteresis(3);
}
