#include <stdio.h>
#include "pico/stdlib.h"

int as5600_init();

bool as5600_get_zmco(uint8_t *val);
bool as5600_get_zero_position(uint16_t *val);
bool as5600_set_zero_position(uint16_t val);
bool as5600_get_max_position(uint16_t *val);
bool as5600_set_max_position(uint16_t val);
bool as5600_get_max_angle(uint16_t *val);
bool as5600_set_max_angle(uint16_t val);
bool as5600_get_config_power_mode(uint8_t *val);
bool as5600_set_config_power_mode(uint8_t val);
bool as5600_get_config_hysteresis(uint8_t *val);
bool as5600_set_config_hysteresis(uint8_t val);
bool as5600_get_config_output_stage(uint8_t *val);
bool as5600_set_config_output_stage(uint8_t val);
bool as5600_get_config_pwm_frequency(uint8_t *val);
bool as5600_set_config_pwm_frequency(uint8_t val);
bool as5600_get_config_slow_filter(uint8_t *val);
bool as5600_set_config_slow_filter(uint8_t val);
bool as5600_get_config_fast_filter_threshold(uint8_t *val);
bool as5600_set_config_fast_filter_threshold(uint8_t val);
bool as5600_get_config_watchdog(uint8_t *val);
bool as5600_set_config_watchdog(uint8_t val);
bool as5600_get_raw_angle(uint16_t *val);
bool as5600_get_angle(uint16_t *val);
bool as5600_get_status_magnet_too_strong(uint8_t *val);
bool as5600_get_status_magnet_too_weak(uint8_t *val);
bool as5600_get_status_magnet_detected(uint8_t *val);
bool as5600_get_status(uint8_t *detected, uint8_t *too_weak, uint8_t *too_strong);
bool as5600_get_automatic_gain_control(uint8_t *val);
bool as5600_get_magnitude(uint16_t *val);
bool as5600_burn(uint8_t val);