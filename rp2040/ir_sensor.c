#include "ir_sensor.h"

#define IR_SENSOR_PIN 0

int ir_sensor_init() {
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_pull_down(IR_SENSOR_PIN);
}

bool ir_sensor_value()
{
    return gpio_get(IR_SENSOR_PIN);
}
