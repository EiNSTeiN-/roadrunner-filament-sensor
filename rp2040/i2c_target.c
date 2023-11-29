#include <stdio.h>

#include <hardware/i2c.h>
#include <pico/i2c_slave.h>
#include <pico/stdlib.h>

#include "i2c_target.h"

#define I2C_INST i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_BAUDRATE 100 * 1000 /* 100 kHz */
#define I2C_ADDRESS 0x40

static struct
{
    uint8_t mem[256];
    uint8_t mem_address;
    bool mem_address_written;
} context;

void prepare_register_data(uint8_t reg, uint8_t *buf, size_t *length);

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (!context.mem_address_written) {
            // writes always start with the memory address
            context.mem_address = i2c_read_byte_raw(i2c);
            context.mem_address_written = true;
        } else {
            /* read and discard, we do not support I2C writes */
            i2c_read_byte_raw(i2c);
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
    {
        size_t length = 0;
        prepare_register_data(context.mem_address, (uint8_t *)&context.mem, &length);
        for(int i = 0; i < length; i++)
            i2c_write_byte_raw(i2c, context.mem[i]);
        break;
    }
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        context.mem_address_written = false;
        break;
    default:
        break;
    }
}

void i2c_target_init()
{
    gpio_init(I2C_SDA_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);

    gpio_init(I2C_SCL_PIN);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL_PIN);

    i2c_init(I2C_INST, I2C_BAUDRATE);
    i2c_slave_init(I2C_INST, I2C_ADDRESS, &i2c_slave_handler);
}
