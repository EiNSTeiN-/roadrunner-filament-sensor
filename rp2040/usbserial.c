#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/uart.h"

#include "usbserial.h"

void usbserial_init()
{
}

uint8_t usbserial_getc()
{
    uint8_t c = 0;
    usbserial_read(&c, 1);
    return c;
}

void usbserial_read(uint8_t *buf, size_t len)
{
    for(size_t i=0; i<len; i++)
        buf[i] = getchar_timeout_us(0);
}

void usbserial_write(uint8_t *buf, size_t len)
{
    for(size_t i=0; i<len; i++)
        putchar_raw(buf[i]);
    stdio_flush();
}

bool usbserial_sync()
{
    return getchar_timeout_us(10) == 0xf5;
}

void prepare_register_data(uint8_t reg, uint8_t *buf, size_t *length);

static void usbserial_send_response(uint8_t reg)
{
    uint8_t data[255];
    size_t length = 0;

    memset((void *)&data, 0, sizeof(data));
    data[0] = 0x05;
    data[1] = 0xff;
    data[2] = reg;

    prepare_register_data(reg, &data[3], &length);

    if((length + 3) > sizeof(data))
        return;

    usbserial_write((uint8_t *)&data, length+3);
}

void usbserial_loop()
{
    uint8_t reg;

    if (!usbserial_sync())
        return;

    usbserial_read((uint8_t *)&reg, 1);

    usbserial_send_response(reg);
}