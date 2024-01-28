#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/uart.h"

#include "tmcuart.h"

#define SERIAL_BAUD 40000

#define UART_INST uart1
#define UART_TX_PIN 4
#define UART_RX_PIN 5

void tmcuart_init()
{
    uart_init(UART_INST, SERIAL_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

uint8_t tmcuart_crc8(uint8_t *buf, size_t len)
{
    // Generate a CRC8-ATM value for a bytearray
    uint8_t crc = 0;

    for(size_t j = 0; j < len; j++) {
        uint8_t b = buf[j];
        for(size_t i = 0; i < 8; i++) {
            if ((crc >> 7) ^ (b & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            crc &= 0xff;
            b >>= 1;
        }
    }

    return crc;
}

uint8_t tmcuart_getc()
{
    uint8_t c = 0;
    tmcuart_read(&c, 1);
    return c;
}

void tmcuart_read(uint8_t *buf, size_t len)
{
    uart_read_blocking(UART_INST, buf, len);
}

void tmcuart_write(uint8_t *buf, size_t len)
{
    uart_write_blocking(UART_INST, buf, len);
}

bool tmcuart_sync()
{
    if(uart_is_readable(UART_INST))
        return tmcuart_getc() == 0xf5;
    return false;
}

void prepare_register_data(uint8_t reg, uint8_t *buf, size_t *length);

static void tmcuart_send_response(uint8_t reg)
{
    uint8_t data[255];
    size_t length = 0;

    memset((void *)&data, 0, sizeof(data));
    data[0] = 0x05;
    data[1] = 0xff;
    data[2] = reg;

    prepare_register_data(reg, &data[3], &length);

    if((length + 4) > sizeof(data))
        return;

    data[length+3] = tmcuart_crc8((uint8_t *)&data, length+3);

    // klipper tmcuart bitbang is sensitive to timing, avoid replying too fast
    sleep_us(500);

    tmcuart_write((uint8_t *)&data, length+4);
}

void tmcuart_loop()
{
    uint8_t cmd[4] = { 0xf5, 0, 0, 0 };
    uint8_t addr, reg, crc;

    if (!tmcuart_sync())
        return;

    tmcuart_read(&cmd[1], 3);

    addr = cmd[1];
    reg = cmd[2];
    crc = cmd[3];

    if(crc != tmcuart_crc8((uint8_t *)&cmd, 3)) {
        printf("received addr=%02x reg=%02x crc=%02x (BAD CRC)\n", addr, reg, crc);
        return;
    }

    tmcuart_send_response(reg);
}