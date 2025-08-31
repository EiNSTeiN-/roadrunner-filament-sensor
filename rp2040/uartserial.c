#include <stdio.h>
#include <string.h>

#include "pico/stdio_uart.h"
#include "hardware/pio.h"
#include "hardware/uart.h"

#include "uartserial.h"

#define SERIAL_BAUD 115200

#define UART_INST uart1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

void uartserial_init()
{
    uart_init(UART_INST, SERIAL_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(UART_INST, true);
    stdio_uart_init_full(UART_INST, SERIAL_BAUD, UART_TX_PIN, UART_RX_PIN);
}


uint8_t uartserial_getc()
{
    uint8_t c = 0;
    uartserial_read(&c, 1);
    return c;
}

void uartserial_read(uint8_t *buf, size_t len)
{
    for(size_t i=0; i<len; i++)
        // buf[i] = getchar_timeout_us(0);
        buf[i] = uart_getc(UART_INST);
}

void uartserial_write(uint8_t *buf, size_t len)
{
    for(size_t i=0; i<len; i++)
        // putchar_raw(buf[i]);
        uart_putc_raw(UART_INST,buf[i]);
    // stdio_flush();
}

bool uartserial_sync()
{
    return getchar_timeout_us(10) == 0xf5;
}

void prepare_register_data(uint8_t reg, uint8_t *buf, size_t *length);

static void uartserial_send_response(uint8_t reg)
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

    uartserial_write((uint8_t *)&data, length+3);
}

void uartserial_loop()
{
    uint8_t reg;

    if (!uartserial_sync())
        return;

    uartserial_read((uint8_t *)&reg, 1);

    uartserial_send_response(reg);
}