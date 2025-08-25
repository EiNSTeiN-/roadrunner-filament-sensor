#include <stdio.h>
#include "pico/stdlib.h"

void uartserial_init();
void uartserial_read(uint8_t *buf, size_t len);
void uartserial_write(uint8_t *buf, size_t len);
uint8_t uartserial_getc();
bool uartserial_sync();
void uartserial_loop();