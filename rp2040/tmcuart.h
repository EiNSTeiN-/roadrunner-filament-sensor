#include <stdio.h>
#include "pico/stdlib.h"

void tmcuart_init();
void tmcuart_read(uint8_t *buf, size_t len);
void tmcuart_write(uint8_t *buf, size_t len);
uint8_t tmcuart_crc8(uint8_t *buf, size_t len);
uint8_t tmcuart_getc();
bool tmcuart_sync();
void tmcuart_loop();