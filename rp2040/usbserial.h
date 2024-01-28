#include <stdio.h>
#include "pico/stdlib.h"

void usbserial_init();
void usbserial_read(uint8_t *buf, size_t len);
void usbserial_write(uint8_t *buf, size_t len);
uint8_t usbserial_getc();
bool usbserial_sync();
void usbserial_loop();