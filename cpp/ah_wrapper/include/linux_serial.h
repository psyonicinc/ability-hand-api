#pragma once
#include <stdint.h>

int autoconnect_serial(const uint32_t &baud_rate);
int serial_write(uint8_t *data, uint16_t &size);
int read_serial(uint8_t *readbuf, uint16_t &bufsize);
void close_serial(void);
