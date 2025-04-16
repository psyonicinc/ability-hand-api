#pragma once
#include <Windows.h>
#include <stdint.h>

int connect_to_usb_serial(HANDLE *serial_handle, const char *com_port_name,
                          unsigned long baud);
int autoconnect_serial(void);
int serial_write(uint8_t *data, int size);
int read_serial(uint8_t *readbuf, int bufsize);
void close_serial(void);
