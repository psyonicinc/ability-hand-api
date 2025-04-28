#include "serial_helper.h"

#ifdef PLATFORM_WINDOWS
#include "winserial.h"
#elif defined(PLATFORM_LINUX)
#include "linux_serial.h"
#endif

uint16_t MAX_READ_SIZE = 128;

bool compute_checksum(uint8_t *buffer, uint16_t &buffer_size) {
  int checksum = 0;
  for (uint16_t i = 0; i < buffer_size - 1; ++i) {
    checksum += buffer[i];
  }
  if (((-checksum) & 0xFF) != buffer[buffer_size - 1]) {
    return false;
  } else {
    buffer_size -= 1;
    return true;
  }
}

int read_until(uint8_t *stuffed_buffer, uint8_t *buffer,
               const uint16_t &stuffed_buffer_size,
               const uint16_t &buffer_size) {
  int bytes_read = 0;
  uint32_t attempts = 0;
  Unstuffer unstuffer(buffer, buffer_size);

  while (true) {
    if (attempts >= MAX_ATTEMPTS) {
      printf("Max read attempts reached, %d bytes received after %d attempts\n",
             bytes_read, attempts);
      return -1;
    }

    int result = read_serial(stuffed_buffer + bytes_read, MAX_READ_SIZE);

    for (uint16_t i = bytes_read; i < result + bytes_read; ++i) {
      uint16_t size = unstuffer.unstuff_byte(stuffed_buffer[i]);
      if (size > 0) {
        bool checksum_passed = compute_checksum(buffer, size);
        if (checksum_passed) {
          return size;
        } else {
          printf("Checksum failed\n");
          return -1;
        }
      }
    }

    bytes_read += result;
    ++attempts;
  }

  return -1;
}