#pragma once
#include <array>
#include <chrono>
#include <stdio.h>
#include <thread>
#include <unistd.h>

#include "ppp.h"

const uint32_t MAX_ATTEMPTS = 150000;
uint16_t MAX_READ_SIZE = 128;

template <std::size_t N>
bool compute_checksum(std::array<uint8_t, N> &buffer, uint16_t &buffer_size) {
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

template <std::size_t N, std::size_t N2>
int read_until(std::array<uint8_t, N> &stuffed_buffer,
               std::array<uint8_t, N2> &buffer) {
  int bytes_read = 0;
  uint32_t attempts = 0;
  Unstuffer unstuffer(buffer);

  while (true) {
    if (attempts >= MAX_ATTEMPTS) {
      printf("Max read attempts reached, %d bytes received after %d attempts\n",
             bytes_read, attempts);
      return -1;
    }

    int result = read_serial(stuffed_buffer.data() + bytes_read, MAX_READ_SIZE);

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
