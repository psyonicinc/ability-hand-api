#pragma once
#include <array>
#include <stdint.h>

#define FRAME_CHAR 0x7E
#define ESC_CHAR 0x7D
#define ESC_MASK 0x20

int ppp_stuff(uint8_t *payload, uint16_t &payload_size, uint8_t *stuffed_buffer,
              const uint16_t &stuffed_buffer_size);

class Unstuffer {
private:
  bool unmask_next_char = false;
  void reset_state();
  void add_to_buffer(const uint8_t &byte);
  uint8_t *m_buffer;
  uint16_t m_buffer_size;
  uint16_t m_idx = 0;

public:
  Unstuffer(uint8_t *unstuffed_buffer, const uint16_t &unstuffed_buffer_size);
  uint16_t unstuff_byte(uint8_t byte);
};