#pragma once
#include <array>
#include <stdint.h>

#define FRAME_CHAR 0x7E
#define ESC_CHAR 0x7D
#define ESC_MASK 0x20

int ppp_stuff(uint8_t *payload, uint16_t &payload_size, uint8_t *stuffed_buffer,
              const uint16_t &stuffed_buffer_size);

enum PPPState { START_FRAME, DATA, END_FRAME };

class Unstuffer {
private:
  PPPState state = PPPState::START_FRAME;
  bool unmask_next_char = false;
  void reset_state();
  void add_to_buffer(const uint8_t &byte);

public:
  uint8_t *buffer;
  uint16_t buffer_size;
  template <std::size_t N> Unstuffer(std::array<uint8_t, N> &unstuffed_buffer) {
    buffer = unstuffed_buffer.data();
    buffer_size = unstuffed_buffer.size();
  }
  uint16_t unstuff_byte(uint8_t byte);
  uint16_t idx = 0;
};