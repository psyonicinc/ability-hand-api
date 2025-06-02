#include <stdio.h>

#include "ppp.h"

int ppp_stuff(uint8_t *payload, uint16_t &payload_size, uint8_t *stuffed_buffer,
              const uint16_t &stuffed_buffer_size) {
  int bidx = 0;
  stuffed_buffer[bidx++] = FRAME_CHAR;
  for (int i = 0; i < payload_size; i++) {
    uint8_t b = payload[i];
    if ((b == FRAME_CHAR) || (b == ESC_CHAR)) {
      if (bidx + 1 >= stuffed_buffer_size)
        return 0;
      stuffed_buffer[bidx++] = ESC_CHAR;
      stuffed_buffer[bidx++] = b ^ ESC_MASK;
    } else {
      if (bidx >= stuffed_buffer_size)
        return 0;
      stuffed_buffer[bidx++] = b;
    }
  }
  stuffed_buffer[bidx++] = FRAME_CHAR;
  return bidx;
}

Unstuffer::Unstuffer(uint8_t *unstuffed_buffer,
                     const uint16_t &unstuffed_buffer_size)
    : m_buffer(unstuffed_buffer), m_buffer_size(unstuffed_buffer_size) {}

void Unstuffer::reset_state() {
  state = PPPState::START_FRAME;
  m_idx = 0;
}

void Unstuffer::add_to_buffer(const uint8_t &byte) {
  if (m_idx >= m_buffer_size) {
    printf("Warning exceeded unstuffer max buffer size");
    reset_state();
  }
  m_buffer[m_idx++] = byte;
}

/*
Stateful byte parser for unstuffing PPP stuffed frames.  Unstuffing
simply require you to remove the FRAME_CHAR 0x20 '~' byte from the end
and beginning of the frame, it also requires removing any ESC_CHAR
characters 0x7D (NOT ASCII) and XOR bytes that follow ESC_CHARS with
MASK_CHAR 0x20.  This is required if the frame contains a FRAME_CHAR or
ESC_CHAR not intended to be used for stuffing. Really only needs to be a
one state state machine, read data, or don't, but state machine helps
with readability and understanding the if statements
*/

uint16_t Unstuffer::unstuff_byte(uint8_t byte) {
  if (byte != FRAME_CHAR) {
    // If we see a non frame char and not in a reading data state, skip
    if (state != PPPState::DATA) {
      return 0;
    }
  } else {
    if (m_idx > 0) {
      // We are at the end of the frame
      state = PPPState::END_FRAME; // For readability
      uint8_t idx_copy = m_idx;
      reset_state();
      return idx_copy;
    } else {
      // We are at the beginning of the frame
      reset_state();
      state = PPPState::DATA;
      return 0;
    }
  }

  if (byte == ESC_CHAR) {
    // Next byte needs to be unmasked
    unmask_next_char = true;
    return 0;
  }

  if (unmask_next_char) {
    byte ^= ESC_MASK;
    unmask_next_char = false;
  }

  if (state == PPPState::DATA) {
    add_to_buffer(byte);
    return 0;
  }

  return 0;
}
