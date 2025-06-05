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

void Unstuffer::add_to_buffer(const uint8_t &byte) {
  if (m_idx >= m_buffer_size) {
    printf("Warning exceeded unstuffer max buffer size");
    m_idx = 0;
  }
  m_buffer[m_idx++] = byte;
}

/*
Stateful byte parser for unstuffing PPP stuffed frames.  Unstuffing
simply require you to remove the FRAME_CHAR 0x20 '~' byte from the end
and beginning of the frame, it also requires removing any ESC_CHAR
characters 0x7D (NOT ASCII) and XOR bytes that follow ESC_CHARS with
MASK_CHAR 0x20.  This is required if the frame contains a FRAME_CHAR or
ESC_CHAR not intended to be used for stuffing.
*/

uint16_t Unstuffer::unstuff_byte(uint8_t byte) {
  if (byte == FRAME_CHAR) {
    //  Have to deal with annoying null terminated/begin RS485 adapters which can sometimes create a frame of one or two which will pass checksum
    if (m_idx > 3) {
      uint8_t idx_copy = m_idx;
      m_idx = 0;
      return idx_copy;
    } else {
      m_idx = 0;
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

  add_to_buffer(byte);
  return 0;
}
