#include "parser.h"

#include <stdio.h>

/* Takes in a pointer to data array and the size of that array.  Depending on
mode will parse the packet and update the hand's properties */
int parse_packet(uint8_t *data, const uint16_t &packet_size, Hand &hand,
                 const uint8_t &r_mode) {
  uint8_t header;      // Not currently used
  int16_t val;         // Data placeholder for single byte
  uint16_t offset = 0; // Current offset / idx in data
  uint8_t hot_cold;    // Not currently used
  uint8_t fsr_idx = 0;

  switch (r_mode) {
  case 0:
    if (packet_size != 71) {
      printf("Bad Packet Size");
      return -1;
    }
    std::memcpy(&header, data + offset, 1);
    ++offset;

    for (uint8_t i = 0; i < 6; ++i) {
      std::memcpy(&val, data + offset, 2);
      offset += 2;
      hand.pos[i] = val * DEGREES_CONSTANT_INV;
      std::memcpy(&val, data + offset, 2);
      offset += 2;
      hand.cur[i] = val / C;
    }

    while (offset < 70) {
      std::memcpy(&val, data + offset++, 2);
      hand.fsr[fsr_idx++] = val & 0x0FFF;
      std::memcpy(&val, data + offset++, 2);
      hand.fsr[fsr_idx++] = (val & 0xFFF0) >> 4;
      ++offset;
    }

    std::memcpy(&hot_cold, data + offset, 1);
    return 0;

  case 1:
    if (packet_size != 71) {
      printf("Bad Packet Size");
      return -1;
    }
    std::memcpy(&header, data + offset, 1);
    ++offset;

    for (uint8_t i = 0; i < 6; ++i) {
      std::memcpy(&val, data + offset, 2);
      offset += 2;
      hand.pos[i] = val * DEGREES_CONSTANT_INV;
      std::memcpy(&val, data + offset, 2);
      offset += 2;
      hand.vel[i] = val * (VELOCITY_CONSTANT_INV / 4);
    }

    while (offset < 70) {
      std::memcpy(&val, data + offset++, 2);
      hand.fsr[fsr_idx++] = val & 0x0FFF;
      std::memcpy(&val, data + offset++, 2);
      hand.fsr[fsr_idx++] = (val & 0xFFF0) >> 4;
      ++offset;
    }

    std::memcpy(&hot_cold, data + offset, 1);
    return 0;

  case 2:
    if (packet_size != 38) {
      printf("Bad Packet Size");
      return -1;
    }
    std::memcpy(&header, data + offset, 1);
    ++offset;

    for (uint8_t i = 0; i < 6; ++i) {
      std::memcpy(&val, data + offset, 2);
      offset += 2;
      hand.pos[i] = val * DEGREES_CONSTANT_INV;
      std::memcpy(&val, data + offset, 2);
      offset += 2;
      hand.cur[i] = val / C;
    }

    for (uint8_t i = 0; i < 6; ++i) {
      std::memcpy(&val, data + offset, 2);
      offset += 2;
      hand.vel[i] = val * (VELOCITY_CONSTANT_INV / 4);
    }

    std::memcpy(&hot_cold, data + offset, 1);
    return 0;
  }

  return -1;
}
