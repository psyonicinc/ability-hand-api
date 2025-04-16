#pragma once
#include "api.h"
#include "hand.h"

float V_CONSTANT = 3.3 / 4096.0;
float C_1 = 121591.0;
float C_2 = 0.878894;

template <std::size_t N>
int parse_packet(const std::array<uint8_t, N> &packet,
                 const uint16_t &packet_size, Hand &hand,
                 const uint8_t &r_mode) {
  uint8_t header;
  int16_t val;
  uint16_t offset = 0;
  uint8_t hot_cold;
  uint8_t fsr_idx = 0;

  switch (r_mode) {
  case 0:
    if (packet_size != 71) {
      printf("Bad Packet Size");
      return -1;
    }
    std::memcpy(&header, packet.data() + offset, 1);
    ++offset;

    for (uint8_t i = 0; i < 6; ++i) {
      std::memcpy(&val, packet.data() + offset, 2);
      offset += 2;
      hand.pos[i] = val * DEGREES_CONSTANT_INV;
      std::memcpy(&val, packet.data() + offset, 2);
      offset += 2;
      hand.cur[i] = val / C;
    }

    while (offset < 70) {
      std::memcpy(&val, packet.data() + offset++, 2);
      hand.fsr[fsr_idx++] = val & 0x0FFF;
      std::memcpy(&val, packet.data() + offset++, 2);
      hand.fsr[fsr_idx++] = (val & 0xFFF0) >> 4;
      ++offset;
    }

    std::memcpy(&hot_cold, packet.data() + offset, 1);
    return 0;

  case 1:
    if (packet_size != 71) {
      printf("Bad Packet Size");
      return -1;
    }
    std::memcpy(&header, packet.data() + offset, 1);
    ++offset;

    for (uint8_t i = 0; i < 6; ++i) {
      std::memcpy(&val, packet.data() + offset, 2);
      offset += 2;
      hand.pos[i] = val * DEGREES_CONSTANT_INV;
      std::memcpy(&val, packet.data() + offset, 2);
      offset += 2;
      hand.vel[i] = val * (VELOCITY_CONSTANT_INV / 4);
    }

    while (offset < 70) {
      std::memcpy(&val, packet.data() + offset++, 2);
      hand.fsr[fsr_idx++] = val & 0x0FFF;
      std::memcpy(&val, packet.data() + offset++, 2);
      hand.fsr[fsr_idx++] = (val & 0xFFF0) >> 4;
      ++offset;
    }

    std::memcpy(&hot_cold, packet.data() + offset, 1);
    return 0;

  case 2:
    if (packet_size != 38) {
      printf("Bad Packet Size");
      return -1;
    }
    std::memcpy(&header, packet.data() + offset, 1);
    ++offset;

    for (uint8_t i = 0; i < 6; ++i) {
      std::memcpy(&val, packet.data() + offset, 2);
      offset += 2;
      hand.pos[i] = val * DEGREES_CONSTANT_INV;
      std::memcpy(&val, packet.data() + offset, 2);
      offset += 2;
      hand.cur[i] = val / C;
    }

    for (uint8_t i = 0; i < 6; ++i) {
      std::memcpy(&val, packet.data() + offset, 2);
      offset += 2;
      hand.vel[i] = val * (VELOCITY_CONSTANT_INV / 4);
    }

    std::memcpy(&hot_cold, packet.data() + offset, 1);
    return 0;
  }

  return -1;
}
