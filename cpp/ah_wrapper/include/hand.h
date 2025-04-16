#pragma once
#include <array>
#include <cstdint>

class Hand {
public:
  Hand(const uint8_t &h_address);
  std::array<float, 6> pos;
  std::array<float, 6> vel;
  std::array<float, 6> cur;
  std::array<uint16_t, 30> fsr;
  const uint8_t address;
};