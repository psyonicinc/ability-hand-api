#pragma once

#include <chrono>

#include "hand.h"
#include "ppp.h"

const uint16_t BUFFER_SIZE = 512;
const uint16_t STUFFED_BUFFER_SIZE = BUFFER_SIZE * 2;

enum Command { POSITION, VELOCITY, CURRENT, DUTY };

class AHWrapper {
public:
  AHWrapper(const uint8_t &hand_addr, const uint32_t &b_rate);
  ~AHWrapper();
  int connect();
  int read_write_once(const std::array<float, 6> &cmd_values,
                      const Command &cmd, const uint8_t &reply_mode);
  Hand hand;
  size_t n_reads = 0;
  size_t n_writes = 0;

private:
  std::array<uint8_t, BUFFER_SIZE> buffer;
  std::array<uint8_t, STUFFED_BUFFER_SIZE> stuffed_buffer;
  uint16_t buffer_idx;
  uint16_t stuffed_idx;
  const uint32_t baud_rate;
  std::chrono::time_point<std::chrono::steady_clock> start_time;
};