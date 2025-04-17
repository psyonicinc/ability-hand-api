#pragma once
#include <array>
#include <cstring>
#include <stdint.h>

const float DEGREES_CONSTANT = 32767.0 / 150.0;
const float DEGREES_CONSTANT_INV = 150.0 / 32767.0;
const float VELOCITY_CONSTANT = 32767.0 / 3000.0;
const float VELOCITY_CONSTANT_INV = 3000.0 / 32767.0;
const float LIMIT = 32767.0;
const float VOLTAGE_LIMIT = 3546.0;
const float C = 620.606079;

float map_value(float val, float in_min, float in_max, float out_min,
                float out_max);

template <std::size_t N>
uint16_t build_pos_msg(const std::array<float, 6> &position,
                       std::array<uint8_t, N> &cmd_array, const uint8_t &addr,
                       const uint8_t &reply_mode) {
  uint32_t checksum = 0;
  uint16_t idx = 0;
  cmd_array[idx] = addr;
  checksum += cmd_array[idx++];

  cmd_array[idx] = 0x10 + reply_mode;
  checksum += cmd_array[idx++];

  for (float p : position) {
    int16_t p_scaled = static_cast<int16_t>(
        std::max(std::min(p * DEGREES_CONSTANT, LIMIT), -LIMIT));
    uint8_t p_bytes[2];
    std::memcpy(p_bytes, &p_scaled, sizeof(p_scaled));
    cmd_array[idx] = p_bytes[0];
    checksum += cmd_array[idx++];
    cmd_array[idx] = p_bytes[1];
    checksum += cmd_array[idx++];
  }

  cmd_array[idx++] = -checksum & 0xFF;

  return idx;
}

template <std::size_t N>
uint16_t build_vel_msg(const std::array<float, 6> &velocities,
                       std::array<uint8_t, N> &cmd_array, const uint8_t &addr,
                       const uint8_t &reply_mode) {
  uint32_t checksum = 0;
  uint16_t idx = 0;
  cmd_array[idx] = addr;
  checksum += cmd_array[idx++];

  cmd_array[idx] = 0x20 + reply_mode;
  checksum += cmd_array[idx++];

  for (float v : velocities) {
    int16_t v_scaled = static_cast<int16_t>(v * VELOCITY_CONSTANT);
    uint8_t v_bytes[2];
    std::memcpy(v_bytes, &v_scaled, sizeof(v_scaled));
    cmd_array[idx] = v_bytes[0];
    checksum += cmd_array[idx++];
    cmd_array[idx] = v_bytes[1];
    checksum += cmd_array[idx++];
  }

  cmd_array[idx++] = -checksum & 0xFF;

  return idx;
}

template <std::size_t N>
uint16_t build_tor_msg(const std::array<float, 6> &torques,
                       std::array<uint8_t, N> &cmd_array, const uint8_t &addr,
                       const uint8_t &reply_mode) {
  uint32_t checksum = 0;
  uint16_t idx = 0;
  cmd_array[idx] = addr;
  checksum += cmd_array[idx++];

  cmd_array[idx] = 0x30 + reply_mode;
  checksum += cmd_array[idx++];

  for (float t : torques) {
    int16_t t_scaled = static_cast<int16_t>(t * C);
    uint8_t t_bytes[2];
    std::memcpy(t_bytes, &t_scaled, sizeof(t_scaled));
    cmd_array[idx] = t_bytes[0];
    checksum += cmd_array[idx++];
    cmd_array[idx] = t_bytes[1];
    checksum += cmd_array[idx++];
  }

  cmd_array[idx++] = -checksum & 0xFF;

  return idx;
}

template <std::size_t N>
uint16_t build_dut_msg(const std::array<float, 6> &duties,
                       std::array<uint8_t, N> &cmd_array, const uint8_t &addr,
                       const uint8_t &reply_mode) {
  uint32_t checksum = 0;
  uint16_t idx = 0;
  cmd_array[idx] = addr;
  checksum += cmd_array[idx++];

  cmd_array[idx] = 0x40 + reply_mode;
  checksum += cmd_array[idx++];

  for (float d : duties) {
    int16_t d_scaled = static_cast<int16_t>(
        map_value(d, -100, 100, -VOLTAGE_LIMIT, VOLTAGE_LIMIT));
    uint8_t d_bytes[2];
    std::memcpy(d_bytes, &d_scaled, sizeof(d_scaled));
    cmd_array[idx] = d_bytes[0];
    checksum += cmd_array[idx++];
    cmd_array[idx] = d_bytes[1];
    checksum += cmd_array[idx++];
  }

  cmd_array[idx++] = -checksum & 0xFF;

  return idx;
}

template <std::size_t N>
uint16_t build_misc_msg(const uint8_t &cmd, std::array<uint8_t, N> &cmd_array,
                        const uint8_t &addr) {
  uint32_t checksum = 0;
  uint16_t idx = 0;
  cmd_array[idx] = addr;
  checksum += cmd_array[idx++];

  cmd_array[idx] = cmd;
  checksum += cmd_array[idx++];

  cmd_array[idx++] = -checksum & 0xFF;

  return idx;
}

template <std::size_t N>
uint16_t build_grip_msg(const uint8_t &cmd, const uint8_t &speed,
                        std::array<uint8_t, N> &cmd_array,
                        const uint8_t &addr) {
  uint32_t checksum = 0;
  uint16_t idx = 0;
  cmd_array[idx] = addr;
  checksum += cmd_array[idx++];

  cmd_array[idx] = 0x1D;
  checksum += cmd_array[idx++];

  cmd_array[idx] = cmd;
  checksum += cmd_array[idx++];

  cmd_array[idx] = speed;
  checksum += cmd_array[idx++];

  cmd_array[idx++] = -checksum & 0xFF;

  return idx;
}