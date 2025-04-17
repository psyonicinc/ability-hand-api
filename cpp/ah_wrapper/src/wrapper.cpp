#include "wrapper.h"

#include <iostream>

#ifdef PLATFORM_WINDOWS
#include "winserial.h"
#elif defined(PLATFORM_LINUX)
#include "linux_serial.h"
#endif

#include "api.h"
#include "parser.h"
#include "serial_helper.h"

AHWrapper::AHWrapper(const uint8_t &hand_addr, const uint32_t &b_rate)
    : hand(hand_addr), baud_rate(b_rate) {}

AHWrapper::~AHWrapper() {
  printf("Closing connection to Hand %d\n", hand.address);
  std::chrono::duration<double> duration = std::chrono::steady_clock::now() - start_time;
  std::cout << "Rate: " << n_reads / duration.count() << std::endl;
  std::cout << "# of Reads: " << n_reads << "\n# Of Writes: " << n_writes
            << std::endl;
  std::cout << "Dropped Packet %: "
            << (1.0 -
                (static_cast<float>(n_reads) / static_cast<float>(n_writes))) *
                   100.0
            << std::endl;
  close_serial();
}

int AHWrapper::connect() {
  start_time = std::chrono::steady_clock::now();
  if (autoconnect_serial(baud_rate)) {
    return 1; // Could not connect
  } else {
    printf("Connected to Hand %d\n", hand.address);
    return 0;
  }
}

int AHWrapper::read_write_once(const std::array<float, 6> &cmd_values,
                               const Command &cmd, const uint8_t &reply_mode) {
  switch (cmd) {
  case POSITION:
    buffer_idx = build_pos_msg(cmd_values, buffer, hand.address, reply_mode);
    break;
  case VELOCITY:
    buffer_idx = build_vel_msg(cmd_values, buffer, hand.address, reply_mode);
    break;
  case CURRENT:
    buffer_idx = build_tor_msg(cmd_values, buffer, hand.address, reply_mode);
    break;
  case DUTY:
    buffer_idx = build_dut_msg(cmd_values, buffer, hand.address, reply_mode);
    break;
  }

  stuffed_idx = ppp_stuff(buffer.data(), buffer_idx, stuffed_buffer.data(),
                          STUFFED_BUFFER_SIZE);
  serial_write(stuffed_buffer.data(), stuffed_idx);
  ++n_writes; // Can't determine if write fails or succeeds

  int unstuffed_bytes_read = read_until(stuffed_buffer, buffer);
  if (unstuffed_bytes_read > 0) {
    // Response received, unstuffed and passed checksum
    ++n_reads;
    parse_packet(buffer, unstuffed_bytes_read, hand, reply_mode);
    // printf("%f %f %f %f %f %f\n", hand.pos[0], hand.pos[1], hand.pos[2],
    //        hand.pos[3], hand.pos[4], hand.pos[5]);
  }

  return 0;
}