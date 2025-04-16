#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <unistd.h>

#ifdef PLATFORM_WINDOWS
#include "winserial.h"
#elif defined(PLATFORM_LINUX)
#include "linux_serial.h"
#endif
#include "api.h"
#include "hand.h"
#include "parser.h"
#include "ppp.h"
#include "serial_helper.h"

// CONSTANTS
const uint8_t HAND_ADDRESS = 0x50;

const uint16_t BUFFER_SIZE = 512;
const uint16_t STUFFED_BUFFER_SIZE = BUFFER_SIZE * 2;

int main(int argc, char *argv[]) {
  // Buffer variables
  std::array<uint8_t, BUFFER_SIZE> buffer;
  std::array<uint8_t, STUFFED_BUFFER_SIZE> stuffed_buffer;
  uint16_t buffer_idx;
  uint16_t stuffed_idx;
  int unstuffed_bytes_read;

  // Cmd variables
  std::array<float, 6> cmd;
  uint8_t reply_mode = 0;

  // Stats variables
  size_t n_reads = 0;
  size_t n_writes = 0;

  // Hand class
  Hand hand = Hand(HAND_ADDRESS);

  // Connect to hand
  if (autoconnect_serial()) {
    return 1; // Could not connect
  } else {
    printf("Connected to Hand %d\n", hand.address);
  }

  auto start = std::chrono::high_resolution_clock::now();
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration<double>(now.time_since_epoch());
  cmd = {30.0, 30.0, 30.0, 30.0, 30.0, -30};

  // Example of a synchronous write / read loop
  for (size_t i = 0; i < 100000; i++) {

    // Calculate Hand Wave
    now = std::chrono::steady_clock::now();
    duration = std::chrono::duration<double>(now.time_since_epoch());
    for (size_t j = 0; j < cmd.size(); ++j) {
      double ft =
          static_cast<double>(duration.count()) * 3.0 + j * (2.0 * M_PI / 12.0);
      cmd[j] = (0.5 * std::sin(ft) + 0.5) * 45.0 + 15.0;
    }
    cmd[5] = -cmd[5];

    // Build POS message
    buffer_idx = build_pos_msg(cmd, buffer, HAND_ADDRESS, reply_mode);

    // Byte stuff POS message
    stuffed_idx = ppp_stuff(buffer.data(), buffer_idx, stuffed_buffer.data(),
                            STUFFED_BUFFER_SIZE);

    // Write to Serial
    serial_write(stuffed_buffer.data(), stuffed_idx);
    ++n_writes; // Can't determine if write fails or succeeds

    // Read until full stuffed return frame detected
    unstuffed_bytes_read = read_until(stuffed_buffer, buffer);
    if (unstuffed_bytes_read > 0) {
      // Response received, unstuffed and passed checksum
      ++n_reads;
      parse_packet(buffer, unstuffed_bytes_read, hand, reply_mode);
      // printf("%f %f %f %f %f %f\n", hand.pos[0], hand.pos[1], hand.pos[2],
      // hand.pos[3], hand.pos[4], hand.pos[5]);
    }
  }

  // Print statistics
  duration = std::chrono::high_resolution_clock::now() - start;
  std::cout << "Rate: " << n_reads / duration.count() << std::endl;
  std::cout << "# of Reads: " << n_reads << "\n# Of Writes: " << n_writes
            << std::endl;
  std::cout << "Dropped Packet %: "
            << (1.0 -
                (static_cast<float>(n_reads) / static_cast<float>(n_writes))) *
                   100.0
            << std::endl;
  close_serial();

  return 0;
}
