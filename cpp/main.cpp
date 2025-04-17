#include <array>
#include <stdio.h>

#include "wrapper.h"

int main(int argc, char *argv[]) {
  AHWrapper wrapper = AHWrapper(0x50, 921600);
  wrapper.connect();

  // Send Position Command [0,100]
  std::array<float, 6> cmd = {30.0, 30.0, 30.0, 30.0, 30.0, -30};
  for (size_t i = 0; i < 250; ++i) {
    wrapper.read_write_once(cmd, POSITION, 0);
    for (float &val : cmd) {
      val += 0.1f;
    }
    for (uint8_t j = 0; j < 6; ++j) {
      printf("%f ", wrapper.hand.pos[j]);
    }
    printf("\n");

  }
  
  // Send Velocity Command 
  cmd = {-20.0, -20.0, -20.0, -20.0, -20.0, -20.0};
  for (size_t i = 0; i < 500; ++i) {
    wrapper.read_write_once(cmd, VELOCITY, 1);
    for (uint8_t j = 0; j < 6; ++j) {
      printf("%f ", wrapper.hand.vel[j]);
    }
    printf("\n");
  }

  // Send Current Command
  cmd = {.1, .1, .1, .1, .1, .1};
  for (size_t i = 0; i < 500; ++i) {
    wrapper.read_write_once(cmd, CURRENT, 0);
    for (uint8_t j = 0; j < 6; ++j) {
      printf("%f ", wrapper.hand.cur[j]);
    }
    printf("\n");
  }

  // Send Duty Command
  cmd = {-10.0, -10.0, -10.0, -10.0, -10.0, 10.0};
  for (size_t i = 0; i < 500; ++i) {
    wrapper.read_write_once(cmd, DUTY, 0);
  }

  // Verify Index FSR
  cmd = {0.0, 30.0, 30.0, 30.0, 30.0, -30.0};
  for (size_t i = 0; i < 10000; ++i) {
    wrapper.read_write_once(cmd, POSITION, 0);
    for (uint8_t j = 0; j < 6; ++j) {
      printf("%d ", wrapper.hand.fsr[j]);
    }
    printf("\n");
  }

  return 0;
}
