#pragma once
#include <array>
#include <stdio.h>

#include "ppp.h"

const uint32_t MAX_ATTEMPTS = 150000;

bool compute_checksum(uint8_t buffer, uint16_t &buffer_size);

int read_until(uint8_t *stuffed_buffer, uint8_t *buffer,
               const uint16_t &stuffed_buffer_size,
               const uint16_t &buffer_size);
