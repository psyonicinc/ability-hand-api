#pragma once

#include "api.h"
#include "hand.h"
#include <cstdint>

int parse_packet(uint8_t *data, const uint16_t &packet_size, Hand &hand,
                 const uint8_t &r_mode);
