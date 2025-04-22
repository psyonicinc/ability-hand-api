#include "api.h"

float map_value(float val, float in_min, float in_max, float out_min,
                float out_max) {
  float mapped =
      (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return std::max(out_min, std::min(out_max, mapped));
}