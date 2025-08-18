#include "timeout_duration_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2415h {

static const char *TAG = "ld2415h.timeout_duration";

void TimeoutDurationNumber::control(float ms) {
  this->publish_state(ms);
  this->parent_->set_timeout_duration((uint32_t)ms);
}

}  // namespace ld2415h
}  // namespace esphome
