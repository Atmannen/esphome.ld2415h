#include "approaching_last_max_speed_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2415h {

static const char *const TAG = "LD2415H.approaching_last_max_speed";

void ApproachingLastMaxSpeedSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2415H Approaching Last Max Speed:");
  LOG_SENSOR("  ", "Approaching Last Max Speed", this);
}

}  // namespace ld2415h
}  // namespace esphome