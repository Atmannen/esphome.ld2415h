#include "approaching_speed_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2415h {

static const char *const TAG = "LD2415H.approaching_speed";

void ApproachingSpeedSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2415H Approaching Speed Sensor:");
  LOG_SENSOR("  ", "Approaching Speed", this);
}

}  // namespace ld2415h
}  // namespace esphome