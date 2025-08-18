#include "departing_speed_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2415h {

static const char *const TAG = "LD2415H.departing_speed";

void DepartingSpeedSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2415H Departing Speed Sensor:");
  LOG_SENSOR("  ", "Departing Speed", this);
}

}  // namespace ld2415h
}  // namespace esphome