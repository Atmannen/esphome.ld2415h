#include "departing_last_max_speed_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2415h {

static const char *const TAG = "LD2415H.departing_last_max_speed";

void DepartingLastMaxSpeedSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2415H Departing Last Max Speed:");
  LOG_SENSOR("  ", "Departing Last Max Speed", this);
}

}  // namespace ld2415h
}  // namespace esphome