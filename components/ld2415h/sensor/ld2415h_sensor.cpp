#include "ld2415h_sensor.h"

namespace esphome {
namespace ld2415h {

static const char *const TAG = "ld2415h.sensor";

void LD2415HSensor::dump_config() {
  LOG_SENSOR("", "LD2415H Sensor", this);
}

}  // namespace ld2415h
}  // namespace esphome
