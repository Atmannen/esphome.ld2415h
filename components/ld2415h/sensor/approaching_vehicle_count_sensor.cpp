#include "approaching_vehicle_count_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2415h {

static const char *const TAG = "ld2415h.approaching_vehicle_count";

void ApproachingVehicleCountSensor::dump_config() {
  LOG_SENSOR("", "LD2415H Approaching Vehicle Count", this);
}

}  // namespace ld2415h
}  // namespace esphome
