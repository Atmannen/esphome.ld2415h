#include "departing_vehicle_count_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2415h {

static const char *const TAG = "ld2415h.departing_vehicle_count";

void DepartingVehicleCountSensor::dump_config() {
  LOG_SENSOR("", "LD2415H Departing Vehicle Count", this);
}

}  // namespace ld2415h
}  // namespace esphome
