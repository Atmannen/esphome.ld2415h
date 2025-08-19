#include "current_departing_vehicle_speed_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2415h {

static const char *const TAG = "ld2415h.current_departing_vehicle_speed";

void CurrentDepartingVehicleSpeedSensor::dump_config() {
  LOG_SENSOR("", "LD2415H Current Departing Vehicle Speed", this);
}

}  // namespace ld2415h
}  // namespace esphome
