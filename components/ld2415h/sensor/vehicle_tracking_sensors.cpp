#include "vehicle_tracking_sensors.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2415h {

static const char *const TAG = "ld2415h.vehicle_tracking";

void ApproachingVehicleCountSensor::dump_config() {
  LOG_SENSOR("", "LD2415H Approaching Vehicle Count Sensor", this);
}

void DepartingVehicleCountSensor::dump_config() {
  LOG_SENSOR("", "LD2415H Departing Vehicle Count Sensor", this);
}

void CurrentApproachingVehicleSpeedSensor::dump_config() {
  LOG_SENSOR("", "LD2415H Current Approaching Vehicle Speed Sensor", this);
}

void CurrentDepartingVehicleSpeedSensor::dump_config() {
  LOG_SENSOR("", "LD2415H Current Departing Vehicle Speed Sensor", this);
}

}  // namespace ld2415h
}  // namespace esphome
