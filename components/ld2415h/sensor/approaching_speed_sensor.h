#pragma once

#include "../ld2415h.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ld2415h {

class ApproachingSpeedSensor : public Component, public sensor::Sensor {
 public:
  void dump_config() override;
  void set_approaching_speed_sensor(sensor::Sensor *sensor) { this->approaching_speed_sensor_ = sensor; }
  void set_parent(LD2415HComponent *parent) { this->parent_ = parent; }   
  void publish_speed(double speed) {
    this->publish_state(speed);
  }
 protected:
  sensor::Sensor *approaching_speed_sensor_{nullptr};
  LD2415HComponent *parent_{nullptr};
};

}  // namespace ld2415h
}  // namespace esphome