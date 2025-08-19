#pragma once

#include "../ld2415h.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ld2415h {

class LD2415HSensor : public Component, public sensor::Sensor {
 public:
  void dump_config() override;
  void set_speed_sensor(sensor::Sensor *sensor) { this->speed_sensor_ = sensor; }
  void set_parent(LD2415HComponent *parent) { this->parent_ = parent; }

 protected:
  sensor::Sensor *speed_sensor_{nullptr};
  LD2415HComponent *parent_{nullptr};
};

}  // namespace ld2415h
}  // namespace esphome
