#pragma once

#include "../ld2415h.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ld2415h {

class ApproachingVehicleCountSensor : public Component, public sensor::Sensor {
 public:
  void dump_config() override;
  void set_parent(LD2415HComponent *parent) { this->parent_ = parent; }

 protected:
  LD2415HComponent *parent_{nullptr};
};

}  // namespace ld2415h
}  // namespace esphome
