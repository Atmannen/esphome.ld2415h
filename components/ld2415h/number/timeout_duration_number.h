#pragma once
#include "esphome/components/number/number.h"
#include "../ld2415h.h"

namespace esphome {
namespace ld2415h {

class TimeoutDurationNumber : public number::Number, public Parented<LD2415HComponent> {
  public:
  TimeoutDurationNumber() = default;

 protected:
  void control(float ms) override;
};

}  // namespace ld2415h
}  // namespace esphome
