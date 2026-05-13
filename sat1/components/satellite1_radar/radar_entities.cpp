#include "radar_entities.h"

namespace esphome {
namespace satellite1_radar {

void Satellite1RadarTunerSwitch::write_state(bool state) {
  this->publish_state(state);
  if (this->write_callback_)
    this->write_callback_(state);
}

}  // namespace satellite1_radar
}  // namespace esphome
