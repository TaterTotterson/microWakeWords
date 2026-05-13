#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/light/addressable_light.h"

#include "esphome/components/satellite1/satellite1.h"

namespace esphome {
namespace satellite1 {

const uint32_t NUMBER_OF_LEDS = 24;

const uint32_t LED_RES_ID = 200;
const uint32_t CMD_WRITE_LED_RING_RAW = 0;

class LEDRing : public light::AddressableLight, public Satellite1SPIService {
 public:
  LEDRing() : num_leds_(NUMBER_OF_LEDS) {}

  int32_t size() const override { return this->num_leds_; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  light::LightTraits get_traits() override {
    auto traits = light::LightTraits();
    traits.set_supported_color_modes({light::ColorMode::RGB});
    return traits;
  }

  void write_state(light::LightState *state) override;

  void clear_effect_data() override {
    for (int i = 0; i < this->size(); i++)
      this->effect_data_[i] = 0;
  }

 protected:
  light::ESPColorView get_view_internal(int32_t index) const override;

 private:
  size_t buffer_size_{};
  uint8_t *effect_data_{nullptr};
  uint8_t *buf_{nullptr};
  int32_t num_leds_;
};

}  // namespace satellite1
}  // namespace esphome
