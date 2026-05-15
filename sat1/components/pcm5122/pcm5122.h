#pragma once

#include "esphome/components/audio_dac/audio_dac.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace pcm5122 {

class PCM5122 : public audio_dac::AudioDac, public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;

  bool set_mute_off() override;
  bool set_mute_on() override;
  bool set_volume(float volume) override;

  bool is_muted() override;
  float volume() override;

 protected:
  bool write_mute_();
  bool write_volume_();

  float volume_{0};
};

}  // namespace pcm5122
}  // namespace esphome
