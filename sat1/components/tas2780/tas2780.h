#pragma once

#include "esphome/components/audio_dac/audio_dac.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace tas2780 {

enum ChannelSelect : uint8_t { MONO_DWN_MIX, LEFT_CHANNEL, RIGHT_CHANNEL };

class TAS2780 : public audio_dac::AudioDac, public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;

  void init();
  void reset();
  void activate(uint8_t power_mode = 2);
  void deactivate();
  void update_register();
  void log_error_states();

  bool set_mute_off() override;
  bool set_mute_on() override;
  bool set_volume(float volume) override;

  bool is_muted() override;
  float volume() override;

  void set_amp_level(uint8_t amp_level) { this->amp_level_ = amp_level; }
  void set_vol_range_min(float min_val) { this->vol_range_min_ = min_val; }
  void set_vol_range_max(float max_val) { this->vol_range_max_ = max_val; }
  void set_selected_channel(ChannelSelect channel) { this->selected_channel_ = channel; }

 protected:
  void set_power_mode_(const uint8_t power_mode);
  bool write_mute_();
  bool write_volume_();

  float volume_{0};
  uint8_t power_mode_{2};
  uint8_t amp_level_{8};
  float vol_range_min_{.3};
  float vol_range_max_{1.};
  ChannelSelect selected_channel_{MONO_DWN_MIX};
};

}  // namespace tas2780
}  // namespace esphome