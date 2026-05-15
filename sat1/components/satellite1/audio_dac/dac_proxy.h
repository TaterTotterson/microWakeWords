#pragma once

#include "esphome/components/audio_dac/audio_dac.h"
#include "esphome/core/component.h"
#include "esphome/components/satellite1/satellite1.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace satellite1 {

static const uint8_t AUDIO_SERVICER_CMD_SET_DAC = 0x00;

enum DacOutput : uint8_t {
  SPEAKER = 0,
  LINE_OUT,
};

struct DACProxyRestoreState {
  uint8_t dac_output;
  float speaker_volume;
  bool speaker_is_muted;
  float line_out_volume;
  bool line_out_is_muted;
};

class DACProxy : public audio_dac::AudioDac, public Component, public Satellite1SPIService, public EntityBase {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  bool set_mute_off() override;
  bool set_mute_on() override;
  bool set_volume(float volume) override;

  bool is_muted() override;
  float volume() override;

  void set_lineout_dac(audio_dac::AudioDac *pcm5122) { this->pcm5122_ = pcm5122; }
  void set_speaker_dac(audio_dac::AudioDac *tas2780) { this->tas2780_ = tas2780; }
  template<typename F> void add_on_state_callback(F &&callback) {
    this->state_callback_.add(std::forward<F>(callback));
  }

  void activate();
  void activate_line_out();
  void activate_speaker();

  DacOutput active_dac{SPEAKER};

 protected:
  bool setup_was_called_{false};
  ESPPreferenceObject pref_;
  DACProxyRestoreState restore_state_;
  void save_volume_restore_state_();

  void send_selected_dac_() {}

  audio_dac::AudioDac *pcm5122_{nullptr};
  audio_dac::AudioDac *tas2780_{nullptr};

  CallbackManager<void()> state_callback_{};
};

}  // namespace satellite1
}  // namespace esphome