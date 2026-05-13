#include "dac_proxy.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1 {

static const char *const TAG = "dac_proxy";

void DACProxy::setup() {
  ESP_LOGD(TAG, "Setting up DACProxy...");
  this->pref_ = this->make_entity_preference<DACProxyRestoreState>();

  if (this->pref_.load(&this->restore_state_)) {
    ESP_LOGD(TAG, "Read preferences from flash");
    if (this->pcm5122_) {
      this->pcm5122_->set_volume(this->restore_state_.line_out_volume);
      if (this->restore_state_.line_out_is_muted) {
        this->pcm5122_->set_mute_on();
      }
    }
    if (this->tas2780_) {
      this->tas2780_->set_volume(this->restore_state_.speaker_volume);
      if (this->restore_state_.speaker_is_muted) {
        this->tas2780_->set_mute_on();
      }
    }
    ESP_LOGD(TAG, "   active dac: %d", this->restore_state_.dac_output);
    this->active_dac = (DacOutput) this->restore_state_.dac_output;
    this->activate();
  } else {
    ESP_LOGW(TAG, "Preferences not found, using default settings");
    this->active_dac = LINE_OUT;
    this->restore_state_.dac_output = LINE_OUT;
    this->restore_state_.speaker_volume = .5;
    this->restore_state_.speaker_is_muted = false;
    this->restore_state_.line_out_volume = .5;
    this->restore_state_.line_out_is_muted = false;
    if (this->pcm5122_) {
      this->pcm5122_->set_volume(this->restore_state_.line_out_volume);
    }
    if (this->tas2780_) {
      this->tas2780_->set_volume(this->restore_state_.speaker_volume);
    }
  }
  this->setup_was_called_ = true;
  this->defer([this]() { this->state_callback_.call(); });
}

void DACProxy::dump_config() {
  if (this->tas2780_) {
    esph_log_config(TAG, "SPEAKER-DAC, volume: %4.2f, muted: %s %s", this->tas2780_->volume(),
                    this->restore_state_.speaker_is_muted ? "true" : "false",
                    this->active_dac == SPEAKER ? "(active)" : "");
  }
  if (this->pcm5122_) {
    esph_log_config(TAG, "LINE-OUT-DAC, volume: %4.2f, muted: %s %s", this->pcm5122_->volume(),
                    this->restore_state_.line_out_is_muted ? "true" : "false",
                    this->active_dac == LINE_OUT ? "(active)" : "");
  }
}

void DACProxy::save_volume_restore_state_() {
  ESP_LOGD(TAG, "Saving volume restore state...");
  ESP_LOGD(TAG, "Active DAC: %d", this->active_dac);

  this->restore_state_.dac_output = this->active_dac;
  if (this->active_dac == LINE_OUT && this->pcm5122_) {
    this->restore_state_.line_out_volume = this->pcm5122_->volume();
  }
  if (this->active_dac == SPEAKER && this->tas2780_) {
    this->restore_state_.speaker_volume = this->tas2780_->volume();
  }
  this->pref_.save(&this->restore_state_);
}

void DACProxy::activate_line_out() {
  if (this->pcm5122_ == nullptr) {
    return;
  }
  ESP_LOGD(TAG, "Activate Line-Out DAC.");
  this->active_dac = LINE_OUT;

  if (this->tas2780_) {
    this->tas2780_->set_mute_on();
  }
  if (!this->restore_state_.line_out_is_muted) {
    this->pcm5122_->set_mute_off();
  }
  this->send_selected_dac_();
  this->defer([this]() { this->state_callback_.call(); });
  this->save_volume_restore_state_();
}

void DACProxy::activate_speaker() {
  if (this->tas2780_ == nullptr) {
    return;
  }
  ESP_LOGD(TAG, "Activate Speaker DAC.");
  this->active_dac = SPEAKER;
  this->send_selected_dac_();
  if (this->pcm5122_) {
    this->pcm5122_->set_mute_on();
  }
  if (!this->restore_state_.speaker_is_muted) {
    this->tas2780_->set_mute_off();
  }
  this->defer([this]() { this->state_callback_.call(); });
  this->save_volume_restore_state_();
}

void DACProxy::activate() {
  if (this->active_dac == SPEAKER && this->tas2780_) {
    if (this->pcm5122_) {
      this->pcm5122_->set_mute_on();
    }
    if (!this->restore_state_.speaker_is_muted) {
      this->tas2780_->set_mute_off();
    }
  } else if (this->pcm5122_) {
    if (this->tas2780_) {
      this->tas2780_->set_mute_on();
    }
    if (!this->restore_state_.line_out_is_muted) {
      this->pcm5122_->set_mute_off();
    }
  }
}

bool DACProxy::set_mute_off() {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::set_mute_off() called before setup()");
    return false;
  }
  bool has_changed = false;
  bool ret = false;
  if (this->active_dac == LINE_OUT && this->pcm5122_ && this->pcm5122_->is_muted()) {
    ret = this->pcm5122_->set_mute_off();
    this->restore_state_.line_out_is_muted = false;
    has_changed = true;
  }
  if (this->active_dac == SPEAKER && this->tas2780_ && this->tas2780_->is_muted()) {
    ret = this->tas2780_->set_mute_off();
    this->restore_state_.speaker_is_muted = false;
    has_changed = true;
  }
  if (has_changed) {
    ESP_LOGD(TAG, "set_mute_off: for %s", this->active_dac == LINE_OUT ? "Line-Out" : "Speaker");
    this->save_volume_restore_state_();
  }
  return ret;
}

bool DACProxy::set_mute_on() {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::set_mute_on() called before setup()");
    return false;
  }
  bool has_changed = false;
  bool ret = false;
  if (this->active_dac == LINE_OUT && this->pcm5122_ && !this->pcm5122_->is_muted()) {
    ret = this->pcm5122_->set_mute_on();
    this->restore_state_.line_out_is_muted = true;
    has_changed = true;
  }
  if (this->active_dac == SPEAKER && this->tas2780_ && !this->tas2780_->is_muted()) {
    ret = this->tas2780_->set_mute_on();
    this->restore_state_.speaker_is_muted = true;
    has_changed = true;
  }
  if (has_changed) {
    ESP_LOGD(TAG, "set_mute_on: for %s", this->active_dac == LINE_OUT ? "Line-Out" : "Speaker");
    this->save_volume_restore_state_();
  }
  return ret;
}

bool DACProxy::set_volume(float volume) {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::set_volume() called before setup()");
    return false;
  }
  bool has_changed = false;
  bool ret = false;
  if (this->active_dac == LINE_OUT && this->pcm5122_ && this->pcm5122_->volume() != volume) {
    ret = this->pcm5122_->set_volume(volume);
    has_changed = true;
  } else if (this->active_dac == SPEAKER && this->tas2780_ && this->tas2780_->volume() != volume) {
    ret = this->tas2780_->set_volume(volume);
    has_changed = true;
  }
  if (has_changed) {
    this->save_volume_restore_state_();
  }
  return ret;
}

bool DACProxy::is_muted() {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::is_muted() called before setup()");
    return false;
  }
  if (this->active_dac == LINE_OUT && this->pcm5122_) {
    return this->pcm5122_->is_muted();
  }
  if (this->active_dac == SPEAKER && this->tas2780_) {
    return this->tas2780_->is_muted();
  }
  return false;
}

float DACProxy::volume() {
  if (this->setup_was_called_ == false) {
    ESP_LOGD(TAG, "DACProxy::volume() called before setup()");
    return .5;
  }
  if (this->active_dac == LINE_OUT && this->pcm5122_) {
    return this->pcm5122_->volume();
  }
  if (this->active_dac == SPEAKER && this->tas2780_) {
    return this->tas2780_->volume();
  }
  return 0.;
}

}  // namespace satellite1
}  // namespace esphome