#include "pcm5122.h"

#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pcm5122 {

static const char *const TAG = "pcm5122";

static const uint8_t PCM5122_REG00_PAGE_SELECT = 0x00;  // Page Select

void PCM5122::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PCM5122...");
  // select page 0
  this->reg(PCM5122_REG00_PAGE_SELECT) = 0x00;

  uint8_t chd1 = this->reg(0x09).get();
  uint8_t chd2 = this->reg(0x10).get();
  if (chd1 == 0x00 && chd2 == 0x00) {
    ESP_LOGD(TAG, "PCM5122 chip found.");
  } else {
    ESP_LOGD(TAG, "PCM5122 chip not found.");
    this->mark_failed();
    return;
  }

  // RESET
  this->reg(0x01) = 0x10;
  delay(20);
  this->reg(0x01) = 0x00;

  uint8_t err_detect = this->reg(0x25).get();
  // set 'Ignore Clock Halt Detection'
  err_detect |= (1 << 3);
  // enable Clock Divider Autoset
  err_detect &= ~(1 << 1);
  this->reg(0x25) = err_detect;

  // set 32bit - I2S
  this->reg(0x28) = 3;  // 32bits

  // 001: The PLL reference clock is BCK
  uint8_t pll_ref = this->reg(0x0D).get();
  pll_ref &= ~(7 << 4);
  pll_ref |= (1 << 4);
  this->reg(0x0D) = pll_ref;

  this->set_mute_on();
}

void PCM5122::dump_config() {}

bool PCM5122::set_mute_off() {
  this->is_muted_ = false;
  return this->write_mute_();
}

bool PCM5122::set_mute_on() {
  this->is_muted_ = true;
  return this->write_mute_();
}

bool PCM5122::set_volume(float volume) {
  this->volume_ = clamp<float>(volume, 0.0, 1.0);
  return this->write_volume_();
}

bool PCM5122::is_muted() { return this->is_muted_; }

float PCM5122::volume() { return this->volume_; }

bool PCM5122::write_mute_() {
  uint8_t mute_byte = this->is_muted() ? 0x11 : 0x00;
  if (!this->write_byte(PCM5122_REG00_PAGE_SELECT, 0x00) || !this->write_byte(0x03, mute_byte)) {
    ESP_LOGE(TAG, "Writing mute failed");
    return false;
  }
  return true;
}

bool PCM5122::write_volume_() {
  const uint8_t dvc_min_byte = 0x44;  //   0x00: 24 dB ; 0x30:   0dB
  const uint8_t dvc_max_byte = 0x99;  //   0xFF:  mute ; 0x94: -50dB

  const uint8_t volume_byte = dvc_min_byte + ((1. - this->volume_) * (dvc_max_byte - dvc_min_byte) + 0.5);

  ESP_LOGD(TAG, "Setting volume to 0x%.2x", volume_byte & 0xFF);

  if ((!this->write_byte(PCM5122_REG00_PAGE_SELECT, 0x00)) || (!this->write_byte(0x3D, volume_byte)) ||
      (!this->write_byte(0x3E, volume_byte))) {
    ESP_LOGE(TAG, "Writing volume failed");
    return false;
  }
  return true;
}

}  // namespace pcm5122
}  // namespace esphome
