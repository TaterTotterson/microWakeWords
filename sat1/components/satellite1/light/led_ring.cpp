#include "led_ring.h"

namespace esphome {
namespace satellite1 {

static const char *const TAG = "LED-Ring";

void LEDRing::setup() {
  ExternalRAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);
  this->buffer_size_ = this->size() * 3;
  this->buf_ = allocator.allocate(this->buffer_size_);
  if (this->buf_ == nullptr) {
    esph_log_e(TAG, "Failed to allocate buffer of size %u", this->buffer_size_);
    this->mark_failed();
    return;
  }
  this->effect_data_ = allocator.allocate(this->size());
  if (this->effect_data_ == nullptr) {
    esph_log_e(TAG, "Failed to allocate effect data of size %u", this->num_leds_);
    this->mark_failed();
    return;
  }
  memset(this->buf_, 0, this->buffer_size_);
}

float LEDRing::get_setup_priority() const { return setup_priority::HARDWARE; }

void LEDRing::dump_config() {
  esph_log_config(TAG, "Satellite1 LED-Ring:");
  esph_log_config(TAG, "  LEDs: %d", this->num_leds_);
}

void LEDRing::write_state(light::LightState *state) {
  if (this->is_failed()) {
    return;
  }
  this->parent_->transfer(LED_RES_ID, CMD_WRITE_LED_RING_RAW, this->buf_, this->buffer_size_);
}

light::ESPColorView LEDRing::get_view_internal(int32_t index) const {
  size_t pos = index * 3;
  return {this->buf_ + pos + 1,  // r
          this->buf_ + pos + 0,  // g
          this->buf_ + pos + 2,  // b
          0,                     // no separate white channel
          this->effect_data_ + index,
          &this->correction_};
}

}  // namespace satellite1
}  // namespace esphome
