#include "pcm_gpio.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pcm5122 {

static const char *TAG = "PCM-GPIOs";

void PCMGPIOPin::setup() {
  // set input/output mode for pin in register 0x08
  uint8_t curr = this->parent_->reg(0x08).get();
  if (this->flags_ & gpio::FLAG_INPUT) {
    this->parent_->reg(0x08) = curr & ~(1 << (this->pin_ - 1));
  } else if (this->flags_ & gpio::FLAG_OUTPUT) {
    this->parent_->reg(0x08) = curr | (1 << (this->pin_ - 1));
    // set pin to be used as GPIO
    this->parent_->reg(0x50 + (this->pin_ - 1)) = 0x02;
    // set / clear inversion bit
    curr = this->parent_->reg(0x57).get();
    if (this->inverted_) {
      this->parent_->reg(0x57) = curr | (1 << (this->pin_ - 1));
    } else {
      this->parent_->reg(0x57) = curr & ~(1 << (this->pin_ - 1));
    }
  }
}

void PCMGPIOPin::digital_write(bool value) {
  uint8_t curr = this->parent_->reg(0x56).get();
  if (value) {
    this->parent_->reg(0x56) = curr | (1 << (this->pin_ - 1));
  } else {
    this->parent_->reg(0x56) = curr & ~(1 << (this->pin_ - 1));
  }
}

bool PCMGPIOPin::digital_read() {
  optional<uint8_t> read = this->parent_->read_byte(0x77);
  if (read.has_value()) {
    this->value_ = !!(read.value() & (1 << (this->pin_ - 1))) != this->inverted_;
  }
  return this->value_;
}

}  // namespace pcm5122
}  // namespace esphome
