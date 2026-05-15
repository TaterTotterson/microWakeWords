#include "testing.h"

#include "esphome/core/log.h"

namespace esphome {
namespace satellite1 {

static const char *TAG = "Satellite1-Testing";

void SPIErrorRate::setup() { start_test(); }

void SPIErrorRate::loop() {
  if (this->start_time_) {
    this->send_test_frame();
    this->read_test_frame();
    if ((millis() - this->start_time_) > 1000) {
      this->report();
      this->start_time_ = millis();
      this->frames_received_ = 0;
      this->incorrect_bytes_ = 0;
      this->ignored_cmds_ = 0;
    }
  }
}

void SPIErrorRate::start_test() {
  this->start_time_ = millis();
  this->waiting_ = false;
}

void SPIErrorRate::stop_test() { this->start_time_ = 0; }

bool SPIErrorRate::send_test_frame() {
  if (!this->waiting_) {
    for (int i = 0; i < this->bytes_per_frame_; i++) {
      this->last_sent_[i] = rand() % 255;
    }
    if (this->parent_->transfer(ECHO_RES_ID, this->echo_cmd_, this->last_sent_, this->bytes_per_frame_)) {
      uint8_t *send_recv_buf = this->last_sent_;
      // ESP_LOGD(TAG, "SEND: %x %x %x %x %x", send_recv_buf[0], send_recv_buf[1], send_recv_buf[2], send_recv_buf[3],
      // send_recv_buf[4] );
      this->waiting_ = true;
      return true;
    } else {
      this->ignored_cmds_++;
    }
  }
  return false;
}

bool SPIErrorRate::read_test_frame() {
  if (this->waiting_) {
    uint8_t buffer[256];
    memset(buffer, 0, 256);
    if (this->parent_->transfer(ECHO_RES_ID, this->echo_cmd_ | 0x80, buffer, this->bytes_per_frame_)) {
      for (int i = 0; i < this->bytes_per_frame_; i++) {
        if (buffer[i] != this->last_sent_[i]) {
          this->incorrect_bytes_++;
        }
      }
      this->frames_received_++;
      this->waiting_ = false;
      return true;
    } else {
      this->ignored_cmds_++;
    }
  }
  return false;
}

bool SPIErrorRate::handle_response(uint8_t status, uint8_t res_id, uint8_t cmd, uint8_t *payload, uint8_t payload_len) {
  return false;
};

void SPIErrorRate::report() {
  uint32_t elapsed_time = (millis() - this->start_time_);

  ESP_LOGD(
      TAG, "Frames: %d, bytes/sec: %4.2f, error_rate: %4.2f incorrect: %d (failed: %d)", this->frames_received_,
      this->frames_received_ * this->bytes_per_frame_ * 1000. / elapsed_time,
      this->frames_received_ ? 1. * this->incorrect_bytes_ / (this->bytes_per_frame_ * this->frames_received_) : 0.,
      this->incorrect_bytes_, this->ignored_cmds_);
}

}  // namespace satellite1
}  // namespace esphome