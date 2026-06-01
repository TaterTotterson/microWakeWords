#include "voicepe_doa.h"

#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome {
namespace voicepe_doa {

static const char *const TAG = "voicepe_doa";

static uint32_t read_u32_le(const uint8_t *payload) {
  return (uint32_t) payload[0] | ((uint32_t) payload[1] << 8) | ((uint32_t) payload[2] << 16) |
         ((uint32_t) payload[3] << 24);
}

void VoicePEDoA::dump_config() {
  ESP_LOGCONFIG(TAG, "VoicePE DoA:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Sample Delay", this->sample_delay_sensor_);
  LOG_SENSOR("  ", "Confidence", this->confidence_sensor_);
  LOG_SENSOR("  ", "Energy", this->energy_sensor_);
  LOG_SENSOR("  ", "Frame Counter", this->frame_counter_sensor_);
  LOG_BINARY_SENSOR("  ", "Valid", this->valid_binary_sensor_);
}

void VoicePEDoA::update() {
  const uint8_t request[3] = {DOA_RESOURCE_ID, DOA_CMD_READ_STATE, DOA_STATE_RESPONSE_LEN};
  uint8_t response[DOA_STATE_RESPONSE_LEN] = {0};

  auto error_code = this->write(request, sizeof(request));
  if (error_code != i2c::ERROR_OK) {
    this->failed_reads_++;
    if (this->failed_reads_ == 1 || (this->failed_reads_ % 50) == 0) {
      ESP_LOGW(TAG, "DoA request failed (%" PRIu32 " failures)", this->failed_reads_);
    }
    return;
  }

  error_code = this->read(response, sizeof(response));
  if (error_code != i2c::ERROR_OK || response[0] != 0) {
    this->failed_reads_++;
    if (this->failed_reads_ == 1 || (this->failed_reads_ % 50) == 0) {
      ESP_LOGW(TAG, "DoA state read failed (%" PRIu32 " failures, status=%u)", this->failed_reads_, response[0]);
    }
    return;
  }

  this->failed_reads_ = 0;
  const uint8_t *payload = &response[1];
  this->sample_delay_ = (int16_t) ((uint16_t) payload[0] | ((uint16_t) payload[1] << 8));
  this->confidence_ = payload[2];
  this->valid_ = (payload[3] & DOA_FLAG_VALID) != 0;
  this->energy_ = read_u32_le(&payload[4]);
  this->frame_counter_ = read_u32_le(&payload[8]);

  const uint32_t now = millis();
  if (now - this->last_publish_ms_ >= 1000) {
    this->last_publish_ms_ = now;
    ESP_LOGV(TAG, "delay=%d confidence=%u valid=%u energy=%" PRIu32 " frame=%" PRIu32, this->sample_delay_,
             this->confidence_, this->valid_, this->energy_, this->frame_counter_);
    if (this->sample_delay_sensor_ != nullptr) {
      this->sample_delay_sensor_->publish_state(this->sample_delay_);
    }
    if (this->confidence_sensor_ != nullptr) {
      this->confidence_sensor_->publish_state(this->confidence_);
    }
    if (this->energy_sensor_ != nullptr) {
      this->energy_sensor_->publish_state(this->energy_);
    }
    if (this->frame_counter_sensor_ != nullptr) {
      this->frame_counter_sensor_->publish_state(this->frame_counter_);
    }
    if (this->valid_binary_sensor_ != nullptr) {
      this->valid_binary_sensor_->publish_state(this->valid_);
    }
  }

  ESP_LOGVV(TAG, "delay=%d confidence=%u valid=%u energy=%" PRIu32 " frame=%" PRIu32, this->sample_delay_,
            this->confidence_, this->valid_, this->energy_, this->frame_counter_);
}

}  // namespace voicepe_doa
}  // namespace esphome
