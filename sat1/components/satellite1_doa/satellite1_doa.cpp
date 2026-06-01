#include "satellite1_doa.h"

#include "esphome/core/log.h"

namespace esphome {
namespace satellite1_doa {

static const char *const TAG = "satellite1_doa";

static uint32_t read_u32_le(const uint8_t *payload) {
  return (uint32_t) payload[0] | ((uint32_t) payload[1] << 8) | ((uint32_t) payload[2] << 16) |
         ((uint32_t) payload[3] << 24);
}

void Satellite1DoA::dump_config() {
  ESP_LOGCONFIG(TAG, "Satellite1 DoA:");
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Sample Delay", this->sample_delay_sensor_);
  LOG_SENSOR("  ", "Vertical Delay", this->vertical_delay_sensor_);
  LOG_SENSOR("  ", "Angle Index", this->angle_index_sensor_);
  LOG_SENSOR("  ", "Confidence", this->confidence_sensor_);
  LOG_SENSOR("  ", "Energy", this->energy_sensor_);
  LOG_SENSOR("  ", "Mic East Energy", this->mic_east_energy_sensor_);
  LOG_SENSOR("  ", "Mic West Energy", this->mic_west_energy_sensor_);
  LOG_SENSOR("  ", "Mic North Energy", this->mic_north_energy_sensor_);
  LOG_SENSOR("  ", "Mic South Energy", this->mic_south_energy_sensor_);
  LOG_SENSOR("  ", "Frame Counter", this->frame_counter_sensor_);
  LOG_BINARY_SENSOR("  ", "Valid", this->valid_binary_sensor_);
}

void Satellite1DoA::update() {
  if (this->parent_->state != satellite1::SAT_XMOS_CONNECTED_STATE) {
    return;
  }

  uint8_t payload[DOA_STATE_PAYLOAD_LEN] = {0};
  if (!this->parent_->transfer(DOA_RESOURCE_ID, DOA_CMD_READ_STATE, payload, DOA_STATE_PAYLOAD_LEN)) {
    this->failed_reads_++;
    if (this->failed_reads_ == 1 || (this->failed_reads_ % 50) == 0) {
      ESP_LOGW(TAG, "DoA state read failed (%" PRIu32 " failures)", this->failed_reads_);
    }
    return;
  }

  this->failed_reads_ = 0;
  this->sample_delay_ = (int16_t) ((uint16_t) payload[0] | ((uint16_t) payload[1] << 8));
  this->confidence_ = payload[2];
  this->valid_ = (payload[3] & DOA_FLAG_VALID) != 0;
  this->four_mic_ = (payload[3] & DOA_FLAG_FOUR_MIC) != 0;
  this->energy_ = read_u32_le(&payload[4]);
  this->frame_counter_ = read_u32_le(&payload[8]);
  this->vertical_delay_ = (int16_t) ((uint16_t) payload[12] | ((uint16_t) payload[13] << 8));
  this->angle_index_ = payload[14] % 24;
  this->mic_energy_[0] = read_u32_le(&payload[16]);
  this->mic_energy_[1] = read_u32_le(&payload[20]);
  this->mic_energy_[2] = read_u32_le(&payload[24]);
  this->mic_energy_[3] = read_u32_le(&payload[28]);

  const uint32_t now = millis();
  if (now - this->last_publish_ms_ >= 1000) {
    this->last_publish_ms_ = now;
    ESP_LOGV(TAG,
             "x=%d y=%d angle=%u confidence=%u valid=%u four_mic=%u energy=%" PRIu32
             " e=%" PRIu32 " w=%" PRIu32 " n=%" PRIu32 " s=%" PRIu32 " frame=%" PRIu32,
             this->sample_delay_, this->vertical_delay_, this->angle_index_, this->confidence_, this->valid_,
             this->four_mic_, this->energy_, this->mic_energy_[0], this->mic_energy_[1], this->mic_energy_[2],
             this->mic_energy_[3], this->frame_counter_);
    if (this->sample_delay_sensor_ != nullptr) {
      this->sample_delay_sensor_->publish_state(this->sample_delay_);
    }
    if (this->vertical_delay_sensor_ != nullptr) {
      this->vertical_delay_sensor_->publish_state(this->vertical_delay_);
    }
    if (this->angle_index_sensor_ != nullptr) {
      this->angle_index_sensor_->publish_state(this->angle_index_);
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
    if (this->mic_east_energy_sensor_ != nullptr) {
      this->mic_east_energy_sensor_->publish_state(this->mic_energy_[0]);
    }
    if (this->mic_west_energy_sensor_ != nullptr) {
      this->mic_west_energy_sensor_->publish_state(this->mic_energy_[1]);
    }
    if (this->mic_north_energy_sensor_ != nullptr) {
      this->mic_north_energy_sensor_->publish_state(this->mic_energy_[2]);
    }
    if (this->mic_south_energy_sensor_ != nullptr) {
      this->mic_south_energy_sensor_->publish_state(this->mic_energy_[3]);
    }
    if (this->valid_binary_sensor_ != nullptr) {
      this->valid_binary_sensor_->publish_state(this->valid_);
    }
  }

  ESP_LOGVV(TAG, "x=%d y=%d angle=%u confidence=%u valid=%u four_mic=%u energy=%" PRIu32 " frame=%" PRIu32,
            this->sample_delay_, this->vertical_delay_, this->angle_index_, this->confidence_, this->valid_,
            this->four_mic_, this->energy_, this->frame_counter_);
}

int Satellite1DoA::led_index() const {
  if (this->four_mic_) {
    return this->angle_index_ % 24;
  }

  int lag = this->sample_delay_;
  if (lag < -4) {
    lag = -4;
  } else if (lag > 4) {
    lag = 4;
  }

  int index = 12 + (lag * 2);
  while (index < 0) {
    index += 24;
  }
  return index % 24;
}

}  // namespace satellite1_doa
}  // namespace esphome
