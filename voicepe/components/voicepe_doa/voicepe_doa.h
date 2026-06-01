#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace voicepe_doa {

static const uint8_t DOA_RESOURCE_ID = 231;
static const uint8_t DOA_CMD_READ_STATE = 0x80;
static const uint8_t DOA_STATE_PAYLOAD_LEN = 12;
static const uint8_t DOA_STATE_RESPONSE_LEN = DOA_STATE_PAYLOAD_LEN + 1;
static const uint8_t DOA_FLAG_VALID = 1u << 0;

class VoicePEDoA : public PollingComponent, public i2c::I2CDevice {
 public:
  void update() override;
  void dump_config() override;

  void set_sample_delay_sensor(sensor::Sensor *sensor) { this->sample_delay_sensor_ = sensor; }
  void set_confidence_sensor(sensor::Sensor *sensor) { this->confidence_sensor_ = sensor; }
  void set_energy_sensor(sensor::Sensor *sensor) { this->energy_sensor_ = sensor; }
  void set_frame_counter_sensor(sensor::Sensor *sensor) { this->frame_counter_sensor_ = sensor; }
  void set_valid_binary_sensor(binary_sensor::BinarySensor *sensor) { this->valid_binary_sensor_ = sensor; }

  int16_t sample_delay() const { return this->sample_delay_; }
  uint8_t confidence() const { return this->confidence_; }
  bool valid() const { return this->valid_; }
  uint32_t energy() const { return this->energy_; }
  uint32_t frame_counter() const { return this->frame_counter_; }

 protected:
  sensor::Sensor *sample_delay_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *frame_counter_sensor_{nullptr};
  binary_sensor::BinarySensor *valid_binary_sensor_{nullptr};

  int16_t sample_delay_{0};
  uint8_t confidence_{0};
  bool valid_{false};
  uint32_t energy_{0};
  uint32_t frame_counter_{0};
  uint32_t failed_reads_{0};
  uint32_t last_publish_ms_{0};
};

}  // namespace voicepe_doa
}  // namespace esphome
