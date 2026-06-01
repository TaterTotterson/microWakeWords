#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/satellite1/satellite1.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace satellite1_doa {

static const uint8_t DOA_RESOURCE_ID = 231;
static const uint8_t DOA_CMD_READ_STATE = 0 | satellite1::CONTROL_CMD_READ_BIT;
static const uint8_t DOA_STATE_PAYLOAD_LEN = 32;
static const uint8_t DOA_FLAG_VALID = 1u << 0;
static const uint8_t DOA_FLAG_FOUR_MIC = 1u << 1;

class Satellite1DoA : public PollingComponent, public satellite1::Satellite1SPIService {
 public:
  void update() override;
  void dump_config() override;

  void set_sample_delay_sensor(sensor::Sensor *sensor) { this->sample_delay_sensor_ = sensor; }
  void set_confidence_sensor(sensor::Sensor *sensor) { this->confidence_sensor_ = sensor; }
  void set_energy_sensor(sensor::Sensor *sensor) { this->energy_sensor_ = sensor; }
  void set_angle_index_sensor(sensor::Sensor *sensor) { this->angle_index_sensor_ = sensor; }
  void set_vertical_delay_sensor(sensor::Sensor *sensor) { this->vertical_delay_sensor_ = sensor; }
  void set_mic_east_energy_sensor(sensor::Sensor *sensor) { this->mic_east_energy_sensor_ = sensor; }
  void set_mic_west_energy_sensor(sensor::Sensor *sensor) { this->mic_west_energy_sensor_ = sensor; }
  void set_mic_north_energy_sensor(sensor::Sensor *sensor) { this->mic_north_energy_sensor_ = sensor; }
  void set_mic_south_energy_sensor(sensor::Sensor *sensor) { this->mic_south_energy_sensor_ = sensor; }
  void set_frame_counter_sensor(sensor::Sensor *sensor) { this->frame_counter_sensor_ = sensor; }
  void set_valid_binary_sensor(binary_sensor::BinarySensor *sensor) { this->valid_binary_sensor_ = sensor; }

  int16_t sample_delay() const { return this->sample_delay_; }
  int16_t vertical_delay() const { return this->vertical_delay_; }
  uint8_t angle_index() const { return this->angle_index_; }
  uint8_t confidence() const { return this->confidence_; }
  bool valid() const { return this->valid_; }
  bool four_mic() const { return this->four_mic_; }
  uint32_t energy() const { return this->energy_; }
  uint32_t frame_counter() const { return this->frame_counter_; }

  int led_index() const;

 protected:
  sensor::Sensor *sample_delay_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *angle_index_sensor_{nullptr};
  sensor::Sensor *vertical_delay_sensor_{nullptr};
  sensor::Sensor *mic_east_energy_sensor_{nullptr};
  sensor::Sensor *mic_west_energy_sensor_{nullptr};
  sensor::Sensor *mic_north_energy_sensor_{nullptr};
  sensor::Sensor *mic_south_energy_sensor_{nullptr};
  sensor::Sensor *frame_counter_sensor_{nullptr};
  binary_sensor::BinarySensor *valid_binary_sensor_{nullptr};

  int16_t sample_delay_{0};
  int16_t vertical_delay_{0};
  uint8_t angle_index_{12};
  uint8_t confidence_{0};
  bool valid_{false};
  bool four_mic_{false};
  uint32_t energy_{0};
  uint32_t mic_energy_[4]{0, 0, 0, 0};
  uint32_t frame_counter_{0};
  uint32_t failed_reads_{0};
  uint32_t last_publish_ms_{0};
};

}  // namespace satellite1_doa
}  // namespace esphome
