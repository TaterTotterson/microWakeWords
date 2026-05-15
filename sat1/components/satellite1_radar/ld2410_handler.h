#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/preferences.h"
#include "radar_entities.h"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace esphome {
namespace satellite1_radar {

class LD2410Handler {
 public:
  explicit LD2410Handler(uart::UARTDevice &uart) : uart_(uart) {}
  static constexpr size_t NUM_GATES = 9;  // g0 through g8

  struct LD2410BackendConfig {
    uint16_t timeout_seconds{0};
    uint8_t max_move_gate{8};
    uint8_t max_still_gate{8};
    uint8_t gate_move_threshold[NUM_GATES]{50, 50, 50, 50, 50, 50, 50, 50, 50};
    uint8_t gate_still_threshold[NUM_GATES]{50, 50, 50, 50, 50, 50, 50, 50, 50};
    uint8_t distance_resolution{0};  // 0=0.75m, 1=0.2m
    bool bluetooth_enabled{false};
  };

  // Sensors
  sensor::Sensor *moving_distance{nullptr};
  sensor::Sensor *still_distance{nullptr};
  sensor::Sensor *moving_energy{nullptr};
  sensor::Sensor *still_energy{nullptr};
  sensor::Sensor *detection_distance{nullptr};
  sensor::Sensor *light_sensor{nullptr};

  // Text sensors
  text_sensor::TextSensor *version_text_sensor{nullptr};
  text_sensor::TextSensor *target_state_text_sensor{nullptr};

  // Common entities (references from parent)
  binary_sensor::BinarySensor *presence_sensor{nullptr};

  void set_device_class_indices(const DeviceClassMeta &meta) { this->device_class_meta_ = meta; }
  void set_unit_indices(const UnitMeta &meta) { this->unit_meta_ = meta; }
  void set_icon_indices(const IconMeta &meta) { this->icon_meta_ = meta; }
  void create_and_register_entities();

  void setup();
  void loop();

  void factory_reset();
  void restart();
  void query_params();
  void query_mac_address();
  void write_gate_config();
  void set_distance_resolution(bool fine);
  void set_bluetooth_enabled(bool enabled);
  void enable_engineering_mode();
  void disable_engineering_mode();

  const LD2410BackendConfig &get_backend_config() const { return config_; }
  bool set_backend_config(const LD2410BackendConfig &cfg);
  void apply_backend_config();
  float get_gate_move_energy(size_t gate) const { return (gate < NUM_GATES) ? gate_move_energy_values_[gate] : 0.0f; }
  float get_gate_still_energy(size_t gate) const { return (gate < NUM_GATES) ? gate_still_energy_values_[gate] : 0.0f; }

 protected:
  static constexpr size_t MAX_BUF = 256;
  std::unique_ptr<uint8_t[]> buf_{};
  size_t buf_pos_{0};
  uart::UARTDevice &uart_;

  static constexpr size_t MAX_CMD_FRAME = 64;
  static constexpr size_t MAX_CMD_QUEUE_DEPTH = 32;
  struct QueuedCmd {
    uint8_t frame[MAX_CMD_FRAME];
    size_t len{0};
  };
  QueuedCmd cmd_queue_[MAX_CMD_QUEUE_DEPTH]{};
  size_t cmd_queue_head_{0};
  size_t cmd_queue_count_{0};
  bool waiting_for_ack_{false};
  uint32_t ack_wait_start_{0};
  bool bt_readback_pending_{false};
  uint32_t bt_readback_due_ms_{0};
  static const uint32_t BT_READBACK_DELAY_MS = 1500;
  static const uint32_t ACK_TIMEOUT_MS = 1000;
  static const int TARGET_STATE_STREAK_THRESHOLD = 3;
  std::string pub_target_state_;
  std::string cand_target_state_;
  int streak_target_state_{0};

  static uint16_t to_uint16_(uint8_t lo, uint8_t hi);

  void process_queue_();
  void on_ack_received_();
  void queue_enter_config_();
  void queue_exit_config_();
  void queue_config_command_(uint16_t command, const uint8_t *data, size_t data_len);
  void enqueue_frame_(const QueuedCmd &cmd);
  bool dequeue_frame_(QueuedCmd &cmd);
  void parse_data_frame_(const uint8_t *buf, size_t len);
  void handle_ack_frame_(const uint8_t *buf, size_t len);
  void debounce_target_state_(const std::string &raw, int threshold);

  static void publish_sensor_(sensor::Sensor *s, float val);
  static void put_uint16_le_(uint8_t *buf, uint16_t val);
  static void put_uint32_le_(uint8_t *buf, uint32_t val);

  bool validate_backend_config_(LD2410BackendConfig &cfg) const;
  void save_backend_config_();

  LD2410BackendConfig config_{};
  ESPPreferenceObject config_pref_;
  float gate_move_energy_values_[NUM_GATES]{};
  float gate_still_energy_values_[NUM_GATES]{};
  DeviceClassMeta device_class_meta_{};
  UnitMeta unit_meta_{};
  IconMeta icon_meta_{};

  std::unique_ptr<Satellite1RadarDynamicBinarySensor> runtime_presence_binary_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicTextSensor> runtime_radar_firmware_text_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicTextSensor> runtime_target_state_text_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicSensor> runtime_moving_distance_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicSensor> runtime_still_distance_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicSensor> runtime_moving_energy_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicSensor> runtime_still_energy_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicSensor> runtime_detection_distance_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicSensor> runtime_light_sensor_{};
  std::unique_ptr<Satellite1RadarButton> runtime_factory_reset_button_{};
  std::unique_ptr<Satellite1RadarButton> runtime_restart_button_{};
  std::unique_ptr<Satellite1RadarButton> runtime_query_params_button_{};
};

}  // namespace satellite1_radar
}  // namespace esphome
