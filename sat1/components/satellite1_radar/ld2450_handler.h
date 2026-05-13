#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/preferences.h"
#include "radar_entities.h"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace esphome {
namespace satellite1_radar {

using TargetUpdateCallback = std::function<void(int target, float x, float y)>;

class LD2450Handler {
 public:
  explicit LD2450Handler(uart::UARTDevice &uart) : uart_(uart) {}
  static constexpr size_t NUM_TARGETS = 3;
  static constexpr size_t NUM_ZONES = 3;
  static constexpr size_t MAX_ZONE_POINTS = 8;
  static constexpr size_t DATA_FRAME_SIZE = 30;

  struct Point {
    int16_t x{0};
    int16_t y{0};
  };

  struct Polygon {
    uint8_t points_count{0};
    Point points[MAX_ZONE_POINTS]{};
  };

  struct LD2450BackendConfig {
    Polygon zones[NUM_ZONES]{};
    Polygon exclusion{};
    uint16_t detection_range_cm{0};
    uint8_t stability{5};
    uint16_t timeout_seconds{0};
    bool bluetooth_enabled{false};
    bool multi_target_enabled{true};
  };

  // Aggregate counts
  sensor::Sensor *target_count{nullptr};
  sensor::Sensor *still_target_count{nullptr};
  sensor::Sensor *moving_target_count{nullptr};

  // Per-zone state text sensors
  text_sensor::TextSensor *zone_state[NUM_ZONES]{};

  // Target update callback (set by parent when zone editor is active)
  TargetUpdateCallback on_target_update{nullptr};

  // Text sensors
  text_sensor::TextSensor *version_text_sensor{nullptr};
  text_sensor::TextSensor *target_state_text_sensor{nullptr};

  // Common entities (published by parent, stored as references)
  binary_sensor::BinarySensor *presence_sensor{nullptr};

  void set_device_class_indices(const DeviceClassMeta &meta) { this->device_class_meta_ = meta; }
  void set_unit_indices(const UnitMeta &meta) { this->unit_meta_ = meta; }
  void set_icon_indices(const IconMeta &meta) { this->icon_meta_ = meta; }
  void create_and_register_entities();

  void setup();
  void loop();

  void factory_reset();
  void restart();

  void set_single_target();
  void set_multi_target();
  void turn_bluetooth_on();
  void turn_bluetooth_off();

  const LD2450BackendConfig &get_backend_config() const { return config_; }
  bool is_reboot_required() const { return reboot_required_; }
  bool set_backend_config(const LD2450BackendConfig &cfg);
  void set_bluetooth_enabled(bool enabled);
  void set_multi_target_enabled(bool enabled);

 protected:
  static constexpr size_t MAX_BUF = 160;
  static constexpr size_t MAX_CMD_LEN = 16;
  static const int FW_MAX_RETRIES = 5;
  static const int DEFAULT_STABILITY = 5;
  std::unique_ptr<uint8_t[]> buf_{};
  size_t buf_pos_{0};
  uart::UARTDevice &uart_;
  bool fw_version_received_{false};
  int fw_retry_count_{0};
  uint32_t fw_next_retry_ms_{0};
  bool fw_query_in_flight_{false};
  uint32_t fw_query_timeout_ms_{0};
  bool bt_restart_pending_{false};
  uint32_t bt_restart_due_ms_{0};
  bool bt_readback_pending_{false};
  uint32_t bt_readback_due_ms_{0};
  static const uint32_t BT_RESTART_DELAY_MS = 200;
  static const uint32_t BT_READBACK_DELAY_MS = 1500;
  static const uint32_t COMMAND_ACK_TIMEOUT_MS = 250;
  static const uint8_t COMMAND_MAX_RETRIES = 2;

  enum class TxState : uint8_t {
    IDLE,
    WAIT_ENABLE_ACK,
    WAIT_COMMAND_ACK,
    WAIT_DISABLE_ACK,
  };

  struct QueuedCommand {
    uint8_t bytes[MAX_CMD_LEN]{};
    size_t len{0};
  };

  static constexpr size_t MAX_COMMAND_QUEUE_DEPTH = 16;
  QueuedCommand command_queue_[MAX_COMMAND_QUEUE_DEPTH]{};
  size_t command_queue_head_{0};
  size_t command_queue_count_{0};
  QueuedCommand active_command_{};
  bool has_active_command_{false};
  bool drop_active_after_disable_{false};
  TxState tx_state_{TxState::IDLE};
  uint32_t tx_deadline_ms_{0};
  uint16_t tx_expected_ack_cmd_{0};
  uint8_t tx_retry_count_{0};
  bool tx_ack_pending_{false};
  uint16_t tx_ack_cmd_{0};
  uint8_t tx_ack_status_{0};
  uint32_t ack_timeout_count_{0};
  uint32_t ack_failure_count_{0};
  uint16_t last_failed_cmd_{0};
  bool config_session_open_{false};

  // Debounce state for aggregate count sensors
  int pub_total_count_{-1}, cand_total_count_{-1}, streak_total_{0};
  int pub_moving_count_{-1}, cand_moving_count_{-1}, streak_moving_{0};
  int pub_still_count_{-1}, cand_still_count_{-1}, streak_still_{0};

  // Debounce state for binary sensors
  int pub_presence_{-1}, cand_presence_{-1}, streak_presence_{0};

  // Debounce state for zone text sensors
  std::string pub_zone_state_[NUM_ZONES];
  std::string cand_zone_state_[NUM_ZONES];
  int streak_zone_[NUM_ZONES]{};
  std::string pub_target_state_;
  std::string cand_target_state_;
  int streak_target_state_{0};

  static uint16_t to_uint16(uint8_t lo, uint8_t hi);
  static int16_t to_signed(uint16_t val);

  void parse_data_frame_(const uint8_t *buf);
  bool point_in_polygon_(const Point *points, size_t count, float x, float y);
  bool is_in_exclusion_zone_(float x, float y);
  bool is_beyond_detection_range_(float dist);
  void update_zone_states_(int threshold);
  void handle_ack_frame_(const uint8_t *buf, size_t len);
  void request_firmware_version_();
  void request_mac_address_();
  void query_target_tracking_mode_();
  void query_zone_();
  void restart_radar_();
  void request_full_status_();
  void schedule_post_bluetooth_sync_();
  void mark_tx_ack_(uint16_t cmd_word, uint8_t status);
  bool consume_tx_ack_(uint16_t expected_cmd, uint8_t &status_out);
  void mark_command_failed_(uint16_t cmd_word, const char *reason);
  void drop_active_command_(const char *reason);
  uint16_t command_word_(const QueuedCommand &queued) const;
  void send_enable_config_();
  void send_active_command_();
  void send_disable_config_();
  void enqueue_command_(const uint8_t *cmd, size_t len);
  bool dequeue_command_(QueuedCommand &queued);
  void step_command_tx_();
  void send_command_(const uint8_t *cmd, size_t len);
  int get_stability_threshold_();

  void debounce_sensor_(int &pub, int &cand, int &streak, int raw, int threshold, sensor::Sensor *s);
  void debounce_binary_(int &pub, int &cand, int &streak, bool raw, int threshold, binary_sensor::BinarySensor *s);
  void debounce_zone_(int z, const std::string &raw, int threshold);
  void debounce_target_state_(const std::string &raw, int threshold);

  static void publish_sensor_(sensor::Sensor *s, float val);

  bool validate_backend_config_(LD2450BackendConfig &cfg) const;
  bool entity_layout_matches_(const LD2450BackendConfig &lhs, const LD2450BackendConfig &rhs) const;
  void save_backend_config_();

  LD2450BackendConfig config_{};
  LD2450BackendConfig boot_config_{};
  bool boot_config_initialized_{false};
  bool reboot_required_{false};
  ESPPreferenceObject config_pref_;
  float target_x_cm_[NUM_TARGETS]{};
  float target_y_cm_[NUM_TARGETS]{};
  float target_speed_cm_s_[NUM_TARGETS]{};
  float target_distance_cm_[NUM_TARGETS]{};
  bool target_valid_[NUM_TARGETS]{};
  DeviceClassMeta device_class_meta_{};
  UnitMeta unit_meta_{};
  IconMeta icon_meta_{};

  std::unique_ptr<Satellite1RadarDynamicBinarySensor> runtime_presence_binary_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicTextSensor> runtime_radar_firmware_text_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicTextSensor> runtime_target_state_text_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicTextSensor> runtime_zone_state_text_sensor_[NUM_ZONES]{};
  std::unique_ptr<Satellite1RadarDynamicSensor> runtime_target_count_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicSensor> runtime_still_target_count_sensor_{};
  std::unique_ptr<Satellite1RadarDynamicSensor> runtime_moving_target_count_sensor_{};
  std::unique_ptr<Satellite1RadarButton> runtime_factory_reset_button_{};
  std::unique_ptr<Satellite1RadarButton> runtime_restart_button_{};
};

}  // namespace satellite1_radar
}  // namespace esphome
