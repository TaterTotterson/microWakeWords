#include "ld2450_handler.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <cmath>
#include <cstring>

namespace esphome {
namespace satellite1_radar {

static const char *const TAG_LD2450 = "satellite1_radar.ld2450";
static constexpr size_t MAX_UART_BYTES_PER_LOOP = 128;
static const uint8_t LD2450_ENABLE_CONFIG[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF,
                                               0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
static const uint8_t LD2450_FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t LD2450_DISABLE_CONFIG[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};

static inline bool deadline_passed_(uint32_t deadline, uint32_t now) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

void LD2450Handler::setup() {
  ESP_LOGI(TAG_LD2450, "Initializing LD2450 handler");
  if (!buf_) {
    buf_ = std::unique_ptr<uint8_t[]>(new uint8_t[MAX_BUF]());
  }
  fw_version_received_ = false;
  fw_retry_count_ = 0;
  fw_next_retry_ms_ = millis() + 2000;
  config_pref_ = global_preferences->make_preference<LD2450BackendConfig>(fnv1_hash("sat1.ld2450.config"));
  LD2450BackendConfig loaded;
  if (config_pref_.load(&loaded)) {
    this->set_backend_config(loaded);
  }
  boot_config_ = config_;
  boot_config_initialized_ = true;
  reboot_required_ = false;
}

void LD2450Handler::create_and_register_entities() {
  if (this->presence_sensor == nullptr) {
    this->runtime_presence_binary_sensor_.reset(new Satellite1RadarDynamicBinarySensor());
    this->runtime_presence_binary_sensor_->configure_dynamic("Room Presence", ENTITY_CATEGORY_NONE, false,
                                                             this->device_class_meta_.occupancy);
    App.register_binary_sensor(this->runtime_presence_binary_sensor_.get());
    this->presence_sensor = this->runtime_presence_binary_sensor_.get();
  }

  if (this->version_text_sensor == nullptr) {
    this->runtime_radar_firmware_text_sensor_.reset(new Satellite1RadarDynamicTextSensor());
    this->runtime_radar_firmware_text_sensor_->configure_dynamic("Radar Firmware", ENTITY_CATEGORY_DIAGNOSTIC, false,
                                                                 this->icon_meta_.chip);
    App.register_text_sensor(this->runtime_radar_firmware_text_sensor_.get());
    this->version_text_sensor = this->runtime_radar_firmware_text_sensor_.get();
  }

  if (this->target_state_text_sensor == nullptr) {
    this->runtime_target_state_text_sensor_.reset(new Satellite1RadarDynamicTextSensor());
    this->runtime_target_state_text_sensor_->configure_dynamic("Radar Target", ENTITY_CATEGORY_NONE, false,
                                                               this->icon_meta_.motion_sensor);
    App.register_text_sensor(this->runtime_target_state_text_sensor_.get());
    this->target_state_text_sensor = this->runtime_target_state_text_sensor_.get();
  }

  if (config_.multi_target_enabled) {
    if (this->target_count == nullptr) {
      this->runtime_target_count_sensor_.reset(new Satellite1RadarDynamicSensor());
      this->runtime_target_count_sensor_->configure_dynamic("Radar Targets Total", ENTITY_CATEGORY_NONE, false, 0, 0,
                                                            this->icon_meta_.account_multiple, true,
                                                            sensor::STATE_CLASS_MEASUREMENT, 0);
      App.register_sensor(this->runtime_target_count_sensor_.get());
      this->target_count = this->runtime_target_count_sensor_.get();
    }

    if (this->still_target_count == nullptr) {
      this->runtime_still_target_count_sensor_.reset(new Satellite1RadarDynamicSensor());
      this->runtime_still_target_count_sensor_->configure_dynamic("Radar Targets Still", ENTITY_CATEGORY_NONE, false, 0,
                                                                  0, this->icon_meta_.account, true,
                                                                  sensor::STATE_CLASS_MEASUREMENT, 0);
      App.register_sensor(this->runtime_still_target_count_sensor_.get());
      this->still_target_count = this->runtime_still_target_count_sensor_.get();
    }

    if (this->moving_target_count == nullptr) {
      this->runtime_moving_target_count_sensor_.reset(new Satellite1RadarDynamicSensor());
      this->runtime_moving_target_count_sensor_->configure_dynamic("Radar Targets Moving", ENTITY_CATEGORY_NONE, false,
                                                                   0, 0, this->icon_meta_.account_arrow_right, true,
                                                                   sensor::STATE_CLASS_MEASUREMENT, 0);
      App.register_sensor(this->runtime_moving_target_count_sensor_.get());
      this->moving_target_count = this->runtime_moving_target_count_sensor_.get();
    }
  }

  for (size_t i = 0; i < NUM_ZONES; i++) {
    if (this->config_.zones[i].points_count < 3)
      continue;
    if (this->zone_state[i] != nullptr)
      continue;
    this->runtime_zone_state_text_sensor_[i].reset(new Satellite1RadarDynamicTextSensor());
    switch (i) {
      case 0:
        this->runtime_zone_state_text_sensor_[i]->configure_dynamic("Radar Zone 1", ENTITY_CATEGORY_NONE, false,
                                                                    this->icon_meta_.motion_sensor);
        break;
      case 1:
        this->runtime_zone_state_text_sensor_[i]->configure_dynamic("Radar Zone 2", ENTITY_CATEGORY_NONE, false,
                                                                    this->icon_meta_.motion_sensor);
        break;
      default:
        this->runtime_zone_state_text_sensor_[i]->configure_dynamic("Radar Zone 3", ENTITY_CATEGORY_NONE, false,
                                                                    this->icon_meta_.motion_sensor);
        break;
    }
    App.register_text_sensor(this->runtime_zone_state_text_sensor_[i].get());
    this->zone_state[i] = this->runtime_zone_state_text_sensor_[i].get();
  }

  if (this->runtime_factory_reset_button_ == nullptr) {
    this->runtime_factory_reset_button_.reset(new Satellite1RadarButton());
    this->runtime_factory_reset_button_->configure_dynamic("Radar Factory Reset", ENTITY_CATEGORY_DIAGNOSTIC, false,
                                                           this->icon_meta_.factory);
    this->runtime_factory_reset_button_->add_on_press_callback([this]() { this->factory_reset(); });
    App.register_button(this->runtime_factory_reset_button_.get());
  }

  if (this->runtime_restart_button_ == nullptr) {
    this->runtime_restart_button_.reset(new Satellite1RadarButton());
    this->runtime_restart_button_->configure_dynamic("Radar Restart", ENTITY_CATEGORY_DIAGNOSTIC, false,
                                                     this->icon_meta_.restart);
    this->runtime_restart_button_->add_on_press_callback([this]() { this->restart(); });
    App.register_button(this->runtime_restart_button_.get());
  }
}

void LD2450Handler::loop() {
  if (!buf_)
    return;

  uint32_t now = millis();

  if (!fw_version_received_ && !fw_query_in_flight_ && fw_retry_count_ < FW_MAX_RETRIES &&
      deadline_passed_(fw_next_retry_ms_, now)) {
    request_firmware_version_();
  }

  if (fw_query_in_flight_ && deadline_passed_(fw_query_timeout_ms_, now)) {
    fw_query_in_flight_ = false;
    fw_next_retry_ms_ = now + 3000;
    ESP_LOGW(TAG_LD2450, "Firmware query timeout (attempt %d/%d)", fw_retry_count_, FW_MAX_RETRIES);
  }

  if (bt_restart_pending_ && deadline_passed_(bt_restart_due_ms_, now)) {
    bt_restart_pending_ = false;
    restart_radar_();
    bt_readback_pending_ = true;
    bt_readback_due_ms_ = now + BT_READBACK_DELAY_MS;
  }

  if (bt_readback_pending_ && deadline_passed_(bt_readback_due_ms_, now)) {
    bt_readback_pending_ = false;
    request_full_status_();
  }

  size_t bytes_processed = 0;
  while (uart_.available() && bytes_processed < MAX_UART_BYTES_PER_LOOP) {
    uint8_t byte;
    if (!uart_.read_byte(&byte))
      break;
    bytes_processed++;

    if (buf_pos_ < MAX_BUF)
      buf_[buf_pos_++] = byte;
    else
      buf_pos_ = 0;

    if (buf_pos_ >= 10 && buf_[buf_pos_ - 4] == 0x04 && buf_[buf_pos_ - 3] == 0x03 && buf_[buf_pos_ - 2] == 0x02 &&
        buf_[buf_pos_ - 1] == 0x01) {
      handle_ack_frame_(buf_.get(), buf_pos_);
      buf_pos_ = 0;
      continue;
    }

    if (buf_pos_ >= DATA_FRAME_SIZE && buf_[0] == 0xAA && buf_[1] == 0xFF && buf_[2] == 0x03 && buf_[3] == 0x00 &&
        buf_[28] == 0x55 && buf_[29] == 0xCC) {
      parse_data_frame_(buf_.get());
      buf_pos_ = 0;
    }
  }

  step_command_tx_();
}

void LD2450Handler::factory_reset() {
  static const uint8_t cmd[] = {0x02, 0x00, 0xA2, 0x00, 0x04, 0x03, 0x02, 0x01};
  send_command_(cmd, sizeof(cmd));
  ESP_LOGI(TAG_LD2450, "Factory reset command sent");
}

void LD2450Handler::restart() {
  static const uint8_t cmd[] = {0x02, 0x00, 0xA3, 0x00, 0x04, 0x03, 0x02, 0x01};
  send_command_(cmd, sizeof(cmd));
  ESP_LOGI(TAG_LD2450, "Restart command sent");
}

void LD2450Handler::set_single_target() {
  static const uint8_t cmd[] = {0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
  send_command_(cmd, sizeof(cmd));
}

void LD2450Handler::set_multi_target() {
  static const uint8_t cmd[] = {0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};
  send_command_(cmd, sizeof(cmd));
}

void LD2450Handler::turn_bluetooth_on() {
  static const uint8_t cmd[] = {0x04, 0x00, 0xA4, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
  enqueue_command_(cmd, sizeof(cmd));
}

void LD2450Handler::turn_bluetooth_off() {
  static const uint8_t cmd[] = {0x04, 0x00, 0xA4, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
  enqueue_command_(cmd, sizeof(cmd));
}

bool LD2450Handler::validate_backend_config_(LD2450BackendConfig &cfg) const {
  if (cfg.detection_range_cm > 600)
    return false;
  if (cfg.stability > 10)
    return false;

  for (size_t z = 0; z < NUM_ZONES; z++) {
    if (cfg.zones[z].points_count > MAX_ZONE_POINTS)
      return false;
    for (size_t p = 0; p < cfg.zones[z].points_count; p++) {
      if (cfg.zones[z].points[p].x < -6000 || cfg.zones[z].points[p].x > 6000)
        return false;
      if (cfg.zones[z].points[p].y < -6000 || cfg.zones[z].points[p].y > 6000)
        return false;
    }
  }

  if (cfg.exclusion.points_count > MAX_ZONE_POINTS)
    return false;
  for (size_t p = 0; p < cfg.exclusion.points_count; p++) {
    if (cfg.exclusion.points[p].x < -6000 || cfg.exclusion.points[p].x > 6000)
      return false;
    if (cfg.exclusion.points[p].y < -6000 || cfg.exclusion.points[p].y > 6000)
      return false;
  }
  return true;
}

void LD2450Handler::save_backend_config_() {
  config_pref_.save(&config_);
  global_preferences->sync();
}

bool LD2450Handler::set_backend_config(const LD2450BackendConfig &cfg) {
  LD2450BackendConfig candidate = cfg;
  if (!validate_backend_config_(candidate))
    return false;

  bool bluetooth_changed = (candidate.bluetooth_enabled != config_.bluetooth_enabled);
  bool multi_target_changed = (candidate.multi_target_enabled != config_.multi_target_enabled);

  config_ = candidate;
  save_backend_config_();
  if (boot_config_initialized_)
    reboot_required_ = !entity_layout_matches_(config_, boot_config_);

  if (bluetooth_changed) {
    if (config_.bluetooth_enabled)
      turn_bluetooth_on();
    else
      turn_bluetooth_off();
  }
  if (multi_target_changed) {
    if (config_.multi_target_enabled)
      set_multi_target();
    else
      set_single_target();
  }

  return true;
}

bool LD2450Handler::entity_layout_matches_(const LD2450BackendConfig &lhs, const LD2450BackendConfig &rhs) const {
  if (lhs.multi_target_enabled != rhs.multi_target_enabled)
    return false;

  for (size_t i = 0; i < NUM_ZONES; i++) {
    const bool lhs_defined = lhs.zones[i].points_count >= 3;
    const bool rhs_defined = rhs.zones[i].points_count >= 3;
    if (lhs_defined != rhs_defined)
      return false;
  }

  return true;
}

void LD2450Handler::set_bluetooth_enabled(bool enabled) {
  if (config_.bluetooth_enabled == enabled)
    return;
  config_.bluetooth_enabled = enabled;
  save_backend_config_();
  if (enabled)
    turn_bluetooth_on();
  else
    turn_bluetooth_off();
}

void LD2450Handler::set_multi_target_enabled(bool enabled) {
  if (config_.multi_target_enabled == enabled)
    return;
  config_.multi_target_enabled = enabled;
  save_backend_config_();
  if (enabled)
    set_multi_target();
  else
    set_single_target();
}

uint16_t LD2450Handler::to_uint16(uint8_t lo, uint8_t hi) { return (static_cast<uint16_t>(hi) << 8) | lo; }

int16_t LD2450Handler::to_signed(uint16_t val) {
  return (val & 0x8000) ? static_cast<int16_t>(val & 0x7FFF) : -static_cast<int16_t>(val & 0x7FFF);
}

void LD2450Handler::parse_data_frame_(const uint8_t *buf) {
  int total_count = 0;
  int still_count = 0;
  int moving_count = 0;
  bool any_target = false;
  bool any_still = false;
  bool any_approaching = false;
  bool any_moving_away = false;

  for (size_t t = 0; t < NUM_TARGETS; t++) {
    size_t base = 4 + t * 8;
    uint16_t raw_x = to_uint16(buf[base], buf[base + 1]);
    uint16_t raw_y = to_uint16(buf[base + 2], buf[base + 3]);
    uint16_t raw_speed = to_uint16(buf[base + 4], buf[base + 5]);
    float x_cm = -static_cast<float>(to_signed(raw_x)) / 10.0f;
    float y_cm = static_cast<float>(to_signed(raw_y)) / 10.0f;
    float speed = -static_cast<float>(to_signed(raw_speed));
    float dist = std::sqrt(x_cm * x_cm + y_cm * y_cm);
    bool valid = (dist > 0.0f);
    target_valid_[t] = valid;
    target_x_cm_[t] = valid ? x_cm : 0.0f;
    target_y_cm_[t] = valid ? y_cm : 0.0f;
    target_speed_cm_s_[t] = valid ? speed : 0.0f;
    target_distance_cm_[t] = valid ? dist : 0.0f;

    float pub_x = valid ? x_cm : 0.0f;
    float pub_y = valid ? y_cm : 0.0f;

    if (valid && !is_beyond_detection_range_(dist) && !is_in_exclusion_zone_(x_cm, y_cm)) {
      total_count++;
      any_target = true;

      if (std::fabs(speed) > 0.0f) {
        moving_count++;
        if (speed > 0.0f) {
          any_approaching = true;
        } else {
          any_moving_away = true;
        }
      } else {
        still_count++;
        any_still = true;
      }
    }

    if (on_target_update) {
      on_target_update(static_cast<int>(t), pub_x, pub_y);
    }
  }

  int threshold = get_stability_threshold_();

  debounce_sensor_(pub_total_count_, cand_total_count_, streak_total_, total_count, threshold, target_count);
  debounce_sensor_(pub_still_count_, cand_still_count_, streak_still_, still_count, threshold, still_target_count);
  debounce_sensor_(pub_moving_count_, cand_moving_count_, streak_moving_, moving_count, threshold, moving_target_count);

  debounce_binary_(pub_presence_, cand_presence_, streak_presence_, any_target, threshold, presence_sensor);

  const char *target_state = any_approaching   ? "Approaching"
                             : any_moving_away ? "Moving Away"
                             : any_still       ? "Still"
                                               : "Clear";
  debounce_target_state_(target_state, threshold);

  update_zone_states_(threshold);
}

bool LD2450Handler::point_in_polygon_(const Point *points, size_t count, float x, float y) {
  if (points == nullptr)
    return false;
  if (count < 3 || count > MAX_ZONE_POINTS)
    return false;

  float xs[MAX_ZONE_POINTS], ys[MAX_ZONE_POINTS];
  for (size_t i = 0; i < count; i++) {
    xs[i] = static_cast<float>(points[i].x);
    ys[i] = static_cast<float>(points[i].y);
  }

  bool inside = false;
  for (size_t i = 0, j = count - 1; i < count; j = i++) {
    if (((ys[i] > y) != (ys[j] > y)) && (x < (xs[j] - xs[i]) * (y - ys[i]) / (ys[j] - ys[i]) + xs[i]))
      inside = !inside;
  }
  return inside;
}

bool LD2450Handler::is_in_exclusion_zone_(float x, float y) {
  return point_in_polygon_(config_.exclusion.points, config_.exclusion.points_count, x, y);
}

bool LD2450Handler::is_beyond_detection_range_(float dist) {
  if (config_.detection_range_cm == 0)
    return false;
  return dist > static_cast<float>(config_.detection_range_cm);
}

void LD2450Handler::update_zone_states_(int threshold) {
  for (size_t z = 0; z < NUM_ZONES; z++) {
    if (zone_state[z] == nullptr)
      continue;

    const Polygon &zone = config_.zones[z];
    if (zone.points_count < 3) {
      debounce_zone_(static_cast<int>(z), "Undefined", threshold);
      continue;
    }

    int best = -1;

    for (size_t t = 0; t < NUM_TARGETS; t++) {
      if (!target_valid_[t])
        continue;

      float tx = target_x_cm_[t];
      float ty = target_y_cm_[t];
      float ts = target_speed_cm_s_[t];
      float td = target_distance_cm_[t];

      if (td <= 0.0f)
        continue;

      if (is_beyond_detection_range_(td))
        continue;

      if (is_in_exclusion_zone_(tx, ty))
        continue;

      if (point_in_polygon_(zone.points, zone.points_count, tx, ty)) {
        if (ts > 0.0f) {
          best = 2;
          break;
        } else if (ts < 0.0f) {
          if (best < 1)
            best = 1;
        } else {
          if (best < 0)
            best = 0;
        }
      }
    }

    const char *state_str = (best == 2) ? "Approaching" : (best == 1) ? "Moving Away" : (best == 0) ? "Still" : "Clear";
    debounce_zone_(static_cast<int>(z), state_str, threshold);
  }
}

void LD2450Handler::handle_ack_frame_(const uint8_t *buf, size_t len) {
  if (len < 10)
    return;
  if (buf[0] != 0xFD || buf[1] != 0xFC || buf[2] != 0xFB || buf[3] != 0xFA)
    return;

  uint16_t cmd_word = to_uint16(buf[6], buf[7]);
  uint8_t status = buf[8];
  mark_tx_ack_(cmd_word, status);

  if (status != 0) {
    ESP_LOGW(TAG_LD2450, "Command 0x%04X failed (status=%u)", cmd_word, status);
  }

  if (cmd_word == 0x01A0 && status == 0 && len >= 20) {
    char ver[32];
    snprintf(ver, sizeof(ver), "V%u.%02X.%02X%02X%02X%02X", buf[13], buf[12], buf[17], buf[16], buf[15], buf[14]);
    if (version_text_sensor != nullptr)
      version_text_sensor->publish_state(ver);
    fw_version_received_ = true;
    fw_query_in_flight_ = false;
    ESP_LOGI(TAG_LD2450, "Firmware version: %s", ver);
  }

  if (cmd_word == 0x01A5 && status == 0 && len >= 16) {
    char mac[20];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X", buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
    ESP_LOGI(TAG_LD2450, "BT MAC: %s", mac);
  }

  if (cmd_word == 0x01A4 && status == 0) {
    ESP_LOGI(TAG_LD2450, "Bluetooth %s command acknowledged", config_.bluetooth_enabled ? "enable" : "disable");
    schedule_post_bluetooth_sync_();
  }
}

void LD2450Handler::request_firmware_version_() {
  if (fw_query_in_flight_ || fw_version_received_ || fw_retry_count_ >= FW_MAX_RETRIES)
    return;

  static const uint8_t cmd[] = {0x02, 0x00, 0xA0, 0x00, 0x04, 0x03, 0x02, 0x01};
  uint32_t now = millis();
  fw_retry_count_++;
  fw_query_in_flight_ = true;
  fw_query_timeout_ms_ = now + 5000;
  fw_next_retry_ms_ = now + 3000;
  ESP_LOGI(TAG_LD2450, "Requesting firmware version (attempt %d/%d)", fw_retry_count_, FW_MAX_RETRIES);
  enqueue_command_(cmd, sizeof(cmd));
}

void LD2450Handler::request_mac_address_() {
  static const uint8_t cmd[] = {0x04, 0x00, 0xA5, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
  enqueue_command_(cmd, sizeof(cmd));
}

void LD2450Handler::query_target_tracking_mode_() {
  static const uint8_t cmd[] = {0x02, 0x00, 0x91, 0x00, 0x04, 0x03, 0x02, 0x01};
  enqueue_command_(cmd, sizeof(cmd));
}

void LD2450Handler::query_zone_() {
  static const uint8_t cmd[] = {0x02, 0x00, 0xC1, 0x00, 0x04, 0x03, 0x02, 0x01};
  enqueue_command_(cmd, sizeof(cmd));
}

void LD2450Handler::restart_radar_() {
  static const uint8_t cmd[] = {0x02, 0x00, 0xA3, 0x00, 0x04, 0x03, 0x02, 0x01};
  ESP_LOGI(TAG_LD2450, "Restarting radar after Bluetooth change");
  enqueue_command_(cmd, sizeof(cmd));
}

void LD2450Handler::request_full_status_() {
  ESP_LOGI(TAG_LD2450, "Reading back status after Bluetooth change");
  request_firmware_version_();
  request_mac_address_();
  query_target_tracking_mode_();
  query_zone_();
}

void LD2450Handler::schedule_post_bluetooth_sync_() {
  const uint32_t now = millis();
  bt_restart_pending_ = true;
  bt_restart_due_ms_ = now + BT_RESTART_DELAY_MS;
  bt_readback_pending_ = false;
  ESP_LOGI(TAG_LD2450, "Scheduled restart/readback for Bluetooth %s", config_.bluetooth_enabled ? "enable" : "disable");
}

void LD2450Handler::mark_tx_ack_(uint16_t cmd_word, uint8_t status) {
  tx_ack_cmd_ = cmd_word;
  tx_ack_status_ = status;
  tx_ack_pending_ = true;
}

bool LD2450Handler::consume_tx_ack_(uint16_t expected_cmd, uint8_t &status_out) {
  if (!tx_ack_pending_)
    return false;

  const uint16_t ack_cmd = tx_ack_cmd_;
  const uint8_t ack_status = tx_ack_status_;
  tx_ack_pending_ = false;

  if (ack_cmd != expected_cmd) {
    ESP_LOGD(TAG_LD2450, "Ignoring ACK for 0x%04X while waiting for 0x%04X", ack_cmd, expected_cmd);
    return false;
  }

  status_out = ack_status;
  return true;
}

void LD2450Handler::mark_command_failed_(uint16_t cmd_word, const char *reason) {
  last_failed_cmd_ = cmd_word;
  ack_failure_count_++;
  ESP_LOGW(TAG_LD2450, "Command 0x%04X failed (%s)", cmd_word, reason);
}

uint16_t LD2450Handler::command_word_(const QueuedCommand &queued) const {
  if (queued.len < 4)
    return 0;
  return to_uint16(queued.bytes[2], queued.bytes[3]);
}

void LD2450Handler::drop_active_command_(const char *reason) {
  const uint16_t cmd_word = command_word_(active_command_);
  if (cmd_word != 0)
    mark_command_failed_(cmd_word, reason);
  has_active_command_ = false;
  drop_active_after_disable_ = false;
  tx_retry_count_ = 0;
  tx_expected_ack_cmd_ = 0;
  tx_state_ = TxState::IDLE;
}

void LD2450Handler::send_enable_config_() {
  uart_.write_array(LD2450_ENABLE_CONFIG, sizeof(LD2450_ENABLE_CONFIG));
  uart_.flush();
  tx_expected_ack_cmd_ = 0x01FF;
  tx_deadline_ms_ = millis() + COMMAND_ACK_TIMEOUT_MS;
  tx_state_ = TxState::WAIT_ENABLE_ACK;
}

void LD2450Handler::send_active_command_() {
  uart_.write_array(LD2450_FRAME_HEADER, sizeof(LD2450_FRAME_HEADER));
  uart_.write_array(active_command_.bytes, active_command_.len);
  uart_.flush();
  tx_expected_ack_cmd_ = command_word_(active_command_) | 0x0100;
  tx_deadline_ms_ = millis() + COMMAND_ACK_TIMEOUT_MS;
  tx_state_ = TxState::WAIT_COMMAND_ACK;
}

void LD2450Handler::send_disable_config_() {
  uart_.write_array(LD2450_DISABLE_CONFIG, sizeof(LD2450_DISABLE_CONFIG));
  uart_.flush();
  tx_expected_ack_cmd_ = 0x01FE;
  tx_deadline_ms_ = millis() + COMMAND_ACK_TIMEOUT_MS;
  tx_state_ = TxState::WAIT_DISABLE_ACK;
}

void LD2450Handler::enqueue_command_(const uint8_t *cmd, size_t len) {
  if (cmd == nullptr || len == 0)
    return;
  if (len > MAX_CMD_LEN) {
    ESP_LOGW(TAG_LD2450, "Command too long (%u > %u), dropping", static_cast<unsigned int>(len),
             static_cast<unsigned int>(MAX_CMD_LEN));
    return;
  }

  QueuedCommand queued{};
  memcpy(queued.bytes, cmd, len);
  queued.len = len;

  if (command_queue_count_ == MAX_COMMAND_QUEUE_DEPTH) {
    command_queue_head_ = (command_queue_head_ + 1) % MAX_COMMAND_QUEUE_DEPTH;
    command_queue_count_--;
    ESP_LOGW(TAG_LD2450, "Command queue full, dropping oldest");
  }

  const size_t tail = (command_queue_head_ + command_queue_count_) % MAX_COMMAND_QUEUE_DEPTH;
  command_queue_[tail] = queued;
  command_queue_count_++;
}

bool LD2450Handler::dequeue_command_(QueuedCommand &queued) {
  if (command_queue_count_ == 0)
    return false;

  queued = command_queue_[command_queue_head_];
  command_queue_head_ = (command_queue_head_ + 1) % MAX_COMMAND_QUEUE_DEPTH;
  command_queue_count_--;
  return true;
}

void LD2450Handler::step_command_tx_() {
  const uint32_t now = millis();
  uint8_t ack_status = 0;

  switch (tx_state_) {
    case TxState::IDLE: {
      if (!has_active_command_ && !dequeue_command_(active_command_))
        return;
      has_active_command_ = true;
      tx_retry_count_ = 0;
      drop_active_after_disable_ = false;
      send_enable_config_();
      return;
    }

    case TxState::WAIT_ENABLE_ACK:
      if (consume_tx_ack_(tx_expected_ack_cmd_, ack_status)) {
        if (ack_status == 0) {
          config_session_open_ = true;
          send_active_command_();
          return;
        }
        if (tx_retry_count_ < COMMAND_MAX_RETRIES) {
          tx_retry_count_++;
          ESP_LOGW(TAG_LD2450, "Enable config ACK failed (status=%u), retry %u/%u", ack_status, tx_retry_count_,
                   COMMAND_MAX_RETRIES);
          send_enable_config_();
          return;
        }
        if (config_session_open_) {
          drop_active_after_disable_ = true;
          send_disable_config_();
          return;
        }
        drop_active_command_("enable config ack failure");
        return;
      }
      if (deadline_passed_(tx_deadline_ms_, now)) {
        ack_timeout_count_++;
        if (tx_retry_count_ < COMMAND_MAX_RETRIES) {
          tx_retry_count_++;
          ESP_LOGW(TAG_LD2450, "Enable config ACK timeout, retry %u/%u", tx_retry_count_, COMMAND_MAX_RETRIES);
          send_enable_config_();
          return;
        }
        if (config_session_open_) {
          drop_active_after_disable_ = true;
          send_disable_config_();
          return;
        }
        drop_active_command_("enable config ack timeout");
      }
      return;

    case TxState::WAIT_COMMAND_ACK:
      if (consume_tx_ack_(tx_expected_ack_cmd_, ack_status)) {
        if (ack_status == 0) {
          send_disable_config_();
          return;
        }
        if (tx_retry_count_ < COMMAND_MAX_RETRIES) {
          tx_retry_count_++;
          ESP_LOGW(TAG_LD2450, "Command ACK failed (cmd=0x%04X status=%u), retry %u/%u", tx_expected_ack_cmd_,
                   ack_status, tx_retry_count_, COMMAND_MAX_RETRIES);
          send_enable_config_();
          return;
        }
        if (config_session_open_) {
          drop_active_after_disable_ = true;
          send_disable_config_();
          return;
        }
        drop_active_command_("command ack retries exhausted");
        return;
      }
      if (deadline_passed_(tx_deadline_ms_, now)) {
        ack_timeout_count_++;
        if (tx_retry_count_ < COMMAND_MAX_RETRIES) {
          tx_retry_count_++;
          ESP_LOGW(TAG_LD2450, "Command ACK timeout (cmd=0x%04X), retry %u/%u", tx_expected_ack_cmd_, tx_retry_count_,
                   COMMAND_MAX_RETRIES);
          send_enable_config_();
          return;
        }
        if (config_session_open_) {
          drop_active_after_disable_ = true;
          send_disable_config_();
          return;
        }
        drop_active_command_("command ack timeout");
      }
      return;

    case TxState::WAIT_DISABLE_ACK:
      if (consume_tx_ack_(tx_expected_ack_cmd_, ack_status)) {
        if (ack_status != 0)
          ESP_LOGW(TAG_LD2450, "Disable config ACK failed (status=%u)", ack_status);
        config_session_open_ = false;
        if (drop_active_after_disable_) {
          drop_active_command_("command ack retries exhausted");
          return;
        }
        has_active_command_ = false;
        tx_retry_count_ = 0;
        tx_expected_ack_cmd_ = 0;
        tx_state_ = TxState::IDLE;
        return;
      }
      if (deadline_passed_(tx_deadline_ms_, now)) {
        ack_timeout_count_++;
        ESP_LOGW(TAG_LD2450, "Disable config ACK timeout");
        config_session_open_ = false;
        if (drop_active_after_disable_) {
          drop_active_command_("disable config ack timeout after command failure");
          return;
        }
        has_active_command_ = false;
        tx_retry_count_ = 0;
        tx_expected_ack_cmd_ = 0;
        tx_state_ = TxState::IDLE;
      }
      return;
  }
}

void LD2450Handler::send_command_(const uint8_t *cmd, size_t len) { enqueue_command_(cmd, len); }

int LD2450Handler::get_stability_threshold_() {
  int val = static_cast<int>(config_.stability);
  return (val < 0) ? 0 : val;
}

void LD2450Handler::debounce_sensor_(int &pub, int &cand, int &streak, int raw, int threshold, sensor::Sensor *s) {
  if (s == nullptr)
    return;
  if (threshold <= 0 || pub == -1) {
    pub = raw;
    s->publish_state(static_cast<float>(raw));
    return;
  }
  if (raw == pub) {
    streak = 0;
    return;
  }
  if (raw == cand) {
    streak++;
  } else {
    cand = raw;
    streak = 1;
  }
  if (streak >= threshold) {
    pub = raw;
    s->publish_state(static_cast<float>(raw));
    streak = 0;
  }
}

void LD2450Handler::debounce_binary_(int &pub, int &cand, int &streak, bool raw, int threshold,
                                     binary_sensor::BinarySensor *s) {
  if (s == nullptr)
    return;
  int raw_int = raw ? 1 : 0;
  if (threshold <= 0 || pub == -1) {
    pub = raw_int;
    s->publish_state(raw);
    return;
  }
  if (raw_int == pub) {
    streak = 0;
    return;
  }
  if (raw_int == cand) {
    streak++;
  } else {
    cand = raw_int;
    streak = 1;
  }
  if (streak >= threshold) {
    pub = raw_int;
    s->publish_state(raw);
    streak = 0;
  }
}

void LD2450Handler::debounce_zone_(int z, const std::string &raw, int threshold) {
  if (zone_state[z] == nullptr)
    return;
  if (threshold <= 0 || pub_zone_state_[z].empty()) {
    pub_zone_state_[z] = raw;
    zone_state[z]->publish_state(raw);
    return;
  }
  if (raw == pub_zone_state_[z]) {
    streak_zone_[z] = 0;
    return;
  }
  if (raw == cand_zone_state_[z]) {
    streak_zone_[z]++;
  } else {
    cand_zone_state_[z] = raw;
    streak_zone_[z] = 1;
  }
  if (streak_zone_[z] >= threshold) {
    pub_zone_state_[z] = raw;
    zone_state[z]->publish_state(raw);
    streak_zone_[z] = 0;
  }
}

void LD2450Handler::debounce_target_state_(const std::string &raw, int threshold) {
  if (target_state_text_sensor == nullptr)
    return;
  if (threshold <= 0 || pub_target_state_.empty()) {
    pub_target_state_ = raw;
    target_state_text_sensor->publish_state(raw);
    return;
  }
  if (raw == pub_target_state_) {
    streak_target_state_ = 0;
    return;
  }
  if (raw == cand_target_state_) {
    streak_target_state_++;
  } else {
    cand_target_state_ = raw;
    streak_target_state_ = 1;
  }
  if (streak_target_state_ >= threshold) {
    pub_target_state_ = raw;
    target_state_text_sensor->publish_state(raw);
    streak_target_state_ = 0;
  }
}

void LD2450Handler::publish_sensor_(sensor::Sensor *s, float val) {
  if (s != nullptr)
    s->publish_state(val);
}

}  // namespace satellite1_radar
}  // namespace esphome
