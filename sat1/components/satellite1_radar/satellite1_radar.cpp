#include "satellite1_radar.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstring>

namespace esphome {
namespace satellite1_radar {

static const char *const TAG = "satellite1_radar";
static constexpr size_t MAX_DETECT_BYTES_PER_LOOP = 128;
static constexpr size_t MAX_DETECT_DRAIN_BYTES = 256;

static const uint8_t LD2410_FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t LD2450_FRAME_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t LD24XX_DISABLE_CONFIG[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};

static void drain_uart_bytes_(uart::UARTDevice *uart, size_t max_bytes) {
  size_t drained = 0;
  while (uart->available() && drained < max_bytes) {
    uint8_t discard;
    uart->read_byte(&discard);
    drained++;
  }
}

void Satellite1Radar::setup() {
  ESP_LOGI(TAG, "Starting mmWave radar auto-detection (%.1fs timeout)...", DETECT_TIMEOUT_MS / 1000.0f);
  if (this->radar_type_text_sensor_ != nullptr)
    this->radar_type_text_sensor_->publish_state("UNKNOWN");
  this->pre_detect_recovery_pending_ = true;
  this->pre_detect_recovery_sent_ms_ = 0;
  this->detection_started_ = true;
  this->detection_complete_ = false;
  this->detected_type_ = RadarType::UNKNOWN;
  this->detect_start_ms_ = millis();
  this->detect_ring_pos_ = 0;
  memset(this->detect_ring_buf_, 0, sizeof(this->detect_ring_buf_));
}

void Satellite1Radar::loop() {
  if (!this->detection_complete_) {
    this->process_detection_();
    return;
  }

  // Normal operation after detection
  if (detected_type_ == RadarType::LD2410 && ld2410_ != nullptr) {
    ld2410_->loop();
    if (this->write_config_pending_.exchange(false)) {
      ld2410_->apply_backend_config();
    }
  } else if (detected_type_ == RadarType::LD2450 && ld2450_ != nullptr) {
    ld2450_->loop();
  }
}

void Satellite1Radar::process_detection_() {
  if (!this->detection_started_ || this->detection_complete_) {
    return;
  }

  if (this->pre_detect_recovery_pending_) {
    if (this->pre_detect_recovery_sent_ms_ == 0) {
      drain_uart_bytes_(this, MAX_DETECT_DRAIN_BYTES);
      this->write_array(LD24XX_DISABLE_CONFIG, sizeof(LD24XX_DISABLE_CONFIG));
      this->flush();
      this->pre_detect_recovery_sent_ms_ = millis();
      ESP_LOGD(TAG, "Sent pre-detect disable-config recovery frame");
      return;
    }

    if (millis() - this->pre_detect_recovery_sent_ms_ < PRE_DETECT_RECOVERY_SETTLE_MS) {
      return;
    }

    drain_uart_bytes_(this, MAX_DETECT_DRAIN_BYTES);
    this->pre_detect_recovery_pending_ = false;
  }

  size_t bytes_processed = 0;
  while (this->available() && bytes_processed < MAX_DETECT_BYTES_PER_LOOP) {
    uint8_t byte;
    if (!this->read_byte(&byte))
      break;
    bytes_processed++;

    this->detect_ring_buf_[this->detect_ring_pos_ % 4] = byte;
    this->detect_ring_pos_++;

    if (this->detect_ring_pos_ < 4)
      continue;

    uint8_t seq[4];
    for (int i = 0; i < 4; i++) {
      seq[i] = this->detect_ring_buf_[(this->detect_ring_pos_ - 4 + static_cast<size_t>(i)) % 4];
    }

    if (memcmp(seq, LD2410_FRAME_HEADER, 4) == 0) {
      ESP_LOGI(TAG, "Detected LD2410 radar (frame header 0xF4F3F2F1)");
      drain_uart_bytes_(this, MAX_DETECT_DRAIN_BYTES);
      this->finalize_detection_(RadarType::LD2410);
      return;
    }

    if (memcmp(seq, LD2450_FRAME_HEADER, 4) == 0) {
      ESP_LOGI(TAG, "Detected LD2450 radar (frame header 0xAAFF0300)");
      drain_uart_bytes_(this, MAX_DETECT_DRAIN_BYTES);
      this->finalize_detection_(RadarType::LD2450);
      return;
    }
  }

  if (millis() - this->detect_start_ms_ >= DETECT_TIMEOUT_MS) {
    ESP_LOGI(TAG, "No mmWave radar detected within timeout");
    this->finalize_detection_(RadarType::NONE);
  }
}

void Satellite1Radar::create_common_entities_() {
  const bool radar_present = (this->detected_type_ == RadarType::LD2410 || this->detected_type_ == RadarType::LD2450);

  if (radar_present && this->runtime_tuner_switch_ == nullptr) {
    this->runtime_tuner_switch_.reset(new Satellite1RadarTunerSwitch());
    this->runtime_tuner_switch_->set_write_callback([this](bool state) {
      if (state)
        this->start_tuner();
      else
        this->stop_tuner();
    });
    this->runtime_tuner_switch_->configure_dynamic("Radar Tuner WebUI", ENTITY_CATEGORY_DIAGNOSTIC, false,
                                                   this->icon_meta_.tune_vertical);
    App.register_switch(this->runtime_tuner_switch_.get());
    this->runtime_tuner_switch_->publish_state(false);
  }
}

void Satellite1Radar::finalize_detection_(RadarType type) {
  this->detected_type_ = type;
  this->detection_complete_ = true;

  this->create_common_entities_();

  if (radar_type_text_sensor_ != nullptr) {
    switch (type) {
      case RadarType::LD2410:
        radar_type_text_sensor_->publish_state("LD2410");
        break;
      case RadarType::LD2450:
        radar_type_text_sensor_->publish_state("LD2450");
        break;
      default:
        radar_type_text_sensor_->publish_state("None");
        break;
    }
  }

  if (type == RadarType::NONE) {
    ESP_LOGI(TAG, "No radar detected, disabling loop");
    this->disable_loop();
    return;
  }

  if (type == RadarType::LD2410) {
    ld2410_ = std::unique_ptr<LD2410Handler>(new LD2410Handler(*this));
    ld2450_.reset();
    ld2410_->set_device_class_indices(this->device_class_meta_);
    ld2410_->set_unit_indices(this->unit_meta_);
    ld2410_->set_icon_indices(this->icon_meta_);
    ld2410_->create_and_register_entities();
    ld2410_->setup();
  } else if (type == RadarType::LD2450) {
    ld2450_ = std::unique_ptr<LD2450Handler>(new LD2450Handler(*this));
    ld2410_.reset();
    ld2450_->set_device_class_indices(this->device_class_meta_);
    ld2450_->set_unit_indices(this->unit_meta_);
    ld2450_->set_icon_indices(this->icon_meta_);
    ld2450_->setup();
    ld2450_->create_and_register_entities();
  }
}

void Satellite1Radar::dump_config() {
  ESP_LOGCONFIG(TAG, "Satellite1 Radar:");
  if (!detection_complete_) {
    ESP_LOGCONFIG(TAG, "  Detection: in progress...");
    return;
  }
  switch (detected_type_) {
    case RadarType::LD2410:
      ESP_LOGCONFIG(TAG, "  Detected sensor: LD2410");
      break;
    case RadarType::LD2450:
      ESP_LOGCONFIG(TAG, "  Detected sensor: LD2450");
      break;
    case RadarType::NONE:
      ESP_LOGCONFIG(TAG, "  Detected sensor: None");
      break;
    default:
      ESP_LOGCONFIG(TAG, "  Detected sensor: Unknown");
      break;
  }
}

void Satellite1Radar::start_tuner() {
  tuner_server_.clear_registrations();

  if (detected_type_ == RadarType::LD2450) {
    tuner_server_.set_html_content(ld2450_html_gz_, ld2450_html_gz_len_);
    tuner_server_.set_ld2450_handler(ld2450_.get());

    if (ld2450_ != nullptr) {
      ld2450_->on_target_update = [this](int target, float x, float y) { tuner_server_.update_target(target, x, y); };
    }

  } else if (detected_type_ == RadarType::LD2410) {
    tuner_server_.set_html_content(ld2410_html_gz_, ld2410_html_gz_len_);
    tuner_server_.set_ld2410_handler(ld2410_.get());
    tuner_server_.set_ld2410_apply_callback([this]() { this->write_config_pending_.store(true); });

    if (ld2410_ != nullptr)
      ld2410_->enable_engineering_mode();
  }

  tuner_server_.start();
}

void Satellite1Radar::stop_tuner() {
  if (detected_type_ == RadarType::LD2450) {
    if (ld2450_ != nullptr)
      ld2450_->on_target_update = nullptr;
  } else if (detected_type_ == RadarType::LD2410) {
    if (ld2410_ != nullptr)
      ld2410_->disable_engineering_mode();
  }
  tuner_server_.stop();
}

}  // namespace satellite1_radar
}  // namespace esphome
