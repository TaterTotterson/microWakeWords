#pragma once

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"
#include "esphome/components/microphone/microphone_source.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/ring_buffer.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace esphome {
namespace remote_wake_word {

enum class State {
  STARTING,
  DETECTING_WAKE_WORD,
  STOPPING,
  STOPPED,
};

struct RemoteWakeWordEvent {
  enum Type {
    DETECTION,
    ERROR,
  };
  Type type;
  std::string wake_word;
  std::string message;
  float score;
};

class RemoteWakeWord : public Component {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;

  void start();
  void stop();
  bool is_running() const { return this->state_ != State::STOPPED || this->task_handle_ != nullptr; }

  void set_microphone_source(microphone::MicrophoneSource *microphone_source) {
    this->microphone_source_ = microphone_source;
  }
  void set_url(const std::string &url) { this->url_ = url; }
  void set_wake_word(const std::string &wake_word) { this->wake_word_ = wake_word; }
  void set_source_device(const std::string &source_device) { this->source_device_ = source_device; }
  void set_chunk_duration_ms(uint32_t chunk_duration_ms) {
    if (chunk_duration_ms < 100) {
      chunk_duration_ms = 100;
    } else if (chunk_duration_ms > 2000) {
      chunk_duration_ms = 2000;
    }
    this->chunk_duration_ms_ = chunk_duration_ms;
  }
  void set_max_failures(uint8_t max_failures) {
    if (max_failures == 0) {
      max_failures = 1;
    }
    this->max_failures_ = max_failures;
  }
  void set_http_timeout_ms(uint32_t http_timeout_ms) {
    if (http_timeout_ms < 250) {
      http_timeout_ms = 250;
    } else if (http_timeout_ms > 10000) {
      http_timeout_ms = 10000;
    }
    this->http_timeout_ms_ = http_timeout_ms;
  }

  Trigger<std::string> *get_wake_word_detected_trigger() { return &this->wake_word_detected_trigger_; }
  Trigger<std::string> *get_error_trigger() { return &this->error_trigger_; }

 protected:
  microphone::MicrophoneSource *microphone_source_{nullptr};
  std::weak_ptr<RingBuffer> ring_buffer_;
  Trigger<std::string> wake_word_detected_trigger_;
  Trigger<std::string> error_trigger_;
  State state_{State::STOPPED};
  TaskHandle_t task_handle_{nullptr};
  QueueHandle_t event_queue_{nullptr};

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> task_started_{false};
  std::atomic<bool> task_done_{false};
  std::atomic<uint8_t> consecutive_failures_{0};

  std::string url_;
  std::string wake_word_;
  std::string source_device_;
  uint32_t chunk_duration_ms_{500};
  uint8_t max_failures_{3};
  uint32_t http_timeout_ms_{3000};

  static void detection_task(void *params);
  std::string build_stream_endpoint_url_(const audio::AudioStreamInfo &stream_info) const;
  void queue_event_(RemoteWakeWordEvent::Type type, const std::string &wake_word, const std::string &message,
                    float score);
  void set_state_(State state);
};

}  // namespace remote_wake_word
}  // namespace esphome

#endif  // USE_ESP32
