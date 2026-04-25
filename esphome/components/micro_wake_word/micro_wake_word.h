#pragma once

#ifdef USE_ESP32

#include "preprocessor_settings.h"
#include "streaming_model.h"

#include "esphome/components/microphone/microphone_source.h"

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/ring_buffer.h"

#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

#include <freertos/event_groups.h>

#include <atomic>
#include <frontend.h>
#include <frontend_util.h>
#include <string>
#include <vector>

namespace esphome {
namespace micro_wake_word {

enum State {
  STARTING,
  DETECTING_WAKE_WORD,
  STOPPING,
  STOPPED,
};

class MicroWakeWord : public Component
#ifdef USE_OTA_STATE_LISTENER
    ,
                      public ota::OTAGlobalStateListener
#endif
{
 public:
  struct CaptureUploadRequest {
    MicroWakeWord *parent;
    std::string upload_url;
    std::string source_device;
    std::string wake_word;
    std::vector<uint8_t> pcm_data;
    uint8_t max_probability;
    uint8_t average_probability;
    bool blocked_by_vad;
    std::string event_type;
  };

  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;

#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif

  void start();
  void stop();

  bool is_running() const { return this->state_ != State::STOPPED; }

  void set_features_step_size(uint8_t step_size) { this->features_step_size_ = step_size; }

  void set_microphone_source(microphone::MicrophoneSource *microphone_source) {
    this->microphone_source_ = microphone_source;
  }

  void set_stop_after_detection(bool stop_after_detection) { this->stop_after_detection_ = stop_after_detection; }
  void set_capture_upload_enabled(bool enabled) { this->capture_upload_enabled_.store(enabled); }
  bool get_capture_upload_enabled() const { return this->capture_upload_enabled_.load(); }
  void set_capture_close_misses_enabled(bool enabled) { this->capture_close_misses_enabled_.store(enabled); }
  bool get_capture_close_misses_enabled() const { return this->capture_close_misses_enabled_.load(); }
  void set_capture_close_miss_probability_cutoff(float cutoff) {
    if (cutoff < 0.0f) {
      cutoff = 0.0f;
    } else if (cutoff > 1.0f) {
      cutoff = 1.0f;
    }
    this->capture_close_miss_probability_cutoff_.store(static_cast<uint8_t>(cutoff * 255.0f));
  }
  float get_capture_close_miss_probability_cutoff() const {
    return this->capture_close_miss_probability_cutoff_.load() / 255.0f;
  }
  void set_capture_upload_url(const std::string &capture_upload_url) { this->capture_upload_url_ = capture_upload_url; }
  const std::string &get_capture_upload_url() const { return this->capture_upload_url_; }

  Trigger<std::string> *get_wake_word_detected_trigger() { return &this->wake_word_detected_trigger_; }

  void add_wake_word_model(WakeWordModel *model);

#ifdef USE_MICRO_WAKE_WORD_VAD
  void add_vad_model(const uint8_t *model_start, uint8_t probability_cutoff, size_t sliding_window_size,
                     size_t tensor_arena_size);

  // Intended for the voice assistant component to fetch VAD status
  bool get_vad_state() { return this->vad_state_; }
#endif

  // Intended for the voice assistant component to access which wake words are available
  // Since these are pointers to the WakeWordModel objects, the voice assistant component can enable or disable them
  std::vector<WakeWordModel *> get_wake_words();

 protected:
  microphone::MicrophoneSource *microphone_source_{nullptr};
  Trigger<std::string> wake_word_detected_trigger_;
  State state_{State::STOPPED};

  std::weak_ptr<RingBuffer> ring_buffer_;
  std::vector<WakeWordModel *> wake_word_models_;

#ifdef USE_MICRO_WAKE_WORD_VAD
  std::unique_ptr<VADModel> vad_model_;
  bool vad_state_{false};
#endif

  bool pending_start_{false};
  bool pending_stop_{false};

  bool stop_after_detection_;

  uint8_t features_step_size_;
  std::shared_ptr<RingBuffer> capture_ring_buffer_;
  std::atomic<bool> capture_upload_enabled_{false};
  std::atomic<bool> capture_close_misses_enabled_{false};
  std::atomic<uint8_t> capture_close_miss_probability_cutoff_{200};
  std::atomic<bool> capture_upload_in_progress_{false};
  uint32_t last_close_miss_upload_ms_{0};
  std::string capture_upload_url_;
  bool pending_close_miss_{false};
  DetectionEvent pending_close_miss_event_;
  uint32_t pending_close_miss_due_ms_{0};

  // Audio frontend handles generating spectrogram features
  struct FrontendConfig frontend_config_;
  struct FrontendState frontend_state_;

  // Handles managing the stop/state of the inference task
  EventGroupHandle_t event_group_;

  // Used to send messages about the models' states to the main loop
  QueueHandle_t detection_queue_;

  static void inference_task(void *params);
  TaskHandle_t inference_task_handle_{nullptr};

  /// @brief Suspends the inference task
  void suspend_task_();
  /// @brief Resumes the inference task
  void resume_task_();

  void set_state_(State state);

  /// @brief Generates spectrogram features from an input buffer of audio samples
  /// @param audio_buffer (int16_t *) Buffer containing input audio samples
  /// @param samples_available (size_t) Number of samples avaiable in the input buffer
  /// @param features_buffer (int8_t *) Buffer to store generated features
  /// @return (size_t) Number of samples processed from the input buffer
  size_t generate_features_(int16_t *audio_buffer, size_t samples_available,
                            int8_t features_buffer[PREPROCESSOR_FEATURE_SIZE]);

  /// @brief Processes any new probabilities for each model. If any wake word is detected, it will send a DetectionEvent
  /// to the detection_queue_.
  void process_probabilities_();

  /// @brief Deletes each model's TFLite interpreters and frees tensor arena memory.
  void unload_models_();

  /// @brief Runs an inference with each model using the new spectrogram features
  /// @param audio_features (int8_t *) Buffer containing new spectrogram features
  /// @return True if successful, false if any errors were encountered
  bool update_model_probabilities_(const int8_t audio_features[PREPROCESSOR_FEATURE_SIZE]);

  bool capture_feature_enabled_() const;
  bool should_capture_close_miss_(const DetectionEvent &detection_event);
  void note_close_miss_upload_();
  void queue_pending_close_miss_(const DetectionEvent &detection_event);
  void clear_pending_close_miss_(const std::string *wake_word = nullptr);
  void flush_pending_close_miss_();
  std::string build_capture_upload_url_() const;
  bool snapshot_capture_audio_(std::vector<uint8_t> &audio_bytes);
  void queue_detection_capture_(const DetectionEvent &detection_event, DetectionEventType event_type);
  bool upload_capture_(const CaptureUploadRequest &request);
  static void capture_upload_task(void *params);
};

}  // namespace micro_wake_word
}  // namespace esphome

#endif  // USE_ESP32
