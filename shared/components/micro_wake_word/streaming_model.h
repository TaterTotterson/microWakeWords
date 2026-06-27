#pragma once

#ifdef USE_ESP32

#include "preprocessor_settings.h"

#include "esphome/core/preferences.h"

#include <tensorflow/lite/core/c/common.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>

namespace esphome {
namespace micro_wake_word {

static const uint8_t MIN_SLICES_BEFORE_DETECTION = 100;
static const uint32_t STREAMING_MODEL_VARIABLE_ARENA_SIZE = 1024;
static const size_t DETECTION_HISTORY_SIZE = 16;

enum class DetectionProfile : uint8_t {
  VERY_SENSITIVE = 0,
  BALANCED = 1,
  STRICT = 2,
  TV_NEARBY = 3,
};

const char *detection_profile_to_string(DetectionProfile profile);
DetectionProfile detection_profile_from_string(const std::string &profile);

enum class DetectionEventType : uint8_t {
  NONE = 0,
  WAKE_DETECTED,
  CLOSE_MISS,
  BLOCKED_BY_VAD,
};

struct DetectionEvent {
  std::string *wake_word{nullptr};
  bool detected{false};
  bool partially_detection{false};  // Set if the close-miss capture threshold is crossed without a wake trigger.
  uint8_t max_probability{0};
  uint8_t average_probability{0};
  uint8_t probability_history[DETECTION_HISTORY_SIZE]{0};
  uint8_t probability_history_size{0};
  uint8_t probability_cutoff{0};
  uint8_t peak_probability_cutoff{0};
  uint8_t active_window_count{0};
  uint8_t min_active_windows{0};
  int16_t rise_score{0};
  uint8_t vad_max_probability{0};
  uint8_t vad_average_probability{0};
  DetectionProfile detection_profile{DetectionProfile::BALANCED};
  bool blocked_by_vad{false};
  DetectionEventType event_type{DetectionEventType::NONE};
};

static_assert(std::is_trivially_copyable<DetectionEvent>::value,
              "DetectionEvent is copied through a FreeRTOS queue and must stay trivially copyable.");

class StreamingModel {
 public:
  virtual void log_model_config() = 0;
  virtual DetectionEvent determine_detected() = 0;

  // Performs inference on the given features.
  //  - If the model is enabled but not loaded, it will load it
  //  - If the model is disabled but loaded, it will unload it
  // Returns true if sucessful or false if there is an error
  bool perform_streaming_inference(const int8_t features[PREPROCESSOR_FEATURE_SIZE]);

  /// @brief Sets all recent_streaming_probabilities to 0 and resets the ignore window count
  void reset_probabilities();

  /// @brief Destroys the TFLite interpreter and frees the tensor and variable arenas' memory
  void unload_model();

  /// @brief Enable the model. The next performing_streaming_inference call will load it.
  virtual void enable() { this->enabled_.store(true); }

  /// @brief Disable the model. The next performing_streaming_inference call will unload it.
  virtual void disable() { this->enabled_.store(false); }

  /// @brief Set model enabled state without saving preferences.
  ///
  /// This is for temporary voice pipeline state changes. Persisted enable/disable
  /// writes can briefly disable flash cache, which is unsafe while inference is
  /// running from a response/stop-word path.
  void set_enabled_temporarily(bool enabled) { this->enabled_.store(enabled); }

  /// @brief Return true if the model is enabled.
  bool is_enabled() const { return this->enabled_.load(); }

  bool get_unprocessed_probability_status() const { return this->unprocessed_probability_status_; }

  // Quantized probability cutoffs mapping 0.0 - 1.0 to 0 - 255
  uint8_t get_default_probability_cutoff() const { return this->default_probability_cutoff_; }
  uint8_t get_probability_cutoff() const { return this->probability_cutoff_; }
  void set_probability_cutoff(uint8_t probability_cutoff) { this->probability_cutoff_ = probability_cutoff; }
  void set_detection_profile(DetectionProfile profile);
  DetectionProfile get_detection_profile() const;

 protected:
  /// @brief Allocates tensor and variable arenas and sets up the model interpreter
  /// @return True if successful, false otherwise
  bool load_model_();
  /// @brief Probes the actual required tensor arena size by trial allocation.
  /// Tries the manifest size first, then 2x if that fails.
  /// @return The required arena size rounded up to 16-byte alignment, or 0 on failure.
  size_t probe_arena_size_();
  /// @brief Validate that all model operators are supported by this firmware before TFLM allocation.
  bool validate_model_ops_(const tflite::Model *model) const;
  /// @brief Returns true if successfully registered the streaming model's TensorFlow operations
  bool register_streaming_ops_(tflite::MicroMutableOpResolver<20> &op_resolver);
  size_t probability_index_(size_t chronological_offset) const;
  void fill_probability_history_(DetectionEvent &detection_event) const;
  uint8_t effective_peak_probability_cutoff_(DetectionProfile profile) const;
  uint8_t minimum_active_windows_(DetectionProfile profile) const;
  int16_t rise_score_() const;
  int16_t minimum_rise_score_(DetectionProfile profile) const;

  tflite::MicroMutableOpResolver<20> streaming_op_resolver_;

  bool loaded_{false};
  std::atomic<bool> enabled_{true};
  bool tensor_arena_size_probed_{false};
  bool unprocessed_probability_status_{false};
  uint8_t current_stride_step_{0};
  int16_t ignore_windows_{-MIN_SLICES_BEFORE_DETECTION};

  uint8_t default_probability_cutoff_;
  uint8_t probability_cutoff_;
  std::atomic<uint8_t> detection_profile_{static_cast<uint8_t>(DetectionProfile::BALANCED)};
  size_t sliding_window_size_;

  size_t last_n_index_{0};
  size_t tensor_arena_size_;
  std::vector<uint8_t> recent_streaming_probabilities_;

  const uint8_t *model_start_;
  uint8_t *tensor_arena_{nullptr};
  uint8_t *var_arena_{nullptr};
  std::unique_ptr<tflite::MicroInterpreter> interpreter_;
  tflite::MicroResourceVariables *mrv_{nullptr};
  tflite::MicroAllocator *ma_{nullptr};
};

class WakeWordModel final : public StreamingModel {
 public:
  /// @brief Constructs a wake word model object
  /// @param id (std::string) identifier for this model
  /// @param model_start (const uint8_t *) pointer to the start of the model's TFLite FlatBuffer
  /// @param default_probability_cutoff (uint8_t) probability cutoff for acceping the wake word has been said
  /// @param sliding_window_average_size (size_t) the length of the sliding window computing the mean rolling
  ///                                    probability
  /// @param wake_word (std::string) Friendly name of the wake word
  /// @param tensor_arena_size (size_t) Size in bytes for allocating the tensor arena
  /// @param default_enabled (bool) If true, it will be enabled by default on first boot
  /// @param internal_only (bool) If true, the model will not be exposed to HomeAssistant as an available model
  WakeWordModel(const std::string &id, const uint8_t *model_start, uint8_t default_probability_cutoff,
                size_t sliding_window_average_size, const std::string &wake_word, size_t tensor_arena_size,
                bool default_enabled, bool internal_only);

  void log_model_config() override;

  /// @brief Checks for the wake word by comparing the mean probability in the sliding window with the probability
  /// cutoff
  /// @return True if wake word is detected, false otherwise
  DetectionEvent determine_detected() override;

  const std::string &get_id() const { return this->id_; }
  const std::string &get_wake_word() const { return this->wake_word_; }

  void add_trained_language(const std::string &language) {
    this->trained_languages_.push_back(language);
    this->compiled_trained_languages_.push_back(language);
  }
  const std::vector<std::string> &get_trained_languages() const { return this->trained_languages_; }

  bool replace_model(const uint8_t *model_start, uint8_t default_probability_cutoff, size_t sliding_window_average_size,
                     const std::string &wake_word, size_t tensor_arena_size,
                     const std::vector<std::string> &trained_languages);
  void restore_compiled_model();

  /// @brief Enable the model and save to flash. The next performing_streaming_inference call will load it.
  void enable() override;

  /// @brief Disable the model and save to flash. The next performing_streaming_inference call will unload it.
  void disable() override;

  bool get_internal_only() { return this->internal_only_; }

 protected:
  std::string id_;
  std::string wake_word_;
  std::vector<std::string> trained_languages_;

  bool internal_only_;

  ESPPreferenceObject pref_;

  const uint8_t *compiled_model_start_{nullptr};
  uint8_t compiled_default_probability_cutoff_{0};
  size_t compiled_sliding_window_size_{0};
  std::string compiled_wake_word_;
  size_t compiled_tensor_arena_size_{0};
  std::vector<std::string> compiled_trained_languages_;
};

class VADModel final : public StreamingModel {
 public:
  VADModel(const uint8_t *model_start, uint8_t default_probability_cutoff, size_t sliding_window_size,
           size_t tensor_arena_size);

  void log_model_config() override;

  /// @brief Checks for voice activity by comparing the max probability in the sliding window with the probability
  /// cutoff
  /// @return True if voice activity is detected, false otherwise
  DetectionEvent determine_detected() override;
};

}  // namespace micro_wake_word
}  // namespace esphome

#endif
