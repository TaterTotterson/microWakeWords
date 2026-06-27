#include "streaming_model.h"

#ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "tensorflow/lite/schema/schema_generated.h"

#include <algorithm>
#include <cctype>

static const char *const TAG = "micro_wake_word";

namespace esphome {
namespace micro_wake_word {

const char *detection_profile_to_string(DetectionProfile profile) {
  switch (profile) {
    case DetectionProfile::VERY_SENSITIVE:
      return "very_sensitive";
    case DetectionProfile::BALANCED:
      return "balanced";
    case DetectionProfile::STRICT:
      return "strict";
    case DetectionProfile::TV_NEARBY:
      return "tv_nearby";
    default:
      return "balanced";
  }
}

DetectionProfile detection_profile_from_string(const std::string &profile) {
  std::string normalized;
  normalized.reserve(profile.size());
  for (char ch : profile) {
    if (ch == ' ' || ch == '-') {
      normalized.push_back('_');
    } else {
      normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
  }

  if (normalized == "very_sensitive" || normalized == "sensitive") {
    return DetectionProfile::VERY_SENSITIVE;
  }
  if (normalized == "strict") {
    return DetectionProfile::STRICT;
  }
  if (normalized == "tv_nearby" || normalized == "tv" || normalized == "near_tv") {
    return DetectionProfile::TV_NEARBY;
  }
  return DetectionProfile::BALANCED;
}

void WakeWordModel::log_model_config() {
  ESP_LOGCONFIG(TAG,
                "    - Wake Word: %s\n"
                "      Probability cutoff: %.2f\n"
                "      Sliding window size: %d\n"
                "      Detection profile: %s",
                this->wake_word_.c_str(), this->probability_cutoff_ / 255.0f, this->sliding_window_size_,
                detection_profile_to_string(this->get_detection_profile()));
}

void VADModel::log_model_config() {
  ESP_LOGCONFIG(TAG,
                "    - VAD Model\n"
                "      Probability cutoff: %.2f\n"
                "      Sliding window size: %d",
                this->probability_cutoff_ / 255.0f, this->sliding_window_size_);
}

bool StreamingModel::load_model_() {
  RAMAllocator<uint8_t> arena_allocator;

  if (this->var_arena_ == nullptr) {
    this->var_arena_ = arena_allocator.allocate(STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    if (this->var_arena_ == nullptr) {
      ESP_LOGE(TAG, "Could not allocate the streaming model's variable tensor arena.");
      return false;
    }
    this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);
  }

  const tflite::Model *model = tflite::GetModel(this->model_start_);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    ESP_LOGE(TAG, "Streaming model's schema is not supported");
    return false;
  }
  if (!this->validate_model_ops_(model)) {
    return false;
  }

  // Probe for the actual required tensor arena size if not yet determined
  if (!this->tensor_arena_size_probed_) {
    size_t probed_size = this->probe_arena_size_();
    if (probed_size > 0) {
      ESP_LOGD(TAG, "Probed tensor arena size: %zu bytes", probed_size);
      this->tensor_arena_size_ = probed_size;
    } else {
      ESP_LOGW(TAG, "Arena size probe failed, using manifest size: %zu bytes", this->tensor_arena_size_);
    }
    this->tensor_arena_size_probed_ = true;
  }

  if (this->tensor_arena_ == nullptr) {
    this->tensor_arena_ = arena_allocator.allocate(this->tensor_arena_size_);
    if (this->tensor_arena_ == nullptr) {
      ESP_LOGE(TAG, "Could not allocate the streaming model's tensor arena.");
      return false;
    }
  }

  if (this->interpreter_ == nullptr) {
    this->interpreter_ =
        make_unique<tflite::MicroInterpreter>(tflite::GetModel(this->model_start_), this->streaming_op_resolver_,
                                              this->tensor_arena_, this->tensor_arena_size_, this->mrv_);
    if (this->interpreter_->AllocateTensors() != kTfLiteOk) {
      ESP_LOGE(TAG, "Failed to allocate tensors for the streaming model");
      return false;
    }

    // Verify input tensor matches expected values
    // Dimension 3 will represent the first layer stride, so skip it may vary
    TfLiteTensor *input = this->interpreter_->input(0);
    if ((input->dims->size != 3) || (input->dims->data[0] != 1) ||
        (input->dims->data[2] != PREPROCESSOR_FEATURE_SIZE)) {
      ESP_LOGE(TAG, "Streaming model tensor input dimensions has improper dimensions.");
      return false;
    }

    if (input->type != kTfLiteInt8) {
      ESP_LOGE(TAG, "Streaming model tensor input is not int8.");
      return false;
    }

    // Verify output tensor matches expected values
    TfLiteTensor *output = this->interpreter_->output(0);
    if ((output->dims->size != 2) || (output->dims->data[0] != 1) || (output->dims->data[1] != 1)) {
      ESP_LOGE(TAG, "Streaming model tensor output dimension is not 1x1.");
      return false;
    }

    if (output->type != kTfLiteUInt8) {
      ESP_LOGE(TAG, "Streaming model tensor output is not uint8.");
      return false;
    }
  }

  this->loaded_ = true;
  this->reset_probabilities();
  return true;
}

size_t StreamingModel::probe_arena_size_() {
  RAMAllocator<uint8_t> arena_allocator;

  // Try with the manifest size first, then escalates to 1.5, then 2x if it fails. Different platforms and different
  // versions of the esp-nn library require different amounts of memory, so the manifest size may not always be correct,
  // and probing allows us to find the actual required size for the current build and platform. Aligns test sizes to 16
  // bytes.
  size_t attempt_sizes[] = {(this->tensor_arena_size_ + 15) & ~15, (this->tensor_arena_size_ * 3 / 2 + 15) & ~15,
                            (this->tensor_arena_size_ * 2 + 15) & ~15};

  for (size_t attempt_size : attempt_sizes) {
    uint8_t *probe_arena = arena_allocator.allocate(attempt_size);
    if (probe_arena == nullptr) {
      continue;
    }

    // Verify the model works at all with this arena size
    auto probe_interpreter = make_unique<tflite::MicroInterpreter>(
        tflite::GetModel(this->model_start_), this->streaming_op_resolver_, probe_arena, attempt_size, this->mrv_);

    if (probe_interpreter->AllocateTensors() != kTfLiteOk) {
      probe_interpreter.reset();
      arena_allocator.deallocate(probe_arena, attempt_size);
      this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
      this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);
      continue;
    }

    // Try to shrink the arena. Start with arena_used_bytes() + 16 (rounded to 16-byte alignment).
    // If that works, use it. Otherwise, try midpoints between that and the full size until one succeeds.
    size_t lower = (probe_interpreter->arena_used_bytes() + 16 + 15) & ~15;
    probe_interpreter.reset();
    this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);

    size_t upper = attempt_size;

    while (lower < upper) {
      auto test_interpreter = make_unique<tflite::MicroInterpreter>(
          tflite::GetModel(this->model_start_), this->streaming_op_resolver_, probe_arena, lower, this->mrv_);

      bool ok = test_interpreter->AllocateTensors() == kTfLiteOk;

      test_interpreter.reset();
      this->ma_ = tflite::MicroAllocator::Create(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
      this->mrv_ = tflite::MicroResourceVariables::Create(this->ma_, 20);

      if (ok) {
        // Found a working size smaller than the full arena
        upper = lower + 16;  // Pad by 16 bytes to be safe for future allocations
        break;
      }

      // Try the midpoint between current attempt and full size
      lower = ((lower + upper) / 2 + 15) & ~15;
    }

    arena_allocator.deallocate(probe_arena, attempt_size);
    return upper;
  }

  return 0;
}

bool StreamingModel::validate_model_ops_(const tflite::Model *model) const {
  if (model == nullptr || model->operator_codes() == nullptr) {
    ESP_LOGE(TAG, "Streaming model is missing operator metadata.");
    return false;
  }

  for (const tflite::OperatorCode *op_code : *model->operator_codes()) {
    switch (op_code->builtin_code()) {
      case tflite::BuiltinOperator_CALL_ONCE:
      case tflite::BuiltinOperator_VAR_HANDLE:
      case tflite::BuiltinOperator_RESHAPE:
      case tflite::BuiltinOperator_READ_VARIABLE:
      case tflite::BuiltinOperator_STRIDED_SLICE:
      case tflite::BuiltinOperator_CONCATENATION:
      case tflite::BuiltinOperator_ASSIGN_VARIABLE:
      case tflite::BuiltinOperator_CONV_2D:
      case tflite::BuiltinOperator_MUL:
      case tflite::BuiltinOperator_ADD:
      case tflite::BuiltinOperator_MEAN:
      case tflite::BuiltinOperator_FULLY_CONNECTED:
      case tflite::BuiltinOperator_LOGISTIC:
      case tflite::BuiltinOperator_QUANTIZE:
      case tflite::BuiltinOperator_DEPTHWISE_CONV_2D:
      case tflite::BuiltinOperator_AVERAGE_POOL_2D:
      case tflite::BuiltinOperator_MAX_POOL_2D:
      case tflite::BuiltinOperator_PAD:
      case tflite::BuiltinOperator_PACK:
      case tflite::BuiltinOperator_SPLIT_V:
        break;
      default:
        ESP_LOGW(TAG, "Streaming model uses unsupported operator code %d.", static_cast<int>(op_code->builtin_code()));
        return false;
    }
  }

  return true;
}

void StreamingModel::unload_model() {
  this->interpreter_.reset();

  RAMAllocator<uint8_t> arena_allocator;

  if (this->tensor_arena_ != nullptr) {
    arena_allocator.deallocate(this->tensor_arena_, this->tensor_arena_size_);
    this->tensor_arena_ = nullptr;
  }

  if (this->var_arena_ != nullptr) {
    arena_allocator.deallocate(this->var_arena_, STREAMING_MODEL_VARIABLE_ARENA_SIZE);
    this->var_arena_ = nullptr;
  }

  this->mrv_ = nullptr;
  this->ma_ = nullptr;
  this->loaded_ = false;
}

bool StreamingModel::perform_streaming_inference(const int8_t features[PREPROCESSOR_FEATURE_SIZE]) {
  const bool enabled = this->enabled_.load();

  if (enabled && !this->loaded_) {
    // Model is enabled but isn't loaded
    if (!this->load_model_()) {
      return false;
    }
  }

  if (!enabled && this->loaded_) {
    // Model is disabled but still loaded
    this->unload_model();
    return true;
  }

  if (this->loaded_) {
    TfLiteTensor *input = this->interpreter_->input(0);

    uint8_t stride = this->interpreter_->input(0)->dims->data[1];
    this->current_stride_step_ = this->current_stride_step_ % stride;

    std::memmove(
        (int8_t *) (tflite::GetTensorData<int8_t>(input)) + PREPROCESSOR_FEATURE_SIZE * this->current_stride_step_,
        features, PREPROCESSOR_FEATURE_SIZE);
    ++this->current_stride_step_;

    if (this->current_stride_step_ >= stride) {
      TfLiteStatus invoke_status = this->interpreter_->Invoke();
      if (invoke_status != kTfLiteOk) {
        ESP_LOGW(TAG, "Streaming interpreter invoke failed");
        return false;
      }

      TfLiteTensor *output = this->interpreter_->output(0);

      ++this->last_n_index_;
      if (this->last_n_index_ == this->sliding_window_size_)
        this->last_n_index_ = 0;
      this->recent_streaming_probabilities_[this->last_n_index_] = output->data.uint8[0];  // probability;
      this->unprocessed_probability_status_ = true;
    }
    if (this->recent_streaming_probabilities_[this->last_n_index_] < this->probability_cutoff_) {
      // Only increment ignore windows if less than the probability cutoff; this forces the model to "cool-off" from a
      // previous detection and calling ``reset_probabilities`` so it avoids duplicate detections
      this->ignore_windows_ = std::min(this->ignore_windows_ + 1, 0);
    }
  }
  return true;
}

void StreamingModel::reset_probabilities() {
  for (auto &prob : this->recent_streaming_probabilities_) {
    prob = 0;
  }
  this->ignore_windows_ = -MIN_SLICES_BEFORE_DETECTION;
}

void StreamingModel::set_detection_profile(DetectionProfile profile) {
  this->detection_profile_.store(static_cast<uint8_t>(profile));
}

DetectionProfile StreamingModel::get_detection_profile() const {
  const uint8_t profile = this->detection_profile_.load();
  if (profile > static_cast<uint8_t>(DetectionProfile::TV_NEARBY)) {
    return DetectionProfile::BALANCED;
  }
  return static_cast<DetectionProfile>(profile);
}

size_t StreamingModel::probability_index_(size_t chronological_offset) const {
  if (this->sliding_window_size_ == 0) {
    return 0;
  }
  return (this->last_n_index_ + 1 + chronological_offset) % this->sliding_window_size_;
}

void StreamingModel::fill_probability_history_(DetectionEvent &detection_event) const {
  const size_t history_size = std::min(this->sliding_window_size_, DETECTION_HISTORY_SIZE);
  detection_event.probability_history_size = static_cast<uint8_t>(history_size);
  const size_t start_offset = this->sliding_window_size_ - history_size;

  for (size_t i = 0; i < history_size; i++) {
    detection_event.probability_history[i] =
        this->recent_streaming_probabilities_[this->probability_index_(start_offset + i)];
  }
}

uint8_t StreamingModel::effective_peak_probability_cutoff_(DetectionProfile profile) const {
  switch (profile) {
    case DetectionProfile::STRICT:
      return std::max<uint8_t>(this->probability_cutoff_, 220);
    case DetectionProfile::TV_NEARBY:
      return std::max<uint8_t>(this->probability_cutoff_, 235);
    case DetectionProfile::VERY_SENSITIVE:
    case DetectionProfile::BALANCED:
    default:
      return this->probability_cutoff_;
  }
}

uint8_t StreamingModel::minimum_active_windows_(DetectionProfile profile) const {
  if (this->sliding_window_size_ == 0) {
    return 0;
  }

  size_t minimum = 1;
  switch (profile) {
    case DetectionProfile::VERY_SENSITIVE:
      minimum = 1;
      break;
    case DetectionProfile::STRICT:
      minimum = std::max<size_t>(2, ((this->sliding_window_size_ * 2) + 2) / 3);
      break;
    case DetectionProfile::TV_NEARBY:
      minimum = std::max<size_t>(3, ((this->sliding_window_size_ * 3) + 3) / 4);
      break;
    case DetectionProfile::BALANCED:
    default:
      minimum = std::max<size_t>(1, (this->sliding_window_size_ + 1) / 2);
      break;
  }

  return static_cast<uint8_t>(std::min(minimum, this->sliding_window_size_));
}

int16_t StreamingModel::rise_score_() const {
  if (this->sliding_window_size_ < 2) {
    return 0;
  }

  const size_t edge_count = this->sliding_window_size_ / 2;
  if (edge_count == 0) {
    return 0;
  }

  uint32_t early_sum = 0;
  uint32_t late_sum = 0;
  for (size_t i = 0; i < edge_count; i++) {
    early_sum += this->recent_streaming_probabilities_[this->probability_index_(i)];
    late_sum += this->recent_streaming_probabilities_[
        this->probability_index_(this->sliding_window_size_ - edge_count + i)];
  }

  const int32_t early_average = early_sum / edge_count;
  const int32_t late_average = late_sum / edge_count;
  return static_cast<int16_t>(std::max<int32_t>(-255, std::min<int32_t>(255, late_average - early_average)));
}

int16_t StreamingModel::minimum_rise_score_(DetectionProfile profile) const {
  switch (profile) {
    case DetectionProfile::STRICT:
      return -8;
    case DetectionProfile::TV_NEARBY:
      return 4;
    case DetectionProfile::VERY_SENSITIVE:
    case DetectionProfile::BALANCED:
    default:
      return -255;
  }
}

WakeWordModel::WakeWordModel(const std::string &id, const uint8_t *model_start, uint8_t default_probability_cutoff,
                             size_t sliding_window_average_size, const std::string &wake_word, size_t tensor_arena_size,
                             bool default_enabled, bool internal_only) {
  this->id_ = id;
  this->model_start_ = model_start;
  this->default_probability_cutoff_ = default_probability_cutoff;
  this->probability_cutoff_ = default_probability_cutoff;
  this->sliding_window_size_ = sliding_window_average_size;
  this->recent_streaming_probabilities_.resize(sliding_window_average_size, 0);
  this->wake_word_ = wake_word;
  this->tensor_arena_size_ = tensor_arena_size;
  this->register_streaming_ops_(this->streaming_op_resolver_);
  this->current_stride_step_ = 0;
  this->internal_only_ = internal_only;
  this->compiled_model_start_ = model_start;
  this->compiled_default_probability_cutoff_ = default_probability_cutoff;
  this->compiled_sliding_window_size_ = sliding_window_average_size;
  this->compiled_wake_word_ = wake_word;
  this->compiled_tensor_arena_size_ = tensor_arena_size;

  this->pref_ = global_preferences->make_preference<bool>(fnv1_hash(id));
  bool enabled;
  if (this->pref_.load(&enabled)) {
    // Use the enabled state loaded from flash
    this->enabled_.store(enabled);
  } else {
    // If no state saved, then use the default
    this->enabled_.store(default_enabled);
  }
};

bool WakeWordModel::replace_model(const uint8_t *model_start, uint8_t default_probability_cutoff,
                                  size_t sliding_window_average_size, const std::string &wake_word,
                                  size_t tensor_arena_size, const std::vector<std::string> &trained_languages) {
  const uint8_t *previous_model_start = this->model_start_;
  const uint8_t previous_default_probability_cutoff = this->default_probability_cutoff_;
  const uint8_t previous_probability_cutoff = this->probability_cutoff_;
  const size_t previous_sliding_window_size = this->sliding_window_size_;
  const std::string previous_wake_word = this->wake_word_;
  const size_t previous_tensor_arena_size = this->tensor_arena_size_;
  const std::vector<std::string> previous_trained_languages = this->trained_languages_;
  const bool previous_tensor_arena_size_probed = this->tensor_arena_size_probed_;

  this->unload_model();
  this->model_start_ = model_start;
  this->default_probability_cutoff_ = default_probability_cutoff;
  this->probability_cutoff_ = default_probability_cutoff;
  this->sliding_window_size_ = sliding_window_average_size;
  this->recent_streaming_probabilities_.assign(sliding_window_average_size, 0);
  this->wake_word_ = wake_word;
  this->tensor_arena_size_ = tensor_arena_size;
  this->trained_languages_ = trained_languages;
  this->tensor_arena_size_probed_ = false;
  this->current_stride_step_ = 0;

  if (!this->load_model_()) {
    this->unload_model();
    this->model_start_ = previous_model_start;
    this->default_probability_cutoff_ = previous_default_probability_cutoff;
    this->probability_cutoff_ = previous_probability_cutoff;
    this->sliding_window_size_ = previous_sliding_window_size;
    this->recent_streaming_probabilities_.assign(previous_sliding_window_size, 0);
    this->wake_word_ = previous_wake_word;
    this->tensor_arena_size_ = previous_tensor_arena_size;
    this->trained_languages_ = previous_trained_languages;
    this->tensor_arena_size_probed_ = previous_tensor_arena_size_probed;
    this->current_stride_step_ = 0;
    return false;
  }

  this->unload_model();
  return true;
}

void WakeWordModel::restore_compiled_model() {
  this->unload_model();
  this->model_start_ = this->compiled_model_start_;
  this->default_probability_cutoff_ = this->compiled_default_probability_cutoff_;
  this->probability_cutoff_ = this->compiled_default_probability_cutoff_;
  this->sliding_window_size_ = this->compiled_sliding_window_size_;
  this->recent_streaming_probabilities_.assign(this->compiled_sliding_window_size_, 0);
  this->wake_word_ = this->compiled_wake_word_;
  this->tensor_arena_size_ = this->compiled_tensor_arena_size_;
  this->trained_languages_ = this->compiled_trained_languages_;
  this->tensor_arena_size_probed_ = false;
  this->current_stride_step_ = 0;
}

void WakeWordModel::enable() {
  this->enabled_.store(true);
  if (!this->internal_only_) {
    bool enabled = true;
    this->pref_.save(&enabled);
  }
}

void WakeWordModel::disable() {
  this->enabled_.store(false);
  if (!this->internal_only_) {
    bool enabled = false;
    this->pref_.save(&enabled);
  }
}

DetectionEvent WakeWordModel::determine_detected() {
  DetectionEvent detection_event;
  detection_event.wake_word = &this->wake_word_;
  detection_event.max_probability = 0;
  detection_event.average_probability = 0;

  if ((this->ignore_windows_ < 0) || !this->enabled_.load()) {
    detection_event.detected = false;
    return detection_event;
  }

  uint32_t sum = 0;
  uint8_t active_window_count = 0;
  for (auto &prob : this->recent_streaming_probabilities_) {
    detection_event.max_probability = std::max(detection_event.max_probability, prob);
    if (prob >= this->probability_cutoff_) {
      active_window_count++;
    }
    sum += prob;
  }

  const DetectionProfile profile = this->get_detection_profile();
  detection_event.average_probability = sum / this->sliding_window_size_;
  detection_event.probability_cutoff = this->probability_cutoff_;
  detection_event.peak_probability_cutoff = this->effective_peak_probability_cutoff_(profile);
  detection_event.active_window_count = active_window_count;
  detection_event.min_active_windows = this->minimum_active_windows_(profile);
  detection_event.rise_score = this->rise_score_();
  detection_event.detection_profile = profile;
  this->fill_probability_history_(detection_event);

  const bool average_detected = sum > this->probability_cutoff_ * this->sliding_window_size_;
  const bool peak_detected = detection_event.max_probability >= detection_event.peak_probability_cutoff;
  const bool active_windows_detected = active_window_count >= detection_event.min_active_windows;
  const bool shape_detected = detection_event.rise_score >= this->minimum_rise_score_(profile);
  detection_event.detected = average_detected && peak_detected && active_windows_detected && shape_detected;

  this->unprocessed_probability_status_ = false;
  return detection_event;
}

VADModel::VADModel(const uint8_t *model_start, uint8_t default_probability_cutoff, size_t sliding_window_size,
                   size_t tensor_arena_size) {
  this->model_start_ = model_start;
  this->default_probability_cutoff_ = default_probability_cutoff;
  this->probability_cutoff_ = default_probability_cutoff;
  this->sliding_window_size_ = sliding_window_size;
  this->recent_streaming_probabilities_.resize(sliding_window_size, 0);
  this->tensor_arena_size_ = tensor_arena_size;
  this->register_streaming_ops_(this->streaming_op_resolver_);
}

DetectionEvent VADModel::determine_detected() {
  DetectionEvent detection_event;
  detection_event.max_probability = 0;
  detection_event.average_probability = 0;

  if (!this->enabled_.load()) {
    // We disabled the VAD model for some reason... so we shouldn't block wake words from being detected
    detection_event.detected = true;
    return detection_event;
  }

  uint32_t sum = 0;
  for (auto &prob : this->recent_streaming_probabilities_) {
    detection_event.max_probability = std::max(detection_event.max_probability, prob);
    sum += prob;
  }

  detection_event.average_probability = sum / this->sliding_window_size_;
  detection_event.probability_cutoff = this->probability_cutoff_;
  detection_event.detected = sum > (this->probability_cutoff_ * this->sliding_window_size_);

  return detection_event;
}

bool StreamingModel::register_streaming_ops_(tflite::MicroMutableOpResolver<20> &op_resolver) {
  if (op_resolver.AddCallOnce() != kTfLiteOk)
    return false;
  if (op_resolver.AddVarHandle() != kTfLiteOk)
    return false;
  if (op_resolver.AddReshape() != kTfLiteOk)
    return false;
  if (op_resolver.AddReadVariable() != kTfLiteOk)
    return false;
  if (op_resolver.AddStridedSlice() != kTfLiteOk)
    return false;
  if (op_resolver.AddConcatenation() != kTfLiteOk)
    return false;
  if (op_resolver.AddAssignVariable() != kTfLiteOk)
    return false;
  if (op_resolver.AddConv2D() != kTfLiteOk)
    return false;
  if (op_resolver.AddMul() != kTfLiteOk)
    return false;
  if (op_resolver.AddAdd() != kTfLiteOk)
    return false;
  if (op_resolver.AddMean() != kTfLiteOk)
    return false;
  if (op_resolver.AddFullyConnected() != kTfLiteOk)
    return false;
  if (op_resolver.AddLogistic() != kTfLiteOk)
    return false;
  if (op_resolver.AddQuantize() != kTfLiteOk)
    return false;
  if (op_resolver.AddDepthwiseConv2D() != kTfLiteOk)
    return false;
  if (op_resolver.AddAveragePool2D() != kTfLiteOk)
    return false;
  if (op_resolver.AddMaxPool2D() != kTfLiteOk)
    return false;
  if (op_resolver.AddPad() != kTfLiteOk)
    return false;
  if (op_resolver.AddPack() != kTfLiteOk)
    return false;
  if (op_resolver.AddSplitV() != kTfLiteOk)
    return false;

  return true;
}

}  // namespace micro_wake_word
}  // namespace esphome

#endif
