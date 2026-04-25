#include "micro_wake_word.h"

#ifdef USE_ESP32

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/network/util.h"

#ifdef USE_OTA
#include "esphome/components/ota/ota_backend.h"
#endif

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

#include <esp_http_client.h>

#include <algorithm>

namespace esphome {
namespace micro_wake_word {

static const char *const TAG = "micro_wake_word";

static const ssize_t DETECTION_QUEUE_LENGTH = 5;

static const size_t DATA_TIMEOUT_MS = 50;

static const uint32_t RING_BUFFER_DURATION_MS = 120;
static const uint32_t CAPTURE_RING_BUFFER_DURATION_MS = 2000;

static const uint32_t INFERENCE_TASK_STACK_SIZE = 3072;
static const UBaseType_t INFERENCE_TASK_PRIORITY = 3;
static const uint32_t CAPTURE_UPLOAD_TASK_STACK_SIZE = 8192;
static const UBaseType_t CAPTURE_UPLOAD_TASK_PRIORITY = 2;
static const uint32_t CAPTURE_UPLOAD_TIMEOUT_MS = 8000;
static const uint32_t CLOSE_MISS_UPLOAD_COOLDOWN_MS = 10000;
static const uint32_t CLOSE_MISS_CONFIRMATION_DELAY_MS = 900;
static const size_t CAPTURE_UPLOAD_BUFFER_SIZE = 2048;

static const char *detection_event_type_to_header(DetectionEventType event_type) {
  switch (event_type) {
    case DetectionEventType::WAKE_DETECTED:
      return "wake_detected";
    case DetectionEventType::CLOSE_MISS:
      return "close_miss";
    case DetectionEventType::BLOCKED_BY_VAD:
      return "blocked_by_vad";
    case DetectionEventType::NONE:
    default:
      return "";
  }
}

enum EventGroupBits : uint32_t {
  COMMAND_STOP = (1 << 0),  // Signals the inference task should stop

  TASK_STARTING = (1 << 3),
  TASK_RUNNING = (1 << 4),
  TASK_STOPPING = (1 << 5),
  TASK_STOPPED = (1 << 6),

  ERROR_MEMORY = (1 << 9),
  ERROR_INFERENCE = (1 << 10),

  WARNING_FULL_RING_BUFFER = (1 << 13),

  ERROR_BITS = ERROR_MEMORY | ERROR_INFERENCE,
  ALL_BITS = 0xfffff,  // 24 total bits available in an event group
};

float MicroWakeWord::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }

static const LogString *micro_wake_word_state_to_string(State state) {
  switch (state) {
    case State::STARTING:
      return LOG_STR("STARTING");
    case State::DETECTING_WAKE_WORD:
      return LOG_STR("DETECTING_WAKE_WORD");
    case State::STOPPING:
      return LOG_STR("STOPPING");
    case State::STOPPED:
      return LOG_STR("STOPPED");
    default:
      return LOG_STR("UNKNOWN");
  }
}

void MicroWakeWord::dump_config() {
  ESP_LOGCONFIG(TAG, "microWakeWord:");
  ESP_LOGCONFIG(TAG, "  models:");
  for (auto &model : this->wake_word_models_) {
    model->log_model_config();
  }
  ESP_LOGCONFIG(TAG, "  captured wake audio uploads: %s", YESNO(this->capture_upload_enabled_.load()));
  ESP_LOGCONFIG(TAG, "  captured close-miss uploads: %s", YESNO(this->capture_close_misses_enabled_.load()));
  ESP_LOGCONFIG(TAG, "  close-miss threshold: %.2f", this->get_capture_close_miss_probability_cutoff());
  const std::string capture_upload_url = this->build_capture_upload_url_();
  ESP_LOGCONFIG(TAG, "  trainer capture endpoint: %s",
                capture_upload_url.empty() ? "not configured" : capture_upload_url.c_str());
#ifdef USE_MICRO_WAKE_WORD_VAD
  this->vad_model_->log_model_config();
#endif
}

void MicroWakeWord::setup() {
  this->frontend_config_.window.size_ms = FEATURE_DURATION_MS;
  this->frontend_config_.window.step_size_ms = this->features_step_size_;
  this->frontend_config_.filterbank.num_channels = PREPROCESSOR_FEATURE_SIZE;
  this->frontend_config_.filterbank.lower_band_limit = FILTERBANK_LOWER_BAND_LIMIT;
  this->frontend_config_.filterbank.upper_band_limit = FILTERBANK_UPPER_BAND_LIMIT;
  this->frontend_config_.noise_reduction.smoothing_bits = NOISE_REDUCTION_SMOOTHING_BITS;
  this->frontend_config_.noise_reduction.even_smoothing = NOISE_REDUCTION_EVEN_SMOOTHING;
  this->frontend_config_.noise_reduction.odd_smoothing = NOISE_REDUCTION_ODD_SMOOTHING;
  this->frontend_config_.noise_reduction.min_signal_remaining = NOISE_REDUCTION_MIN_SIGNAL_REMAINING;
  this->frontend_config_.pcan_gain_control.enable_pcan = PCAN_GAIN_CONTROL_ENABLE_PCAN;
  this->frontend_config_.pcan_gain_control.strength = PCAN_GAIN_CONTROL_STRENGTH;
  this->frontend_config_.pcan_gain_control.offset = PCAN_GAIN_CONTROL_OFFSET;
  this->frontend_config_.pcan_gain_control.gain_bits = PCAN_GAIN_CONTROL_GAIN_BITS;
  this->frontend_config_.log_scale.enable_log = LOG_SCALE_ENABLE_LOG;
  this->frontend_config_.log_scale.scale_shift = LOG_SCALE_SCALE_SHIFT;

  this->event_group_ = xEventGroupCreate();
  if (this->event_group_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    this->mark_failed();
    return;
  }

  this->detection_queue_ = xQueueCreate(DETECTION_QUEUE_LENGTH, sizeof(DetectionEvent));
  if (this->detection_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create detection event queue");
    this->mark_failed();
    return;
  }

  this->microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
    if (this->state_ == State::STOPPED) {
      return;
    }
    std::shared_ptr<RingBuffer> temp_ring_buffer = this->ring_buffer_.lock();
    if (this->ring_buffer_.use_count() > 1) {
      size_t bytes_free = temp_ring_buffer->free();

      if (bytes_free < data.size()) {
        xEventGroupSetBits(this->event_group_, EventGroupBits::WARNING_FULL_RING_BUFFER);
        temp_ring_buffer->reset();
      }
      temp_ring_buffer->write((void *) data.data(), data.size());
    }

    std::shared_ptr<RingBuffer> temp_capture_ring_buffer = this->capture_ring_buffer_;
    // Keep the rolling capture buffer warm whenever detection is running so the first
    // wake after enabling uploads or reconnecting still has pre-roll audio available.
    if (temp_capture_ring_buffer != nullptr) {
      temp_capture_ring_buffer->write((void *) data.data(), data.size());
    }
  });

#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif
}

#ifdef USE_OTA_STATE_LISTENER
void MicroWakeWord::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED) {
    this->suspend_task_();
  } else if (state == ota::OTA_ERROR) {
    this->resume_task_();
  }
}
#endif

void MicroWakeWord::inference_task(void *params) {
  MicroWakeWord *this_mww = (MicroWakeWord *) params;

  xEventGroupSetBits(this_mww->event_group_, EventGroupBits::TASK_STARTING);

  {  // Ensures any C++ objects fall out of scope to deallocate before deleting the task

    const size_t new_bytes_to_process =
        this_mww->microphone_source_->get_audio_stream_info().ms_to_bytes(this_mww->features_step_size_);
    std::unique_ptr<audio::AudioSourceTransferBuffer> audio_buffer;
    int8_t features_buffer[PREPROCESSOR_FEATURE_SIZE];

    if (!(xEventGroupGetBits(this_mww->event_group_) & ERROR_BITS)) {
      // Allocate audio transfer buffer
      audio_buffer = audio::AudioSourceTransferBuffer::create(new_bytes_to_process);

      if (audio_buffer == nullptr) {
        xEventGroupSetBits(this_mww->event_group_, EventGroupBits::ERROR_MEMORY);
      }
    }

    if (!(xEventGroupGetBits(this_mww->event_group_) & ERROR_BITS)) {
      // Allocate ring buffer
      std::shared_ptr<RingBuffer> temp_ring_buffer = RingBuffer::create(
          this_mww->microphone_source_->get_audio_stream_info().ms_to_bytes(RING_BUFFER_DURATION_MS));
      if (temp_ring_buffer.use_count() == 0) {
        xEventGroupSetBits(this_mww->event_group_, EventGroupBits::ERROR_MEMORY);
      }
      audio_buffer->set_source(temp_ring_buffer);
      this_mww->ring_buffer_ = temp_ring_buffer;

      std::shared_ptr<RingBuffer> temp_capture_ring_buffer = RingBuffer::create(
          this_mww->microphone_source_->get_audio_stream_info().ms_to_bytes(CAPTURE_RING_BUFFER_DURATION_MS));
      if (temp_capture_ring_buffer.use_count() == 0) {
        ESP_LOGW(TAG, "Failed to allocate captured audio ring buffer; wake audio uploads will be unavailable.");
      } else {
        this_mww->capture_ring_buffer_ = temp_capture_ring_buffer;
      }
    }

    if (!(xEventGroupGetBits(this_mww->event_group_) & ERROR_BITS)) {
      this_mww->microphone_source_->start();
      xEventGroupSetBits(this_mww->event_group_, EventGroupBits::TASK_RUNNING);

      while (!(xEventGroupGetBits(this_mww->event_group_) & COMMAND_STOP)) {
        audio_buffer->transfer_data_from_source(pdMS_TO_TICKS(DATA_TIMEOUT_MS));

        if (audio_buffer->available() < new_bytes_to_process) {
          // Insufficient data to generate new spectrogram features, read more next iteration
          continue;
        }

        // Generate new spectrogram features
        uint32_t processed_samples = this_mww->generate_features_(
            (int16_t *) audio_buffer->get_buffer_start(), audio_buffer->available() / sizeof(int16_t), features_buffer);
        audio_buffer->decrease_buffer_length(processed_samples * sizeof(int16_t));

        // Run inference using the new spectorgram features
        if (!this_mww->update_model_probabilities_(features_buffer)) {
          xEventGroupSetBits(this_mww->event_group_, EventGroupBits::ERROR_INFERENCE);
          break;
        }

        // Process each model's probabilities and possibly send a Detection Event to the queue
        this_mww->process_probabilities_();
      }
    }
  }

  xEventGroupSetBits(this_mww->event_group_, EventGroupBits::TASK_STOPPING);

  this_mww->unload_models_();
  this_mww->microphone_source_->stop();
  this_mww->capture_ring_buffer_.reset();
  FrontendFreeStateContents(&this_mww->frontend_state_);

  xEventGroupSetBits(this_mww->event_group_, EventGroupBits::TASK_STOPPED);
  while (true) {
    // Continuously delay until the main loop deletes the task
    delay(10);
  }
}

std::vector<WakeWordModel *> MicroWakeWord::get_wake_words() {
  std::vector<WakeWordModel *> external_wake_word_models;
  for (auto *model : this->wake_word_models_) {
    if (!model->get_internal_only()) {
      external_wake_word_models.push_back(model);
    }
  }
  return external_wake_word_models;
}

void MicroWakeWord::add_wake_word_model(WakeWordModel *model) { this->wake_word_models_.push_back(model); }

#ifdef USE_MICRO_WAKE_WORD_VAD
void MicroWakeWord::add_vad_model(const uint8_t *model_start, uint8_t probability_cutoff, size_t sliding_window_size,
                                  size_t tensor_arena_size) {
  this->vad_model_ = make_unique<VADModel>(model_start, probability_cutoff, sliding_window_size, tensor_arena_size);
}
#endif

void MicroWakeWord::suspend_task_() {
  if (this->inference_task_handle_ != nullptr) {
    vTaskSuspend(this->inference_task_handle_);
  }
}

void MicroWakeWord::resume_task_() {
  if (this->inference_task_handle_ != nullptr) {
    vTaskResume(this->inference_task_handle_);
  }
}

void MicroWakeWord::loop() {
  uint32_t event_group_bits = xEventGroupGetBits(this->event_group_);

  if (event_group_bits & EventGroupBits::ERROR_MEMORY) {
    xEventGroupClearBits(this->event_group_, EventGroupBits::ERROR_MEMORY);
    ESP_LOGE(TAG, "Encountered an error allocating buffers");
  }

  if (event_group_bits & EventGroupBits::ERROR_INFERENCE) {
    xEventGroupClearBits(this->event_group_, EventGroupBits::ERROR_INFERENCE);
    ESP_LOGE(TAG, "Encountered an error while performing an inference");
  }

  if (event_group_bits & EventGroupBits::WARNING_FULL_RING_BUFFER) {
    xEventGroupClearBits(this->event_group_, EventGroupBits::WARNING_FULL_RING_BUFFER);
    ESP_LOGW(TAG, "Not enough free bytes in ring buffer to store incoming audio data. Resetting the ring buffer. Wake "
                  "word detection accuracy will temporarily be reduced.");
  }

  if (event_group_bits & EventGroupBits::TASK_STARTING) {
    ESP_LOGD(TAG, "Inference task has started, attempting to allocate memory for buffers");
    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_STARTING);
  }

  if (event_group_bits & EventGroupBits::TASK_RUNNING) {
    ESP_LOGD(TAG, "Inference task is running");

    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_RUNNING);
    this->set_state_(State::DETECTING_WAKE_WORD);
  }

  if (event_group_bits & EventGroupBits::TASK_STOPPING) {
    ESP_LOGD(TAG, "Inference task is stopping, deallocating buffers");
    xEventGroupClearBits(this->event_group_, EventGroupBits::TASK_STOPPING);
  }

  if ((event_group_bits & EventGroupBits::TASK_STOPPED)) {
    ESP_LOGD(TAG, "Inference task is finished, freeing task resources");
    vTaskDelete(this->inference_task_handle_);
    this->inference_task_handle_ = nullptr;
    xEventGroupClearBits(this->event_group_, ALL_BITS);
    xQueueReset(this->detection_queue_);
    this->set_state_(State::STOPPED);
  }

  if ((this->pending_start_) && (this->state_ == State::STOPPED)) {
    this->set_state_(State::STARTING);
    this->pending_start_ = false;
  }

  if ((this->pending_stop_) && (this->state_ == State::DETECTING_WAKE_WORD)) {
    this->set_state_(State::STOPPING);
    this->pending_stop_ = false;
  }

  switch (this->state_) {
    case State::STARTING:
      if ((this->inference_task_handle_ == nullptr) && !this->status_has_error()) {
        // Setup preprocesor feature generator. If done in the task, it would lock the task to its initial core, as it
        // uses floating point operations.
        if (!FrontendPopulateState(&this->frontend_config_, &this->frontend_state_,
                                   this->microphone_source_->get_audio_stream_info().get_sample_rate())) {
          this->status_momentary_error("frontend_alloc", 1000);
          return;
        }

        xTaskCreate(MicroWakeWord::inference_task, "mww", INFERENCE_TASK_STACK_SIZE, (void *) this,
                    INFERENCE_TASK_PRIORITY, &this->inference_task_handle_);

        if (this->inference_task_handle_ == nullptr) {
          FrontendFreeStateContents(&this->frontend_state_);  // Deallocate frontend state
          this->status_momentary_error("task_start", 1000);
        }
      }
      break;
    case State::DETECTING_WAKE_WORD: {
      DetectionEvent detection_event;
      while (xQueueReceive(this->detection_queue_, &detection_event, 0)) {
        constexpr float uint8_to_float_divisor =
            255.0f;  // Converting a quantized uint8 probability to floating point
        if (detection_event.blocked_by_vad) {
          ESP_LOGD(TAG, "Wake word model predicts '%s', but VAD model doesn't.", detection_event.wake_word->c_str());
          this->queue_detection_capture_(detection_event, detection_event.event_type);
        } else if (detection_event.partially_detection) {
          ESP_LOGD(TAG, "Close miss for '%s' with sliding average probability %.2f and max probability %.2f",
                   detection_event.wake_word->c_str(), (detection_event.average_probability / uint8_to_float_divisor),
                   (detection_event.max_probability / uint8_to_float_divisor));
          this->queue_detection_capture_(detection_event, detection_event.event_type);
        } else {
          ESP_LOGD(TAG, "Detected '%s' with sliding average probability is %.2f and max probability is %.2f",
                   detection_event.wake_word->c_str(), (detection_event.average_probability / uint8_to_float_divisor),
                   (detection_event.max_probability / uint8_to_float_divisor));
          this->queue_detection_capture_(detection_event, DetectionEventType::WAKE_DETECTED);
          this->wake_word_detected_trigger_.trigger(*detection_event.wake_word);
          if (this->stop_after_detection_) {
            this->stop();
          }
        }
      }
      break;
    }
    case State::STOPPING:
      xEventGroupSetBits(this->event_group_, EventGroupBits::COMMAND_STOP);
      break;
    case State::STOPPED:
      break;
  }
}

void MicroWakeWord::start() {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Wake word detection can't start as the component hasn't been setup yet");
    return;
  }

  if (this->is_failed()) {
    ESP_LOGW(TAG, "Wake word component is marked as failed. Please check setup logs");
    return;
  }

  if (this->is_running()) {
    ESP_LOGW(TAG, "Wake word detection is already running");
    return;
  }

  ESP_LOGD(TAG, "Starting wake word detection");

  this->pending_start_ = true;
  this->pending_stop_ = false;
}

void MicroWakeWord::stop() {
  if (this->state_ == STOPPED)
    return;

  ESP_LOGD(TAG, "Stopping wake word detection");

  this->pending_start_ = false;
  this->pending_stop_ = true;
}

void MicroWakeWord::set_state_(State state) {
  if (this->state_ != state) {
    ESP_LOGD(TAG, "State changed from %s to %s", LOG_STR_ARG(micro_wake_word_state_to_string(this->state_)),
             LOG_STR_ARG(micro_wake_word_state_to_string(state)));
    this->state_ = state;
  }
}

size_t MicroWakeWord::generate_features_(int16_t *audio_buffer, size_t samples_available,
                                         int8_t features_buffer[PREPROCESSOR_FEATURE_SIZE]) {
  size_t processed_samples = 0;
  struct FrontendOutput frontend_output =
      FrontendProcessSamples(&this->frontend_state_, audio_buffer, samples_available, &processed_samples);

  for (size_t i = 0; i < frontend_output.size; ++i) {
    // These scaling values are set to match the TFLite audio frontend int8 output.
    // The feature pipeline outputs 16-bit signed integers in roughly a 0 to 670
    // range. In training, these are then arbitrarily divided by 25.6 to get
    // float values in the rough range of 0.0 to 26.0. This scaling is performed
    // for historical reasons, to match up with the output of other feature
    // generators.
    // The process is then further complicated when we quantize the model. This
    // means we have to scale the 0.0 to 26.0 real values to the -128 (INT8_MIN)
    // to 127 (INT8_MAX) signed integer numbers.
    // All this means that to get matching values from our integer feature
    // output into the tensor input, we have to perform:
    // input = (((feature / 25.6) / 26.0) * 256) - 128
    // To simplify this and perform it in 32-bit integer math, we rearrange to:
    // input = (feature * 256) / (25.6 * 26.0) - 128
    constexpr int32_t value_scale = 256;
    constexpr int32_t value_div = 666;  // 666 = 25.6 * 26.0 after rounding
    int32_t value = ((frontend_output.values[i] * value_scale) + (value_div / 2)) / value_div;

    value += INT8_MIN;  // Adds a -128; i.e., subtracts 128
    features_buffer[i] = static_cast<int8_t>(clamp<int32_t>(value, INT8_MIN, INT8_MAX));
  }

  return processed_samples;
}

void MicroWakeWord::process_probabilities_() {
#ifdef USE_MICRO_WAKE_WORD_VAD
  DetectionEvent vad_state = this->vad_model_->determine_detected();

  this->vad_state_ = vad_state.detected;  // atomic write, so thread safe
#endif

  for (auto &model : this->wake_word_models_) {
    if (model->get_unprocessed_probability_status()) {
      // Only detect wake words if there is a new probability since the last check
      DetectionEvent wake_word_state = model->determine_detected();
      if (wake_word_state.detected) {
#ifdef USE_MICRO_WAKE_WORD_VAD
        if (vad_state.detected) {
#endif
          xQueueSend(this->detection_queue_, &wake_word_state, portMAX_DELAY);

          // Wake main loop immediately to process wake word detection
          this->clear_pending_close_miss_(wake_word_state.wake_word);
          App.wake_loop_threadsafe();

          model->reset_probabilities();
#ifdef USE_MICRO_WAKE_WORD_VAD
        } else {
          wake_word_state.blocked_by_vad = true;
          if (this->should_capture_close_miss_(wake_word_state)) {
            this->clear_pending_close_miss_(wake_word_state.wake_word);
            wake_word_state.event_type = DetectionEventType::BLOCKED_BY_VAD;
            this->note_close_miss_upload_();
            xQueueSend(this->detection_queue_, &wake_word_state, portMAX_DELAY);
            App.wake_loop_threadsafe();
          } else {
            xQueueSend(this->detection_queue_, &wake_word_state, portMAX_DELAY);
          }
        }
#endif
      } else if (this->should_capture_close_miss_(wake_word_state)) {
        this->queue_pending_close_miss_(wake_word_state);
      }
    }
  }

  this->flush_pending_close_miss_();
}

void MicroWakeWord::unload_models_() {
  for (auto &model : this->wake_word_models_) {
    model->unload_model();
  }
#ifdef USE_MICRO_WAKE_WORD_VAD
  this->vad_model_->unload_model();
#endif
}

bool MicroWakeWord::update_model_probabilities_(const int8_t audio_features[PREPROCESSOR_FEATURE_SIZE]) {
  bool success = true;

  for (auto &model : this->wake_word_models_) {
    // Perform inference
    success = success & model->perform_streaming_inference(audio_features);
  }
#ifdef USE_MICRO_WAKE_WORD_VAD
  success = success & this->vad_model_->perform_streaming_inference(audio_features);
#endif

  return success;
}

bool MicroWakeWord::capture_feature_enabled_() const {
  return this->capture_upload_enabled_.load() || this->capture_close_misses_enabled_.load();
}

bool MicroWakeWord::should_capture_close_miss_(const DetectionEvent &detection_event) {
  if (!this->capture_close_misses_enabled_.load()) {
    return false;
  }

  if (detection_event.average_probability < this->capture_close_miss_probability_cutoff_.load()) {
    return false;
  }

  const uint32_t now = millis();
  if ((this->last_close_miss_upload_ms_ != 0) &&
      ((now - this->last_close_miss_upload_ms_) < CLOSE_MISS_UPLOAD_COOLDOWN_MS)) {
    return false;
  }

  return true;
}

void MicroWakeWord::note_close_miss_upload_() {
  this->last_close_miss_upload_ms_ = millis();
}

void MicroWakeWord::queue_pending_close_miss_(const DetectionEvent &detection_event) {
  DetectionEvent pending_event = detection_event;
  pending_event.partially_detection = true;
  pending_event.event_type = DetectionEventType::CLOSE_MISS;

  this->pending_close_miss_event_ = pending_event;
  this->pending_close_miss_ = true;
  this->pending_close_miss_due_ms_ = millis() + CLOSE_MISS_CONFIRMATION_DELAY_MS;
}

void MicroWakeWord::clear_pending_close_miss_(const std::string *wake_word) {
  if (!this->pending_close_miss_) {
    return;
  }
  if ((wake_word != nullptr) && (this->pending_close_miss_event_.wake_word != wake_word)) {
    return;
  }

  this->pending_close_miss_ = false;
  this->pending_close_miss_due_ms_ = 0;
  this->pending_close_miss_event_ = DetectionEvent();
}

void MicroWakeWord::flush_pending_close_miss_() {
  if (!this->pending_close_miss_) {
    return;
  }
  if (static_cast<int32_t>(millis() - this->pending_close_miss_due_ms_) < 0) {
    return;
  }
  if (!this->should_capture_close_miss_(this->pending_close_miss_event_)) {
    this->clear_pending_close_miss_();
    return;
  }

  DetectionEvent event = this->pending_close_miss_event_;
  this->clear_pending_close_miss_();
  this->note_close_miss_upload_();
  xQueueSend(this->detection_queue_, &event, portMAX_DELAY);
  App.wake_loop_threadsafe();
}

std::string MicroWakeWord::build_capture_upload_url_() const {
  static const char *const RAW_CAPTURE_PATH = "/api/upload_captured_audio_raw";

  if (this->capture_upload_url_.empty()) {
    return "";
  }

  if (this->capture_upload_url_.find(RAW_CAPTURE_PATH) != std::string::npos) {
    return this->capture_upload_url_;
  }

  if (this->capture_upload_url_.back() == '/') {
    return this->capture_upload_url_ + "api/upload_captured_audio_raw";
  }

  return this->capture_upload_url_ + RAW_CAPTURE_PATH;
}

bool MicroWakeWord::snapshot_capture_audio_(std::vector<uint8_t> &audio_bytes) {
  std::shared_ptr<RingBuffer> temp_capture_ring_buffer = this->capture_ring_buffer_;
  if (temp_capture_ring_buffer == nullptr) {
    return false;
  }

  const size_t bytes_available = temp_capture_ring_buffer->available();
  if (bytes_available == 0) {
    return false;
  }

  audio_bytes.resize(bytes_available);
  const size_t bytes_read = temp_capture_ring_buffer->read(audio_bytes.data(), bytes_available, 0);
  if (bytes_read == 0) {
    audio_bytes.clear();
    return false;
  }

  if (bytes_read < bytes_available) {
    audio_bytes.resize(bytes_read);
  }

  return true;
}

void MicroWakeWord::queue_detection_capture_(const DetectionEvent &detection_event, DetectionEventType event_type) {
  const char *event_type_header = detection_event_type_to_header(event_type);
  if (event_type_header[0] == '\0') {
    return;
  }
  if ((event_type == DetectionEventType::WAKE_DETECTED) && !this->capture_upload_enabled_.load()) {
    return;
  }
  if ((event_type != DetectionEventType::WAKE_DETECTED) && !this->capture_close_misses_enabled_.load()) {
    return;
  }

  const std::string upload_url = this->build_capture_upload_url_();
  if (upload_url.empty()) {
    ESP_LOGW(TAG, "Captured wake audio upload skipped because no trainer capture URL is configured.");
    return;
  }

  if (this->capture_upload_in_progress_.exchange(true)) {
    ESP_LOGW(TAG, "Captured wake audio upload already in progress; skipping '%s'.",
             detection_event.wake_word == nullptr ? "unknown" : detection_event.wake_word->c_str());
    return;
  }

  std::vector<uint8_t> pcm_data;
  if (!this->snapshot_capture_audio_(pcm_data)) {
    this->capture_upload_in_progress_.store(false);
    ESP_LOGW(TAG,
             "Captured wake audio upload skipped because the wake-audio ring buffer was empty. "
             "This usually means detection started before enough audio was buffered.");
    return;
  }

  auto *request = new CaptureUploadRequest();
  request->parent = this;
  request->upload_url = upload_url;
  request->source_device = App.get_name().c_str();
  request->wake_word = detection_event.wake_word == nullptr ? "" : *detection_event.wake_word;
  request->pcm_data = std::move(pcm_data);
  request->max_probability = detection_event.max_probability;
  request->average_probability = detection_event.average_probability;
  request->blocked_by_vad = detection_event.blocked_by_vad;
  request->event_type = event_type_header;

  BaseType_t task_created = xTaskCreate(MicroWakeWord::capture_upload_task, "mww_capture_upload",
                                        CAPTURE_UPLOAD_TASK_STACK_SIZE, request, CAPTURE_UPLOAD_TASK_PRIORITY, nullptr);
  if (task_created != pdPASS) {
    this->capture_upload_in_progress_.store(false);
    delete request;
    ESP_LOGW(TAG, "Failed to start captured wake audio upload task.");
  }
}

void MicroWakeWord::capture_upload_task(void *params) {
  auto *request = static_cast<CaptureUploadRequest *>(params);
  if (request == nullptr) {
    vTaskDelete(nullptr);
    return;
  }

  request->parent->upload_capture_(*request);
  request->parent->capture_upload_in_progress_.store(false);
  delete request;

  vTaskDelete(nullptr);
}

bool MicroWakeWord::upload_capture_(const CaptureUploadRequest &request) {
  if (!network::is_connected()) {
    ESP_LOGW(TAG, "Captured wake audio upload skipped because the device is not connected to the network.");
    return false;
  }

  if (request.pcm_data.empty()) {
    ESP_LOGW(TAG, "Captured wake audio upload skipped because the buffered clip was empty.");
    return false;
  }

  esp_http_client_config_t config = {};
  config.url = request.upload_url.c_str();
  config.method = HTTP_METHOD_POST;
  config.timeout_ms = CAPTURE_UPLOAD_TIMEOUT_MS;
  config.buffer_size = CAPTURE_UPLOAD_BUFFER_SIZE;
  config.buffer_size_tx = CAPTURE_UPLOAD_BUFFER_SIZE;

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
  if (request.upload_url.find("https:") != std::string::npos) {
    config.crt_bundle_attach = esp_crt_bundle_attach;
  }
#endif

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    ESP_LOGW(TAG, "Captured wake audio upload failed because the HTTP client could not be initialized.");
    return false;
  }

  const std::string max_probability = std::to_string(request.max_probability);
  const std::string average_probability = std::to_string(request.average_probability);
  const std::string blocked_by_vad = request.blocked_by_vad ? "true" : "false";
  const std::string original_name = "wake_capture.raw";

  esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
  esp_http_client_set_header(client, "X-Audio-Format", "pcm_s16le");
  esp_http_client_set_header(client, "X-Original-Name", original_name.c_str());
  esp_http_client_set_header(client, "X-Source-Device", request.source_device.c_str());
  esp_http_client_set_header(client, "X-Wake-Word", request.wake_word.c_str());
  esp_http_client_set_header(client, "X-Event-Type", request.event_type.c_str());
  esp_http_client_set_header(client, "X-Blocked-By-Vad", blocked_by_vad.c_str());
  esp_http_client_set_header(client, "X-Max-Probability", max_probability.c_str());
  esp_http_client_set_header(client, "X-Average-Probability", average_probability.c_str());

  esp_err_t err = esp_http_client_open(client, request.pcm_data.size());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Captured wake audio upload failed while opening HTTP connection: %s", esp_err_to_name(err));
    esp_http_client_cleanup(client);
    return false;
  }

  size_t bytes_written = 0;
  const char *payload = reinterpret_cast<const char *>(request.pcm_data.data());
  while (bytes_written < request.pcm_data.size()) {
    const size_t remaining = request.pcm_data.size() - bytes_written;
    const size_t chunk_size = std::min(remaining, CAPTURE_UPLOAD_BUFFER_SIZE);
    const int written = esp_http_client_write(client, payload + bytes_written, chunk_size);
    if (written <= 0) {
      ESP_LOGW(TAG, "Captured wake audio upload failed while streaming request body.");
      esp_http_client_close(client);
      esp_http_client_cleanup(client);
      return false;
    }
    bytes_written += written;
    delay(0);
  }

  (void) esp_http_client_fetch_headers(client);
  const int status_code = esp_http_client_get_status_code(client);

  esp_http_client_close(client);
  esp_http_client_cleanup(client);

  if ((status_code < 200) || (status_code >= 300)) {
    ESP_LOGW(TAG, "Captured wake audio upload failed with HTTP status %d.", status_code);
    return false;
  }

  ESP_LOGI(TAG, "Uploaded captured wake audio for '%s' (%u bytes) to trainer.", request.wake_word.c_str(),
           static_cast<unsigned int>(request.pcm_data.size()));
  return true;
}

}  // namespace micro_wake_word
}  // namespace esphome

#endif  // USE_ESP32
