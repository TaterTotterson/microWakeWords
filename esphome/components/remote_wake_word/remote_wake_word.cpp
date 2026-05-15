#include "remote_wake_word.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

#include <esp_http_client.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <cstdlib>

namespace esphome {
namespace remote_wake_word {

static const char *const TAG = "remote_wake_word";
static const size_t DATA_TIMEOUT_MS = 50;
static const uint32_t RING_BUFFER_DURATION_MS = 2000;
static const size_t HTTP_BUFFER_SIZE = 2048;
static const uint32_t TASK_STACK_SIZE = 12288;
static const UBaseType_t TASK_PRIORITY = 2;
static const ssize_t EVENT_QUEUE_LENGTH = 4;

static const LogString *remote_wake_word_state_to_string(State state) {
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

static std::string trim_copy(const std::string &value) {
  size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
    start++;
  }
  size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    end--;
  }
  return value.substr(start, end - start);
}

static bool starts_with_http(const std::string &value) {
  return value.rfind("http://", 0) == 0 || value.rfind("https://", 0) == 0;
}

static bool response_has_true_field(const std::string &body, const std::string &field) {
  const std::string key = "\"" + field + "\"";
  size_t pos = body.find(key);
  if (pos == std::string::npos) {
    pos = body.find(field);
  }
  if (pos == std::string::npos) {
    return false;
  }
  const size_t true_pos = body.find("true", pos);
  const size_t false_pos = body.find("false", pos);
  return true_pos != std::string::npos && (false_pos == std::string::npos || true_pos < false_pos);
}

static std::string json_string_field(const std::string &body, const std::string &field) {
  const std::string key = "\"" + field + "\"";
  size_t pos = body.find(key);
  if (pos == std::string::npos) {
    return "";
  }
  pos = body.find(':', pos + key.size());
  if (pos == std::string::npos) {
    return "";
  }
  pos = body.find('"', pos + 1);
  if (pos == std::string::npos) {
    return "";
  }
  size_t end = pos + 1;
  while (end < body.size()) {
    if (body[end] == '"' && body[end - 1] != '\\') {
      return body.substr(pos + 1, end - pos - 1);
    }
    end++;
  }
  return "";
}

static float json_float_field(const std::string &body, const std::string &field, float fallback) {
  const std::string key = "\"" + field + "\"";
  size_t pos = body.find(key);
  if (pos == std::string::npos) {
    return fallback;
  }
  pos = body.find(':', pos + key.size());
  if (pos == std::string::npos) {
    return fallback;
  }
  pos++;
  while (pos < body.size() && std::isspace(static_cast<unsigned char>(body[pos]))) {
    pos++;
  }
  char *end = nullptr;
  const float parsed = strtof(body.c_str() + pos, &end);
  if (end == body.c_str() + pos) {
    return fallback;
  }
  return parsed;
}

float RemoteWakeWord::get_setup_priority() const { return setup_priority::AFTER_CONNECTION; }

void RemoteWakeWord::dump_config() {
  const std::string endpoint = this->build_endpoint_url_();
  ESP_LOGCONFIG(TAG, "Remote Wake Word:");
  ESP_LOGCONFIG(TAG, "  endpoint: %s", endpoint.empty() ? "not configured" : endpoint.c_str());
  ESP_LOGCONFIG(TAG, "  wake word hint: %s", this->wake_word_.empty() ? "any" : this->wake_word_.c_str());
  ESP_LOGCONFIG(TAG, "  source device: %s", this->source_device_.empty() ? "not configured" : this->source_device_.c_str());
  ESP_LOGCONFIG(TAG, "  chunk duration: %ums", static_cast<unsigned int>(this->chunk_duration_ms_));
  ESP_LOGCONFIG(TAG, "  max failures before fallback: %u", static_cast<unsigned int>(this->max_failures_));
  ESP_LOGCONFIG(TAG, "  HTTP timeout: %ums", static_cast<unsigned int>(this->http_timeout_ms_));
}

void RemoteWakeWord::setup() {
  if (this->microphone_source_ == nullptr) {
    ESP_LOGE(TAG, "Remote wake word requires a microphone source.");
    this->mark_failed();
    return;
  }
  this->event_queue_ = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(RemoteWakeWordEvent *));
  if (this->event_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create remote wake word event queue.");
    this->mark_failed();
    return;
  }
  this->microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
    std::shared_ptr<RingBuffer> temp_ring_buffer = this->ring_buffer_.lock();
    if (this->ring_buffer_.use_count() > 1 && temp_ring_buffer != nullptr) {
      if (temp_ring_buffer->free() < data.size()) {
        temp_ring_buffer->reset();
      }
      temp_ring_buffer->write((void *) data.data(), data.size());
    }
  });
}

void RemoteWakeWord::start() {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "Remote wake word detection can't start because the component is not ready.");
    return;
  }
  if (this->is_failed()) {
    ESP_LOGW(TAG, "Remote wake word component is marked failed.");
    return;
  }
  if (this->is_running()) {
    ESP_LOGW(TAG, "Remote wake word detection is already running.");
    return;
  }
  const std::string endpoint = this->build_endpoint_url_();
  if (endpoint.empty()) {
    this->queue_event_(RemoteWakeWordEvent::ERROR, "", "Remote wake endpoint is not configured.", 0.0f);
    return;
  }
  if (!starts_with_http(endpoint)) {
    this->queue_event_(RemoteWakeWordEvent::ERROR, "", "Remote wake endpoint must be an HTTP URL.", 0.0f);
    return;
  }
  this->stop_requested_.store(false);
  this->task_started_.store(false);
  this->task_done_.store(false);
  this->consecutive_failures_.store(0);
  this->set_state_(State::STARTING);
  if (xTaskCreate(RemoteWakeWord::detection_task, "remote_wake_word", TASK_STACK_SIZE, (void *) this, TASK_PRIORITY,
                  &this->task_handle_) != pdPASS) {
    this->task_handle_ = nullptr;
    this->set_state_(State::STOPPED);
    this->queue_event_(RemoteWakeWordEvent::ERROR, "", "Failed to start remote wake task.", 0.0f);
  }
}

void RemoteWakeWord::stop() {
  if (this->state_ == State::STOPPED && this->task_handle_ == nullptr) {
    return;
  }
  ESP_LOGD(TAG, "Stopping remote wake word detection");
  this->stop_requested_.store(true);
  if (this->state_ != State::STOPPED) {
    this->set_state_(State::STOPPING);
  }
}

void RemoteWakeWord::loop() {
  if (this->task_started_.exchange(false)) {
    this->set_state_(State::DETECTING_WAKE_WORD);
  }

  RemoteWakeWordEvent *event = nullptr;
  while (this->event_queue_ != nullptr && xQueueReceive(this->event_queue_, &event, 0) == pdTRUE) {
    if (event == nullptr) {
      continue;
    }
    if (event->type == RemoteWakeWordEvent::DETECTION) {
      ESP_LOGI(TAG, "Remote wake word detected: %s (score %.3f)", event->wake_word.c_str(), event->score);
      this->wake_word_detected_trigger_.trigger(event->wake_word);
    } else {
      ESP_LOGW(TAG, "Remote wake word error: %s", event->message.c_str());
      this->error_trigger_.trigger(event->message);
    }
    delete event;
  }

  if (this->task_done_.exchange(false)) {
    this->task_handle_ = nullptr;
    this->ring_buffer_.reset();
    this->set_state_(State::STOPPED);
  }
}

void RemoteWakeWord::detection_task(void *params) {
  auto *parent = static_cast<RemoteWakeWord *>(params);
  if (parent == nullptr) {
    vTaskDelete(nullptr);
    return;
  }

  std::string detected_wake_word;
  float detected_score = 0.0f;
  std::string terminal_error;

  {
    const auto stream_info = parent->microphone_source_->get_audio_stream_info();
    const size_t bytes_to_process = stream_info.ms_to_bytes(parent->chunk_duration_ms_);
    if (bytes_to_process == 0) {
      terminal_error = "Remote wake chunk size is zero.";
    } else {
      auto audio_buffer = audio::AudioSourceTransferBuffer::create(bytes_to_process);
      std::shared_ptr<RingBuffer> temp_ring_buffer =
          RingBuffer::create(stream_info.ms_to_bytes(std::max(RING_BUFFER_DURATION_MS, parent->chunk_duration_ms_ * 4)));
      if (audio_buffer == nullptr || temp_ring_buffer.use_count() == 0) {
        terminal_error = "Remote wake failed to allocate audio buffers.";
      } else {
        audio_buffer->set_source(temp_ring_buffer);
        parent->ring_buffer_ = temp_ring_buffer;
        parent->microphone_source_->start();
        parent->task_started_.store(true);

        while (!parent->stop_requested_.load()) {
          audio_buffer->transfer_data_from_source(pdMS_TO_TICKS(DATA_TIMEOUT_MS));
          if (audio_buffer->available() < bytes_to_process) {
            continue;
          }

          std::vector<uint8_t> audio_bytes(bytes_to_process);
          std::memcpy(audio_bytes.data(), audio_buffer->get_buffer_start(), bytes_to_process);
          audio_buffer->decrease_buffer_length(bytes_to_process);

          RemoteWakeWordHttpResult result = parent->post_audio_chunk_(audio_bytes, stream_info);
          if (!result.ok) {
            const uint8_t failures = parent->consecutive_failures_.fetch_add(1) + 1;
            ESP_LOGW(TAG, "Remote wake request failed (%u/%u): %s", static_cast<unsigned int>(failures),
                     static_cast<unsigned int>(parent->max_failures_), result.error.c_str());
            if (failures >= parent->max_failures_) {
              terminal_error = result.error.empty() ? "Remote wake server is unavailable." : result.error;
              break;
            }
            delay(100);
            continue;
          }

          parent->consecutive_failures_.store(0);
          if (result.detected) {
            detected_wake_word = result.wake_word.empty() ? parent->wake_word_ : result.wake_word;
            if (detected_wake_word.empty()) {
              detected_wake_word = "openwakeword";
            }
            detected_score = result.score;
            parent->stop_requested_.store(true);
            break;
          }
        }
        parent->microphone_source_->stop();
      }
    }
  }

  parent->ring_buffer_.reset();
  if (!detected_wake_word.empty()) {
    parent->queue_event_(RemoteWakeWordEvent::DETECTION, detected_wake_word, "", detected_score);
  } else if (!terminal_error.empty() && !parent->stop_requested_.load()) {
    parent->queue_event_(RemoteWakeWordEvent::ERROR, "", terminal_error, 0.0f);
  }
  parent->task_done_.store(true);
  vTaskDelete(nullptr);
}

RemoteWakeWordHttpResult RemoteWakeWord::post_audio_chunk_(const std::vector<uint8_t> &audio_bytes,
                                                           const audio::AudioStreamInfo &stream_info) {
  RemoteWakeWordHttpResult result;
  if (!network::is_connected()) {
    result.error = "network is not connected";
    return result;
  }
  if (audio_bytes.empty()) {
    result.ok = true;
    return result;
  }

  const std::string endpoint = this->build_endpoint_url_();
  esp_http_client_config_t config = {};
  config.url = endpoint.c_str();
  config.method = HTTP_METHOD_POST;
  config.timeout_ms = this->http_timeout_ms_;
  config.buffer_size = HTTP_BUFFER_SIZE;
  config.buffer_size_tx = HTTP_BUFFER_SIZE;

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
  if (endpoint.find("https:") != std::string::npos) {
    config.crt_bundle_attach = esp_crt_bundle_attach;
  }
#endif

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    result.error = "HTTP client could not be initialized";
    return result;
  }

  const std::string sample_rate = std::to_string(stream_info.get_sample_rate());
  const std::string bits_per_sample = std::to_string(stream_info.get_bits_per_sample());
  const std::string channels = std::to_string(stream_info.get_channels());
  esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
  esp_http_client_set_header(client, "X-Audio-Format", "pcm_s16le");
  esp_http_client_set_header(client, "X-Audio-Rate", sample_rate.c_str());
  esp_http_client_set_header(client, "X-Audio-Bits", bits_per_sample.c_str());
  esp_http_client_set_header(client, "X-Audio-Channels", channels.c_str());
  esp_http_client_set_header(client, "X-Source-Device", this->source_device_.c_str());
  esp_http_client_set_header(client, "X-Wake-Word", this->wake_word_.c_str());

  esp_err_t err = esp_http_client_open(client, audio_bytes.size());
  if (err != ESP_OK) {
    result.error = std::string("open failed: ") + esp_err_to_name(err);
    esp_http_client_cleanup(client);
    return result;
  }

  size_t bytes_written = 0;
  const char *payload = reinterpret_cast<const char *>(audio_bytes.data());
  while (bytes_written < audio_bytes.size()) {
    const size_t remaining = audio_bytes.size() - bytes_written;
    const size_t chunk_size = std::min(remaining, HTTP_BUFFER_SIZE);
    const int written = esp_http_client_write(client, payload + bytes_written, chunk_size);
    if (written <= 0) {
      result.error = "write failed";
      esp_http_client_close(client);
      esp_http_client_cleanup(client);
      return result;
    }
    bytes_written += written;
    delay(0);
  }

  (void) esp_http_client_fetch_headers(client);
  const int status_code = esp_http_client_get_status_code(client);
  char response_buffer[512] = {};
  const int read_len = esp_http_client_read_response(client, response_buffer, sizeof(response_buffer) - 1);
  esp_http_client_close(client);
  esp_http_client_cleanup(client);

  if ((status_code < 200) || (status_code >= 300)) {
    result.error = "HTTP status " + std::to_string(status_code);
    return result;
  }
  if (read_len < 0) {
    result.error = "read failed";
    return result;
  }

  const std::string response(response_buffer, read_len > 0 ? read_len : 0);
  result.ok = true;
  result.detected = response_has_true_field(response, "detected");
  if (result.detected) {
    result.wake_word = json_string_field(response, "wake_word");
    result.score = json_float_field(response, "score", 0.0f);
  }
  return result;
}

std::string RemoteWakeWord::build_endpoint_url_() const {
  std::string endpoint = trim_copy(this->url_);
  if (endpoint.empty()) {
    return "";
  }
  if (endpoint.find("/api/openwakeword/detect") != std::string::npos) {
    return endpoint;
  }
  while (!endpoint.empty() && endpoint.back() == '/') {
    endpoint.pop_back();
  }
  return endpoint + "/api/openwakeword/detect";
}

void RemoteWakeWord::queue_event_(RemoteWakeWordEvent::Type type, const std::string &wake_word,
                                  const std::string &message, float score) {
  if (this->event_queue_ == nullptr) {
    return;
  }
  auto *event = new RemoteWakeWordEvent{type, wake_word, message, score};
  if (xQueueSend(this->event_queue_, &event, 0) != pdTRUE) {
    delete event;
    return;
  }
  App.wake_loop_threadsafe();
}

void RemoteWakeWord::set_state_(State state) {
  if (this->state_ != state) {
    ESP_LOGD(TAG, "State changed from %s to %s", LOG_STR_ARG(remote_wake_word_state_to_string(this->state_)),
             LOG_STR_ARG(remote_wake_word_state_to_string(state)));
    this->state_ = state;
  }
}

}  // namespace remote_wake_word
}  // namespace esphome

#endif  // USE_ESP32
