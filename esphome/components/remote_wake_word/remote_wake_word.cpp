#include "remote_wake_word.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

#include <esp_event.h>
#include <esp_websocket_client.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace esphome {
namespace remote_wake_word {

static const char *const TAG = "remote_wake_word";
static const size_t DATA_TIMEOUT_MS = 50;
static const uint32_t RING_BUFFER_DURATION_MS = 2000;
static const uint32_t TASK_STACK_SIZE = 12288;
static const UBaseType_t TASK_PRIORITY = 2;
static const ssize_t EVENT_QUEUE_LENGTH = 4;
static const ssize_t WEBSOCKET_EVENT_QUEUE_LENGTH = 8;

struct RemoteWakeWordWebSocketEvent {
  enum Type {
    CONNECTED,
    DISCONNECTED,
    ERROR,
    DATA,
  };
  Type type;
  std::string data;
};

struct RemoteWakeWordWebSocketContext {
  QueueHandle_t queue{nullptr};
};

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

static bool starts_with_websocket(const std::string &value) {
  return value.rfind("ws://", 0) == 0 || value.rfind("wss://", 0) == 0;
}

static std::string url_encode(const std::string &value) {
  std::string encoded;
  encoded.reserve(value.size());
  char buffer[4] = {};
  for (const unsigned char ch : value) {
    if (std::isalnum(ch) || ch == '-' || ch == '_' || ch == '.' || ch == '~') {
      encoded.push_back(static_cast<char>(ch));
    } else {
      std::snprintf(buffer, sizeof(buffer), "%%%02X", ch);
      encoded.append(buffer);
    }
  }
  return encoded;
}

static void queue_websocket_event(QueueHandle_t queue, RemoteWakeWordWebSocketEvent::Type type,
                                  const char *data = nullptr, int data_len = 0) {
  if (queue == nullptr) {
    return;
  }
  auto *event = new RemoteWakeWordWebSocketEvent();
  event->type = type;
  if (data != nullptr && data_len > 0) {
    event->data.assign(data, data_len);
  }
  if (xQueueSend(queue, &event, 0) != pdTRUE) {
    delete event;
  }
}

static void websocket_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id,
                                    void *event_data) {
  auto *context = static_cast<RemoteWakeWordWebSocketContext *>(handler_args);
  if (context == nullptr) {
    return;
  }
  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      queue_websocket_event(context->queue, RemoteWakeWordWebSocketEvent::CONNECTED);
      break;
    case WEBSOCKET_EVENT_DISCONNECTED:
      queue_websocket_event(context->queue, RemoteWakeWordWebSocketEvent::DISCONNECTED);
      break;
    case WEBSOCKET_EVENT_ERROR:
      queue_websocket_event(context->queue, RemoteWakeWordWebSocketEvent::ERROR);
      break;
    case WEBSOCKET_EVENT_DATA: {
      auto *data = static_cast<esp_websocket_event_data_t *>(event_data);
      if (data == nullptr || data->data_ptr == nullptr || data->data_len <= 0) {
        return;
      }
      if (data->op_code != 0x1 || (data->payload_len > data->data_len && data->payload_offset != 0)) {
        return;
      }
      queue_websocket_event(context->queue, RemoteWakeWordWebSocketEvent::DATA, data->data_ptr, data->data_len);
      break;
    }
    default:
      break;
  }
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
  const std::string endpoint = trim_copy(this->url_);
  ESP_LOGCONFIG(TAG, "Remote Wake Word:");
  ESP_LOGCONFIG(TAG, "  endpoint: %s", endpoint.empty() ? "not configured" : endpoint.c_str());
  ESP_LOGCONFIG(TAG, "  transport: WebSocket stream");
  ESP_LOGCONFIG(TAG, "  wake word hint: %s", this->wake_word_.empty() ? "any" : this->wake_word_.c_str());
  ESP_LOGCONFIG(TAG, "  source device: %s", this->source_device_.empty() ? "not configured" : this->source_device_.c_str());
  ESP_LOGCONFIG(TAG, "  chunk duration: %ums", static_cast<unsigned int>(this->chunk_duration_ms_));
  ESP_LOGCONFIG(TAG, "  max failures before fallback: %u", static_cast<unsigned int>(this->max_failures_));
  ESP_LOGCONFIG(TAG, "  transport timeout: %ums", static_cast<unsigned int>(this->http_timeout_ms_));
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
  const std::string endpoint = trim_copy(this->url_);
  if (endpoint.empty()) {
    this->queue_event_(RemoteWakeWordEvent::ERROR, "", "Remote wake endpoint is not configured.", 0.0f);
    return;
  }
  if (!starts_with_http(endpoint) && !starts_with_websocket(endpoint)) {
    this->queue_event_(RemoteWakeWordEvent::ERROR, "", "Remote wake endpoint must be an HTTP or WebSocket URL.", 0.0f);
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

        {
          const std::string stream_endpoint = parent->build_stream_endpoint_url_(stream_info);
          QueueHandle_t websocket_queue = xQueueCreate(WEBSOCKET_EVENT_QUEUE_LENGTH, sizeof(RemoteWakeWordWebSocketEvent *));
          RemoteWakeWordWebSocketContext websocket_context{websocket_queue};
          esp_websocket_client_config_t websocket_config = {};
          websocket_config.uri = stream_endpoint.c_str();
          websocket_config.disable_auto_reconnect = true;
          websocket_config.network_timeout_ms = std::max<uint32_t>(parent->http_timeout_ms_, 3000);
          websocket_config.reconnect_timeout_ms = std::max<uint32_t>(parent->http_timeout_ms_, 3000);
          websocket_config.ping_interval_sec = 15;
          websocket_config.buffer_size = 1024;
          websocket_config.task_stack = 4096;
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
          if (stream_endpoint.find("wss:") != std::string::npos) {
            websocket_config.crt_bundle_attach = esp_crt_bundle_attach;
          }
#endif

          esp_websocket_client_handle_t websocket_client = nullptr;
          if (websocket_queue == nullptr) {
            terminal_error = "WebSocket event queue could not be initialized.";
          } else {
            websocket_client = esp_websocket_client_init(&websocket_config);
            if (websocket_client == nullptr) {
              terminal_error = "WebSocket client could not be initialized.";
            }
          }

          if (terminal_error.empty()) {
            esp_websocket_register_events(websocket_client, WEBSOCKET_EVENT_ANY, websocket_event_handler,
                                          &websocket_context);
            const esp_err_t err = esp_websocket_client_start(websocket_client);
            if (err != ESP_OK) {
              terminal_error = std::string("WebSocket start failed: ") + esp_err_to_name(err);
            }
          }

          bool websocket_connected = false;
          auto drain_websocket_events = [&]() {
            RemoteWakeWordWebSocketEvent *event = nullptr;
            while (websocket_queue != nullptr && xQueueReceive(websocket_queue, &event, 0) == pdTRUE) {
              delete event;
            }
          };
          auto handle_websocket_event = [&](RemoteWakeWordWebSocketEvent *event) {
            if (event == nullptr) {
              return;
            }
            if (event->type == RemoteWakeWordWebSocketEvent::CONNECTED) {
              websocket_connected = true;
              parent->consecutive_failures_.store(0);
            } else if (event->type == RemoteWakeWordWebSocketEvent::DISCONNECTED) {
              if (!parent->stop_requested_.load() && detected_wake_word.empty()) {
                terminal_error = "WebSocket disconnected.";
              }
            } else if (event->type == RemoteWakeWordWebSocketEvent::ERROR) {
              if (!parent->stop_requested_.load() && detected_wake_word.empty()) {
                terminal_error = "WebSocket error.";
              }
            } else if (event->type == RemoteWakeWordWebSocketEvent::DATA) {
              if (response_has_true_field(event->data, "detected")) {
                detected_wake_word = json_string_field(event->data, "wake_word");
                if (detected_wake_word.empty()) {
                  detected_wake_word = parent->wake_word_.empty() ? "openwakeword" : parent->wake_word_;
                }
                detected_score = json_float_field(event->data, "score", 0.0f);
                parent->stop_requested_.store(true);
              } else if (response_has_true_field(event->data, "ok") == false &&
                         event->data.find("\"ok\"") != std::string::npos) {
                const std::string message = json_string_field(event->data, "error");
                terminal_error = message.empty() ? "WebSocket server returned an error." : message;
              }
            }
          };

          if (terminal_error.empty()) {
            const uint32_t connect_started_ms = millis();
            while (!websocket_connected && terminal_error.empty() && !parent->stop_requested_.load()) {
              RemoteWakeWordWebSocketEvent *event = nullptr;
              if (xQueueReceive(websocket_queue, &event, pdMS_TO_TICKS(50)) == pdTRUE) {
                handle_websocket_event(event);
                delete event;
              }
              if (!websocket_connected && (millis() - connect_started_ms) >= parent->http_timeout_ms_) {
                terminal_error = "WebSocket connect timed out after " + std::to_string(parent->http_timeout_ms_) + "ms.";
                break;
              }
            }
          }

          while (terminal_error.empty() && !parent->stop_requested_.load()) {
            RemoteWakeWordWebSocketEvent *event = nullptr;
            while (xQueueReceive(websocket_queue, &event, 0) == pdTRUE) {
              handle_websocket_event(event);
              delete event;
              if (!terminal_error.empty() || !detected_wake_word.empty()) {
                break;
              }
            }
            if (!terminal_error.empty() || !detected_wake_word.empty()) {
              break;
            }

            audio_buffer->transfer_data_from_source(pdMS_TO_TICKS(DATA_TIMEOUT_MS));
            if (audio_buffer->available() < bytes_to_process) {
              continue;
            }

            std::vector<uint8_t> audio_bytes(bytes_to_process);
            std::memcpy(audio_bytes.data(), audio_buffer->get_buffer_start(), bytes_to_process);
            audio_buffer->decrease_buffer_length(bytes_to_process);

            const int sent =
                esp_websocket_client_send_bin(websocket_client, reinterpret_cast<const char *>(audio_bytes.data()),
                                              audio_bytes.size(), pdMS_TO_TICKS(parent->http_timeout_ms_));
            if (sent != static_cast<int>(audio_bytes.size())) {
              const uint8_t failures = parent->consecutive_failures_.fetch_add(1) + 1;
              ESP_LOGW(TAG, "Remote wake WebSocket send failed (%u/%u): sent %d of %u bytes",
                       static_cast<unsigned int>(failures), static_cast<unsigned int>(parent->max_failures_), sent,
                       static_cast<unsigned int>(audio_bytes.size()));
              if (failures >= parent->max_failures_) {
                terminal_error = "WebSocket send failed.";
                break;
              }
              delay(100);
              continue;
            }
            parent->consecutive_failures_.store(0);
            delay(0);
          }

          if (websocket_client != nullptr) {
            esp_websocket_client_stop(websocket_client);
            esp_websocket_client_destroy(websocket_client);
          }
          drain_websocket_events();
          if (websocket_queue != nullptr) {
            vQueueDelete(websocket_queue);
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

std::string RemoteWakeWord::build_stream_endpoint_url_(const audio::AudioStreamInfo &stream_info) const {
  std::string endpoint = trim_copy(this->url_);
  if (endpoint.empty()) {
    return "";
  }
  if (endpoint.rfind("http://", 0) == 0) {
    endpoint.replace(0, 7, "ws://");
  } else if (endpoint.rfind("https://", 0) == 0) {
    endpoint.replace(0, 8, "wss://");
  }
  const std::string detect_path = "/api/openwakeword/detect";
  const std::string stream_path = "/api/openwakeword/stream";
  const size_t detect_pos = endpoint.find(detect_path);
  if (detect_pos != std::string::npos) {
    endpoint.replace(detect_pos, detect_path.size(), stream_path);
  } else if (endpoint.find(stream_path) == std::string::npos) {
    while (!endpoint.empty() && endpoint.back() == '/') {
      endpoint.pop_back();
    }
    endpoint += stream_path;
  }

  const char separator = endpoint.find('?') == std::string::npos ? '?' : '&';
  endpoint.push_back(separator);
  endpoint += "selector=" + url_encode(this->source_device_);
  if (!this->wake_word_.empty()) {
    endpoint += "&wake_word=" + url_encode(this->wake_word_);
  }
  endpoint += "&rate=" + std::to_string(stream_info.get_sample_rate());
  endpoint += "&bits=" + std::to_string(stream_info.get_bits_per_sample());
  endpoint += "&channels=" + std::to_string(stream_info.get_channels());
  return endpoint;
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
