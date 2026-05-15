#include "i2s_audio_microphone.h"

#ifdef USE_ESP32

#include <driver/i2s.h>
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef I2S_EXTERNAL_ADC
#include "../external_adc.h"
#endif

namespace esphome {
namespace i2s_audio {

static const size_t BUFFER_SIZE = 512;

static const char *const TAG = "i2s_audio.microphone";

void I2SAudioMicrophone::setup() {
  ESP_LOGCONFIG(TAG, "Setting up I2S Audio Microphone...");
#if SOC_I2S_SUPPORTS_ADC
  if (this->use_internal_adc_) {
    if (this->parent_->get_port() != I2S_NUM_0) {
      ESP_LOGE(TAG, "Internal ADC only works on I2S0!");
      this->mark_failed();
      return;
    }
  } else
#endif
      if (this->pdm_) {
    if (this->parent_->get_port() != I2S_NUM_0) {
      ESP_LOGE(TAG, "PDM only works on I2S0!");
      this->mark_failed();
      return;
    }
  }
}
void I2SAudioMicrophone::dump_config() { this->dump_i2s_settings(); }

void I2SAudioMicrophone::start() {
  if (this->is_failed())
    return;
  if (this->state_ == microphone::STATE_RUNNING)
    return;  // Already running
  this->state_ = microphone::STATE_STARTING;
}
void I2SAudioMicrophone::start_() {
  if (!this->claim_i2s_access()) {
    return;  // Waiting for another i2s to return lock
  }

#ifdef I2S_EXTERNAL_ADC
  if (this->external_adc_ != nullptr) {
    this->external_adc_->init_device();
  }
#endif

  i2s_driver_config_t config = this->get_i2s_cfg();
  // config.mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_RX);

#if SOC_I2S_SUPPORTS_ADC
  if (this->use_internal_adc_) {
    config.mode = (i2s_mode_t) (config.mode | I2S_MODE_ADC_BUILT_IN);
    i2s_driver_install(this->parent_->get_port(), &config, 0, nullptr);

    i2s_set_adc_mode(ADC_UNIT_1, this->adc_channel_);
    i2s_adc_enable(this->parent_->get_port());
  } else
#endif
  {
    if (this->pdm_)
      config.mode = (i2s_mode_t) (config.mode | I2S_MODE_PDM);

    this->install_i2s_driver(config);
  }

#ifdef I2S_EXTERNAL_ADC
  if (this->external_adc_ != nullptr) {
    this->external_adc_->apply_i2s_settings(config);
  }
#endif

  this->state_ = microphone::STATE_RUNNING;
  this->high_freq_.start();
}

void I2SAudioMicrophone::stop() {
  if (this->state_ == microphone::STATE_STOPPED || this->is_failed())
    return;
  if (this->state_ == microphone::STATE_STARTING) {
    this->state_ = microphone::STATE_STOPPED;
    return;
  }
  this->state_ = microphone::STATE_STOPPING;
}

void I2SAudioMicrophone::stop_() {
  this->uninstall_i2s_driver();
  this->release_i2s_access();
  this->state_ = microphone::STATE_STOPPED;
  this->high_freq_.stop();
}

size_t I2SAudioMicrophone::read(int16_t *buf, size_t len) {
  size_t bytes_read = 0;
  esp_err_t err = i2s_read(this->parent_->get_port(), buf, len, &bytes_read, (1 / portTICK_PERIOD_MS));
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Error reading from I2S microphone: %s", esp_err_to_name(err));
    this->status_set_warning();
    return 0;
  }

  if (bytes_read == 0) {
    return 0;
  }
  this->status_clear_warning();
  if (this->bits_per_sample_ == I2S_BITS_PER_SAMPLE_16BIT) {
    return bytes_read;
  } else if (this->bits_per_sample_ == I2S_BITS_PER_SAMPLE_32BIT) {
    std::vector<int16_t> samples;
    size_t samples_read = bytes_read / sizeof(int32_t);
    uint8_t shift = 16 - this->gain_log2_;
    samples.resize(samples_read);
    for (size_t i = 0; i < samples_read; i++) {
      int32_t temp = reinterpret_cast<int32_t *>(buf)[i] >> shift;
      samples[i] = static_cast<int16_t>(clamp<int32_t>(temp, INT16_MIN, INT16_MAX));
    }
    memcpy(buf, samples.data(), samples_read * sizeof(int16_t));
    return samples_read * sizeof(int16_t);
  } else {
    ESP_LOGE(TAG, "Unsupported bits per sample: %d", this->bits_per_sample_);
    return 0;
  }
}

void I2SAudioMicrophone::read_() {
  std::vector<int16_t> samples;
  samples.resize(BUFFER_SIZE);
  size_t bytes_read = this->read(samples.data(), BUFFER_SIZE / sizeof(int16_t));
  samples.resize(bytes_read / sizeof(int16_t));
  this->data_callbacks_.call(samples);
}

void I2SAudioMicrophone::loop() {
  switch (this->state_) {
    case microphone::STATE_STOPPED:
      break;
    case microphone::STATE_STARTING:
      this->start_();
      break;
    case microphone::STATE_RUNNING:
      if (this->data_callbacks_.size() > 0) {
        this->read_();
      }
      break;
    case microphone::STATE_STOPPING:
      this->stop_();
      break;
  }
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
