#include "i2s_audio.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

namespace esphome {
namespace i2s_audio {

static const char *const TAG = "i2s_audio";

#if defined(USE_ESP_IDF) && (ESP_IDF_VERSION_MAJOR >= 5)
static const uint8_t I2S_NUM_MAX = SOC_I2S_NUM;  // because IDF 5+ took this away :(
#endif

static const size_t DMA_BUFFERS_COUNT = 4;
static const size_t I2S_EVENT_QUEUE_COUNT = DMA_BUFFERS_COUNT + 1;

void I2SAudioBase::dump_i2s_settings() const {
  std::string init_str = this->is_fixed_ ? "Fixed-CFG" : "Initial-CFG";
  if (this->i2s_access_ == I2SAccess::RX) {
    ESP_LOGCONFIG(TAG, "I2S-Reader (%s):", init_str.c_str());
  } else {
    ESP_LOGCONFIG(TAG, "I2S-Writer (%s):", init_str.c_str());
  }
  ESP_LOGCONFIG(TAG, "  sample-rate: %d slot_mode: %d slot_mask: %d slot_bit_width: %d", this->sample_rate_,
                this->slot_mode_, this->std_slot_mask_, this->slot_bit_width_);
  ESP_LOGCONFIG(TAG, "  use_apll: %s", this->use_apll_ ? "yes" : "no");
}

void I2SPortComponent::setup() {
  static i2s_port_t next_port_num = I2S_NUM_0;

  if (next_port_num >= I2S_NUM_MAX) {
    ESP_LOGE(TAG, "Too many I2S Audio components!");
    this->mark_failed();
    return;
  }

  this->port_ = next_port_num;
  next_port_num = (i2s_port_t) (next_port_num + 1);

  ESP_LOGCONFIG(TAG, "Setting up I2S Audio...");
}

void I2SPortComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "I2SController:");
  ESP_LOGCONFIG(TAG, "  role: %s", this->i2s_role_ == I2S_ROLE_MASTER ? "primary" : "secondary");
  ESP_LOGCONFIG(TAG, "  AccessMode: %s", this->access_mode_ == I2SAccessMode::DUPLEX ? "duplex" : "exclusive");
  ESP_LOGCONFIG(TAG, "  Port: %d", this->get_port());
  if (this->audio_in_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Reader registered.");
  }
  if (this->audio_out_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Writer registered.");
  }
}

bool I2SPortComponent::claim_access_(uint8_t access) {
  bool success = false;
  this->lock();
  if (this->access_mode_ == I2SAccessMode::DUPLEX) {
    this->access_state_ |= access;
    success = true;
  } else {
    if (this->access_state_ == I2SAccess::FREE) {
      this->access_state_ = access;
    }
    success = this->access_state_ & access;
  }
  this->unlock();
  return success;
}

bool I2SPortComponent::release_access_(uint8_t access) {
  this->lock();
  this->access_state_ = this->access_state_ & (~access);
  this->unlock();
  return true;
}

i2s_std_gpio_config_t I2SPortComponent::get_pin_config() const {
  return {.mclk = (gpio_num_t) this->mclk_pin_,
          .bclk = (gpio_num_t) this->bclk_pin_,
          .ws = (gpio_num_t) this->lrclk_pin_,
          .dout = this->audio_out_ == nullptr ? I2S_GPIO_UNUSED : (gpio_num_t) this->audio_out_->get_dout_pin(),
          .din = this->audio_in_ == nullptr ? I2S_GPIO_UNUSED : (gpio_num_t) this->audio_in_->get_din_pin(),
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false,
          }};
}

bool I2SPortComponent::init_driver_(i2s_std_config_t std_cfg) {
  this->lock();
  i2s_chan_config_t chan_cfg = {
      .id = this->get_port(),
      .role = this->i2s_role_,
      .dma_desc_num = this->dma_buffer_count_,
      .dma_frame_num = this->dma_buffer_length_,
      .auto_clear = true,
  };

  i2s_chan_handle_t *pTxHandle = this->audio_out_ != nullptr ? &this->tx_handle_ : nullptr;
  i2s_chan_handle_t *pRxHandle = this->audio_in_ != nullptr ? &this->rx_handle_ : nullptr;

  /* Allocate channels and receive their handles*/
  esp_err_t err = i2s_new_channel(&chan_cfg, pTxHandle, pRxHandle);
  if (err != ESP_OK) {
    this->unlock();
    return false;
  }

  if (this->tx_handle_) {
    err = i2s_channel_init_std_mode(this->tx_handle_, &std_cfg);
    if (err != ESP_OK) {
      i2s_del_channel(this->tx_handle_);
      this->unlock();
      return false;
    }
  }

  if (this->rx_handle_) {
    err = i2s_channel_init_std_mode(this->rx_handle_, &std_cfg);
    if (err != ESP_OK) {
      i2s_del_channel(this->rx_handle_);
      this->unlock();
      return false;
    }
  }

  this->unlock();
  return true;
}

bool I2SAudioOut::start_i2s_channel_(i2s_event_callbacks_t callbacks) {
  if (this->parent_->tx_handle_ == nullptr) {
    if (this->parent_->rx_handle_ != nullptr) {
      ESP_LOGE(TAG, "Trying to start I2S-TX channel, but RX handle is available. This is not allowed.");
      return false;
    }
    i2s_std_config_t std_cfg = {.clk_cfg = this->get_std_clk_cfg(),
                                .slot_cfg = this->get_std_slot_cfg(),
                                .gpio_cfg = this->parent_->get_pin_config()};
    if (!this->parent_->init_driver_(std_cfg)) {
      ESP_LOGE(TAG, "Failed to initialize I2S driver for TX channel.");
      return false;
    }
  }

  i2s_chan_info_t chan_info;
  esp_err_t err = i2s_channel_get_info(this->parent_->tx_handle_, &chan_info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get TX channel info: %s", esp_err_to_name(err));
    return false;
  } else if (chan_info.dir != I2S_DIR_TX) {
    ESP_LOGE(TAG, "TX channel is not configured for TX direction");
    return false;
  }

  if (callbacks.on_sent != nullptr) {
    err = i2s_channel_register_event_callback(this->parent_->tx_handle_, &callbacks, this);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to register TX channel callbacks: %s", esp_err_to_name(err));
      return false;
    }
  }

  err = i2s_channel_enable(this->parent_->tx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable TX channel: %s", esp_err_to_name(err));
    i2s_del_channel(this->parent_->tx_handle_);
    return false;
  }
  return true;
}

bool I2SAudioOut::stop_i2s_channel_() {
  if (this->parent_->tx_handle_ == nullptr) {
    ESP_LOGE(TAG, "Trying to stop I2S-TX channel, but handle is nullptr.");
    return false;
  }
  if (this->parent_->tx_handle_ == nullptr) {
    return false;
  }

  esp_err_t err = i2s_channel_disable(this->parent_->tx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disable TX channel: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

bool I2SAudioIn::start_i2s_channel_(i2s_event_callbacks_t callbacks) {
  if (this->parent_->rx_handle_ == nullptr) {
    if (this->parent_->tx_handle_ != nullptr) {
      ESP_LOGE(TAG, "Trying to start I2S-RX channel, but TX handle is available. This is not allowed.");
      return false;
    }
    i2s_std_config_t std_cfg = {.clk_cfg = this->get_std_clk_cfg(),
                                .slot_cfg = this->get_std_slot_cfg(),
                                .gpio_cfg = this->parent_->get_pin_config()};
    if (!this->parent_->init_driver_(std_cfg)) {
      ESP_LOGE(TAG, "Failed to initialize I2S driver for RX channel.");
      return false;
    }
  }
  i2s_chan_info_t chan_info;
  esp_err_t err = i2s_channel_get_info(this->parent_->rx_handle_, &chan_info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get RX channel info: %s", esp_err_to_name(err));
    return false;
  }

  if (chan_info.dir != I2S_DIR_RX) {
    ESP_LOGE(TAG, "RX channel is not configured for RX direction");
    return false;
  }

  err = i2s_channel_enable(this->parent_->rx_handle_);
  if (err != ESP_OK) {
    i2s_del_channel(this->parent_->rx_handle_);
    return false;
  }
  return true;
}

bool I2SAudioIn::stop_i2s_channel_() {
  if (this->parent_->rx_handle_ == nullptr) {
    ESP_LOGE(TAG, "Trying to stop I2S-RX channel, but handle is nullptr.");
    return false;
  }

  esp_err_t err = i2s_channel_disable(this->parent_->rx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disable RX channel: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
