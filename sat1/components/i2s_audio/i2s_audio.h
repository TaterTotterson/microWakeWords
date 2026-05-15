#pragma once

#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/defines.h"
#include <driver/i2s_std.h>

namespace esphome {
namespace i2s_audio {

enum class I2SAccessMode : uint8_t { EXCLUSIVE, DUPLEX };
class I2SAccess {
 public:
  static constexpr uint8_t FREE = 0;
  static constexpr uint8_t RX = 1;
  static constexpr uint8_t TX = 2;
};

class I2SAudioBase {
 public:
  I2SAudioBase(uint8_t access) : i2s_access_(access) {}
  void set_slot_mode(i2s_slot_mode_t slot_mode) { this->slot_mode_ = slot_mode; }
  void set_std_slot_mask(i2s_std_slot_mask_t std_slot_mask) { this->std_slot_mask_ = std_slot_mask; }
  void set_slot_bit_width(i2s_slot_bit_width_t slot_bit_width) { this->slot_bit_width_ = slot_bit_width; }
  void set_i2s_comm_fmt(std::string mode) { this->i2s_comm_fmt_ = std::move(mode); }
  int num_of_channels() const {
    return (this->std_slot_mask_ == I2S_STD_SLOT_LEFT || this->std_slot_mask_ == I2S_STD_SLOT_RIGHT) ? 1 : 2;
  }
  uint8_t i2s_bits_per_sample() const { return (uint8_t) this->slot_bit_width_; }
  void set_sample_rate(uint32_t sample_rate) { this->sample_rate_ = sample_rate; }
  void set_use_apll(uint32_t use_apll) { this->use_apll_ = use_apll; }
  void set_mclk_multiple(i2s_mclk_multiple_t mclk_multiple) { this->mclk_multiple_ = mclk_multiple; }
  void set_pdm(bool pdm) { this->pdm_ = pdm; }

  void dump_i2s_settings() const;
  bool has_fixed_i2s_rate() const { return this->is_fixed_; }
  bool has_fixed_i2s_bitdepth() const { return this->is_fixed_; }

  virtual void register_at_parent() = 0;
  bool validate_for_duplex(I2SAudioBase *other) const {
    return ((this->i2s_access_ == other->i2s_access_) && (this->sample_rate_ == other->sample_rate_) &&
            (this->mclk_multiple_ == other->mclk_multiple_) && (this->slot_mode_ == other->slot_mode_) &&
            (this->std_slot_mask_ == other->std_slot_mask_) && (this->slot_bit_width_ == other->slot_bit_width_));
  }

  i2s_std_clk_config_t get_std_clk_cfg() const {
    return {
        .sample_rate_hz = this->sample_rate_,
#ifdef I2S_CLK_SRC_APLL
        .clk_src = this->use_appll_ ? I2S_CLK_SRC_APLL : I2S_CLK_SRC_DEFAULT,
#else
        .clk_src = I2S_CLK_SRC_DEFAULT,
#endif
        .mclk_multiple = this->mclk_multiple_,
    };
  }
  i2s_std_slot_config_t get_std_slot_cfg() const {
    i2s_slot_mode_t slot_mode = this->slot_mode_;
    i2s_std_slot_config_t std_slot_cfg;
    if (this->i2s_comm_fmt_ == "std") {
      std_slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t) this->slot_bit_width_, slot_mode);
    } else if (this->i2s_comm_fmt_ == "pcm") {
      std_slot_cfg = I2S_STD_PCM_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t) this->slot_bit_width_, slot_mode);
    } else {
      std_slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t) this->slot_bit_width_, slot_mode);
    }
#ifdef USE_ESP32_VARIANT_ESP32
    // There seems to be a bug on the ESP32 (non-variant) platform where setting the slot bit width higher then the bits
    // per sample causes the audio to play too fast. Setting the ws_width to the configured slot bit width seems to
    // make it play at the correct speed while sending more bits per slot.
    if (this->slot_bit_width_ != I2S_SLOT_BIT_WIDTH_AUTO) {
      std_slot_cfg.ws_width = static_cast<uint32_t>(this->slot_bit_width_);
    }
#else
    std_slot_cfg.slot_bit_width = this->slot_bit_width_;
#endif
    std_slot_cfg.slot_mask = this->std_slot_mask_;
    return std_slot_cfg;
  }

 protected:
  virtual bool start_i2s_channel_(i2s_event_callbacks_t callbacks) = 0;
  virtual bool start_i2s_channel_() {
    const i2s_event_callbacks_t callbacks = {};
    return this->start_i2s_channel_(callbacks);
  }
  virtual bool stop_i2s_channel_() = 0;

  virtual bool IRAM_ATTR i2s_overflow_cb(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
    return true;
  }

  i2s_slot_mode_t slot_mode_;
  i2s_std_slot_mask_t std_slot_mask_;
  i2s_slot_bit_width_t slot_bit_width_;
  std::string i2s_comm_fmt_;
  uint32_t sample_rate_;
  bool use_apll_;
  i2s_mclk_multiple_t mclk_multiple_;
  uint8_t i2s_access_;

  bool pdm_{false};
  bool is_fixed_{true};
};

class I2SAudioIn;
class I2SAudioOut;

class I2SPortComponent : public Component {
 public:
  void setup() override;
  void dump_config() override;

  i2s_std_gpio_config_t get_pin_config() const;

  void set_mclk_pin(int pin) { this->mclk_pin_ = pin; }
  void set_bclk_pin(int pin) { this->bclk_pin_ = pin; }
  void set_lrclk_pin(int pin) { this->lrclk_pin_ = pin; }

  void lock() { this->lock_.lock(); }
  bool try_lock() { return this->lock_.try_lock(); }
  void unlock() { this->lock_.unlock(); }

  i2s_port_t get_port() const { return this->port_; }

  void set_audio_in(I2SAudioIn *comp_in) { this->audio_in_ = comp_in; }
  void set_audio_out(I2SAudioOut *comp_out) { this->audio_out_ = comp_out; }

  void set_access_mode(I2SAccessMode access_mode) { this->access_mode_ = access_mode; }
  bool is_exclusive() { return this->access_mode_ == I2SAccessMode::EXCLUSIVE; }

  void set_i2s_role(i2s_role_t role) { this->i2s_role_ = role; }
  i2s_chan_handle_t get_tx_handle() const { return this->tx_handle_; }
  i2s_chan_handle_t get_rx_handle() const { return this->rx_handle_; }

 protected:
  friend I2SAudioIn;
  friend I2SAudioOut;

  Mutex lock_;
  I2SAccessMode access_mode_{I2SAccessMode::DUPLEX};
  uint8_t access_state_{I2SAccess::FREE};

  bool claim_access_(uint8_t access);
  bool release_access_(uint8_t access);
  bool init_driver_(i2s_std_config_t std_cfg);
  bool free_driver_();

  I2SAudioIn *audio_in_{nullptr};
  I2SAudioOut *audio_out_{nullptr};
  i2s_role_t i2s_role_{};
  i2s_chan_handle_t tx_handle_{nullptr};
  i2s_chan_handle_t rx_handle_{nullptr};
  int mclk_pin_{I2S_GPIO_UNUSED};
  int bclk_pin_{I2S_GPIO_UNUSED};
  int dout_pin_{I2S_GPIO_UNUSED};
  int din_pin_{I2S_GPIO_UNUSED};

  int lrclk_pin_;
  i2s_port_t port_{};
  size_t dma_buffer_length_{240};
  uint8_t dma_buffer_count_{4};

  QueueHandle_t i2s_event_queue_;
  bool driver_loaded_{false};
};

class I2SAudioIn : public I2SAudioBase, public Parented<I2SPortComponent> {
 public:
  I2SAudioIn() : I2SAudioBase(I2SAccess::RX) {}

  void set_din_pin(int8_t pin) { this->din_pin_ = (gpio_num_t) pin; }
  gpio_num_t get_din_pin() { return this->din_pin_; }

  void register_at_parent() override { this->parent_->set_audio_in(this); }

 protected:
  using I2SAudioBase::start_i2s_channel_;
  bool start_i2s_channel_(i2s_event_callbacks_t callbacks) override;
  bool stop_i2s_channel_() override;
  gpio_num_t din_pin_{I2S_GPIO_UNUSED};
};

class I2SAudioOut : public I2SAudioBase, public Parented<I2SPortComponent> {
 public:
  I2SAudioOut() : I2SAudioBase(I2SAccess::TX) {}

  void set_dout_pin(uint8_t pin) { this->dout_pin_ = (gpio_num_t) pin; }
  gpio_num_t get_dout_pin() { return this->dout_pin_; }
  size_t get_dma_buffer_size_bytes() const {
    return this->parent_->dma_buffer_length_ * this->num_of_channels() * this->i2s_bits_per_sample() / 8;
  }
  size_t get_dma_buffer_size_ms() const { return this->parent_->dma_buffer_length_ * 1000 / this->sample_rate_; }
  uint8_t get_dma_buffer_count() const { return this->parent_->dma_buffer_count_; }

  void register_at_parent() override { this->parent_->set_audio_out(this); }

  bool is_adjustable() const { return !this->is_fixed_ && this->parent_->is_exclusive(); }

 protected:
  using I2SAudioBase::start_i2s_channel_;
  bool start_i2s_channel_(i2s_event_callbacks_t callbacks) override;
  bool stop_i2s_channel_() override;
  gpio_num_t dout_pin_{I2S_GPIO_UNUSED};
};

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
