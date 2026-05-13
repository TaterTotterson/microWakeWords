#pragma once

#ifdef USE_ESP32

#include "../i2s_audio.h"

#include "esphome/components/microphone/microphone.h"
#include "esphome/core/component.h"

namespace esphome {
namespace i2s_audio {

class I2SAudioMicrophone : public I2SReader, public microphone::Microphone, public Component {
 public:
  void setup() override;
  void start() override;
  void stop() override;

  void loop() override;
  void dump_config() override;

  size_t read(int16_t *buf, size_t len) override;
  void set_gain_log2(uint8_t gain_log2) { this->gain_log2_ = gain_log2; }

 protected:
  void start_();
  void stop_();
  void read_();

  uint8_t gain_log2_{2};
  HighFrequencyLoopRequester high_freq_;
};

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
