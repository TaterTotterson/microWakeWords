#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace version {

class VersionTextSensor : public text_sensor::TextSensor, public Component {
 public:
  void set_hide_timestamp(bool hide_timestamp);
  void set_git_commit(std::string commit_hash) { this->git_commit_ = commit_hash; }
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

 protected:
  bool hide_timestamp_{false};
  std::string git_commit_{};
};

}  // namespace version
}  // namespace esphome
