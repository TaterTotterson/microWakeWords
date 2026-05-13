#include "version_text_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/version.h"

namespace esphome {
namespace version {

static const char *const TAG = "version.text_sensor";

void VersionTextSensor::setup() {
  if (this->hide_timestamp_) {
    this->publish_state(ESPHOME_VERSION "+" + this->git_commit_);
  } else {
    this->publish_state(ESPHOME_VERSION "+" + this->git_commit_ + " " + App.get_compilation_time());
  }
}
void VersionTextSensor::set_hide_timestamp(bool hide_timestamp) { this->hide_timestamp_ = hide_timestamp; }
void VersionTextSensor::dump_config() { LOG_TEXT_SENSOR("", "Version Text Sensor", this); }

}  // namespace version
}  // namespace esphome
