#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <functional>

namespace esphome {
namespace satellite1_radar {

struct DeviceClassMeta {
  uint8_t distance{0};
  uint8_t illuminance{0};
  uint8_t occupancy{0};
  uint8_t motion{0};
};

struct UnitMeta {
  uint8_t centimeter{0};
  uint8_t percent{0};
};

struct IconMeta {
  uint8_t radar{0};
  uint8_t chip{0};
  uint8_t signal{0};
  uint8_t motion_sensor{0};
  uint8_t account_multiple{0};
  uint8_t account{0};
  uint8_t account_arrow_right{0};
  uint8_t tune_vertical{0};
  uint8_t factory{0};
  uint8_t restart{0};
  uint8_t database_refresh{0};
};

inline uint32_t pack_entity_fields(uint8_t device_class_idx, uint8_t uom_idx, uint8_t icon_idx, bool internal,
                                   bool disabled_by_default, EntityCategory entity_category) {
  return (static_cast<uint32_t>(device_class_idx) << ENTITY_FIELD_DC_SHIFT) |
         (static_cast<uint32_t>(uom_idx) << ENTITY_FIELD_UOM_SHIFT) |
         (static_cast<uint32_t>(icon_idx) << ENTITY_FIELD_ICON_SHIFT) |
         (static_cast<uint32_t>(internal) << ENTITY_FIELD_INTERNAL_SHIFT) |
         (static_cast<uint32_t>(disabled_by_default) << ENTITY_FIELD_DISABLED_BY_DEFAULT_SHIFT) |
         (static_cast<uint32_t>(entity_category) << ENTITY_FIELD_ENTITY_CATEGORY_SHIFT);
}

class Satellite1RadarDynamicSensor : public sensor::Sensor {
 public:
  void configure_dynamic(const char *name, EntityCategory entity_category = ENTITY_CATEGORY_NONE,
                         bool disabled_by_default = false, uint8_t device_class_idx = 0, uint8_t uom_idx = 0,
                         uint8_t icon_idx = 0, bool has_state_class = false,
                         sensor::StateClass state_class = sensor::STATE_CLASS_NONE, int8_t accuracy_decimals = -1) {
    this->configure_entity_(
        name, 0, pack_entity_fields(device_class_idx, uom_idx, icon_idx, false, disabled_by_default, entity_category));
    if (has_state_class)
      this->set_state_class(state_class);
    if (accuracy_decimals >= 0)
      this->set_accuracy_decimals(accuracy_decimals);
  }
};

class Satellite1RadarDynamicBinarySensor : public binary_sensor::BinarySensor {
 public:
  void configure_dynamic(const char *name, EntityCategory entity_category = ENTITY_CATEGORY_NONE,
                         bool disabled_by_default = false, uint8_t device_class_idx = 0, uint8_t icon_idx = 0) {
    this->configure_entity_(
        name, 0, pack_entity_fields(device_class_idx, 0, icon_idx, false, disabled_by_default, entity_category));
  }
};

class Satellite1RadarDynamicTextSensor : public text_sensor::TextSensor {
 public:
  void configure_dynamic(const char *name, EntityCategory entity_category = ENTITY_CATEGORY_NONE,
                         bool disabled_by_default = false, uint8_t icon_idx = 0) {
    this->configure_entity_(name, 0, pack_entity_fields(0, 0, icon_idx, false, disabled_by_default, entity_category));
  }
};

class Satellite1RadarSwitch : public switch_::Switch, public Component {
 public:
  void setup() override {}
  void dump_config() override {}

 protected:
  void write_state(bool state) override { this->publish_state(state); }
};

class Satellite1RadarTunerSwitch : public switch_::Switch, public Component {
 public:
  void setup() override {}
  void dump_config() override {}
  void configure_dynamic(const char *name, EntityCategory entity_category = ENTITY_CATEGORY_NONE,
                         bool disabled_by_default = false, uint8_t icon_idx = 0) {
    this->configure_entity_(name, 0, pack_entity_fields(0, 0, icon_idx, false, disabled_by_default, entity_category));
  }
  void set_write_callback(std::function<void(bool)> callback) { write_callback_ = std::move(callback); }

 protected:
  void write_state(bool state) override;
  std::function<void(bool)> write_callback_{};
};

class Satellite1RadarButton : public button::Button, public Component {
 public:
  void setup() override {}
  void dump_config() override {}
  void configure_dynamic(const char *name, EntityCategory entity_category = ENTITY_CATEGORY_NONE,
                         bool disabled_by_default = false, uint8_t icon_idx = 0) {
    this->configure_entity_(name, 0, pack_entity_fields(0, 0, icon_idx, false, disabled_by_default, entity_category));
  }

 protected:
  void press_action() override {}
};

}  // namespace satellite1_radar
}  // namespace esphome
