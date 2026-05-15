#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#include "pd.h"

namespace esphome {
namespace power_delivery {

enum FUSB302_state_t { FUSB302_STATE_UNATTACHED = 0, FUSB302_STATE_ATTACHED, FUSB302_STATE_FAILED };

typedef union {
  uint8_t bytes[7];
  struct {
    uint8_t status0a;
    uint8_t status1a;
    uint8_t interrupta;
    uint8_t interruptb;
    uint8_t status0;
    uint8_t status1;
    uint8_t interrupt;
  };
} fusb_status;

class FUSB302B : public PowerDelivery, public Component, protected i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void loop() override;

  bool send_message_(const PDMsg &msg) override;
  bool read_message_(PDMsg &msg) override;
  bool read_status() {
    fusb_status regs;
    return read_status(regs);
  }
  bool read_status(fusb_status &status);

  bool read_status_register(uint8_t reg, uint8_t &value);

  void set_irq_pin(int irq_pin) { this->irq_pin_ = irq_pin; }
  void set_i2c_address(uint8_t address) { i2c::I2CDevice::set_i2c_address(address); }
  void set_i2c_bus(i2c::I2CBus *bus) { i2c::I2CDevice::set_i2c_bus(bus); }

  bool check_chip_id();
  bool enable_auto_crc();
  bool disable_auto_crc();

 public:
  bool cc_line_selection_();
  void fusb_reset_();

  void check_status_();

  FUSB302_state_t state_{FUSB302_STATE_UNATTACHED};

  uint32_t response_timer_{0};
  uint32_t startup_delay_{0};

  int irq_pin_{0};

 protected:
  void publish_() override {
    this->defer([this]() { this->state_callback_.call(); });
  }

  bool init_fusb_settings_();

  SemaphoreHandle_t i2c_lock_;
};

}  // namespace power_delivery
}  // namespace esphome