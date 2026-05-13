#include "sat_gpio.h"

namespace esphome {
namespace satellite1 {

static const char *TAG = "Satellite1-GPIOs";

void Satellite1GPIOPin::digital_write(bool value) {
  if (this->port_ != XMOSPort::OUTPUT_A) {
    ESP_LOGE(TAG, "Trying writing to read only port.");
    return;
  }
  uint8_t payload[2] = {this->pin_, value};
  this->parent_->transfer(DC_RESOURCE::GPIO_PORT_OUT_A, GPIO_SERVICER_CMD_SET_PIN, payload, 2);
}

bool Satellite1GPIOPin::digital_read() {
  DC_STATUS_REGISTER::register_id port_register;
  switch (this->port_) {
    case XMOSPort::INPUT_A:
      port_register = DC_STATUS_REGISTER::GPIO_PORT_IN_A;
      break;
    case XMOSPort::INPUT_B:
      port_register = DC_STATUS_REGISTER::GPIO_PORT_IN_B;
      break;
    case XMOSPort::OUTPUT_A:
      port_register = DC_STATUS_REGISTER::GPIO_PORT_OUT_A;
      break;
    default:
      ESP_LOGE(TAG, "Invalid port set.");
      return 0;
      break;
  }
  this->parent_->request_status_register_update();
  uint8_t port_value = this->parent_->get_dc_status(port_register);
  return !!(port_value & (1 << this->pin_)) != this->inverted_;
}

}  // namespace satellite1
}  // namespace esphome