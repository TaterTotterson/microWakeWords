#include "satellite1.h"
#include "esp_rom_gpio.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1 {

static const char *TAG = "Satellite1";

void Satellite1::setup() {
  this->spi_setup();
  this->enable();
  this->transfer_byte(0);
  this->disable();

  if (this->xmos_rst_pin_) {
    this->xmos_rst_pin_->setup();
  }

  memset(this->xmos_fw_version, 0, 5);
  this->dfu_get_fw_version_();
}

void Satellite1::dump_config() {
  ESP_LOGCONFIG(TAG, "Satellite1 config:");
  if (!this->xmos_rst_pin_) {
    ESP_LOGCONFIG(TAG, "    xmos_rst_pin not set up properly.");
  }
}

void Satellite1::loop() {
  switch (this->state) {
    case SAT_DETACHED_STATE:
      if (this->connection_attempts <= MAX_CONNECTION_ATTEMPTS && (millis() - this->last_attempt_timestamp_) > 1000) {
        if (this->connection_attempts == MAX_CONNECTION_ATTEMPTS) {
          this->state_callback_.call();
        } else if (this->check_for_xmos_()) {
          this->state = SAT_XMOS_CONNECTED_STATE;
          this->connection_attempts = 0;
          this->state_callback_.call();
        }
        this->last_attempt_timestamp_ = millis();
        this->connection_attempts++;
      }
      break;
    case SAT_XMOS_CONNECTED_STATE:
    case SAT_FLASH_CONNECTED_STATE:
      break;
  }
}

static std::string prerelease_str(uint8_t pre_idx) {
  switch (pre_idx) {
    case 1:
      return "alpha";
    case 2:
      return "beta";
    case 3:
      return "rc";
    case 4:
      return "dev";
    case 0:  // fallthrough
    default:
      return "";
  }
}

std::string Satellite1::status_string() {
  switch (this->state) {
    case SAT_DETACHED_STATE:
      return "XMOS not responding";

    case SAT_XMOS_CONNECTED_STATE:
      return ("v" + std::to_string(this->xmos_fw_version[0]) + "." + std::to_string(this->xmos_fw_version[1]) + "." +
              std::to_string(this->xmos_fw_version[2]) +
              (this->xmos_fw_version[3] ? "-" + prerelease_str(this->xmos_fw_version[3]) : "") +
              (this->xmos_fw_version[4] ? "." + std::to_string(this->xmos_fw_version[4]) : ""));
    case SAT_FLASH_CONNECTED_STATE:
      return "Flashing Mode";
    default:
      return "";
  }
}

bool Satellite1::request_status_register_update() {
  bool ret = this->transfer(0, 0, NULL, 0);
  uint8_t *arr = this->dc_status_register_;
  return ret;
}

bool Satellite1::transfer(uint8_t resource_id, uint8_t command, uint8_t *payload, uint8_t payload_len) {
  if (this->spi_flash_direct_access_enabled_) {
    return false;
  }

  uint8_t send_recv_buf[256 + 3] = {0};
  int status_report_dummies = std::max<int>(0, DC_STATUS_REGISTER::REGISTER_LEN - payload_len - 1);

  int attempts = 3;
  do {
    send_recv_buf[0] = resource_id;
    send_recv_buf[1] = command;
    send_recv_buf[2] = payload_len + !!(command & CONTROL_CMD_READ_BIT);
    memcpy(&send_recv_buf[3], payload, payload_len);
    this->enable();
    this->transfer_array(&send_recv_buf[0], payload_len + 3 + status_report_dummies);
    this->disable();
    vTaskDelay(1);
  } while (send_recv_buf[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE && attempts-- > 0);

  if (send_recv_buf[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE) {
    return false;
  }

  // XMOS not responding at all
  if ((send_recv_buf[0] + send_recv_buf[1] + send_recv_buf[2]) == 0) {
    return false;
  }

  // Got status register report
  if (send_recv_buf[0] == DC_RESOURCE::CNTRL_ID && send_recv_buf[1] != DC_RET_STATUS::PAYLOAD_AVAILABLE) {
    memcpy(this->dc_status_register_, &send_recv_buf[2], DC_STATUS_REGISTER::REGISTER_LEN);
    uint8_t *arr = this->dc_status_register_;
  }

  if (command & CONTROL_CMD_READ_BIT) {
    attempts = 3;
    do {
      memset(send_recv_buf, 0, payload_len + 3);
      this->enable();
      this->transfer_array(&send_recv_buf[0], payload_len + 3);
      this->disable();
      vTaskDelay(1);
    } while (send_recv_buf[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE && attempts-- > 0);

    if (send_recv_buf[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE) {
      return false;
    }

    memcpy(payload, &send_recv_buf[1], payload_len);
  }

  return true;
}

void Satellite1::set_spi_flash_direct_access_mode(bool enable) {
  this->xmos_rst_pin_->digital_write(enable);
  if (enable) {
    this->state = SAT_FLASH_CONNECTED_STATE;
  } else if (this->spi_flash_direct_access_enabled_) {
    this->state = SAT_DETACHED_STATE;
    this->connection_attempts = 0;
  }
  this->spi_flash_direct_access_enabled_ = enable;
  this->state_callback_.call();
}

bool Satellite1::dfu_get_fw_version_() {
  uint8_t version_resp[5];
  if (!this->transfer(DC_RESOURCE::DFU_CONTROLLER, DC_DFU_CMD::GET_VERSION, version_resp, 5)) {
    ESP_LOGW(TAG, "Requesting XMOS version failed");
    return false;
  }

  memcpy(this->xmos_fw_version, version_resp, 5);
  ESP_LOGI(TAG, "XMOS Firmware Version: %s ", this->status_string().c_str());

  return true;
}

bool Satellite1::check_for_xmos_() {
  if (!this->dfu_get_fw_version_()) {
    return false;
  }
  const uint8_t compare_zeros[5] = {0};
  return (memcmp(this->xmos_fw_version, compare_zeros, 5) != 0);
}

void Satellite1::xmos_hardware_reset() {
  this->xmos_rst_pin_->digital_write(1);
  delay(100);
  this->xmos_rst_pin_->digital_write(0);
  delay(100);
}

}  // namespace satellite1
}  // namespace esphome