#pragma once

#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

namespace esphome {
namespace satellite1 {

static const uint8_t CONTROL_RESOURCE_CNTRL_ID = 1;
static const uint8_t CONTROL_CMD_READ_BIT = 0x80;

static const uint8_t RET_STATUS_PAYLOAD_AVAIL = 23;

static const uint8_t CONTROL_COMMAND_IGNORED_IN_DEVICE = 7;

static const uint8_t GPIO_SERVICER_RESID_PORT_IN_A = 211;
static const uint8_t GPIO_SERVICER_RESID_PORT_IN_B = 212;
static const uint8_t GPIO_SERVICER_RESID_PORT_OUT_A = 221;

static const uint8_t DFU_CONTROLLER_SERVICER_RESID = 240;

static const uint8_t MAX_CONNECTION_ATTEMPTS = 3;

namespace DC_RESOURCE {
enum dc_resource_enum {
  CNTRL_ID = 1,
  DFU_CONTROLLER = DFU_CONTROLLER_SERVICER_RESID,
  GPIO_PORT_IN_A = GPIO_SERVICER_RESID_PORT_IN_A,
  GPIO_PORT_IN_B = GPIO_SERVICER_RESID_PORT_IN_B,
  GPIO_PORT_OUT_A = GPIO_SERVICER_RESID_PORT_OUT_A
};
}

namespace DC_RET_STATUS {
enum dc_ret_status_enum { CMD_SUCCESS = 0, DEVICE_BUSY = 7, PAYLOAD_AVAILABLE = 23 };
}

namespace DC_STATUS_REGISTER {
enum register_id {
  DEVICE_STATUS = 0,
  GPIO_PORT_IN_A = 1,
  GPIO_PORT_IN_B = 2,
  GPIO_PORT_OUT_A = 3,

  REGISTER_LEN = 4
};
}

namespace DC_DFU_CMD {
enum dc_dfu_cmd_id {
  GET_VERSION = (88 | CONTROL_CMD_READ_BIT),
};
}

enum Satellite1State : uint8_t {
  SAT_DETACHED_STATE,
  SAT_XMOS_CONNECTED_STATE,
  SAT_FLASH_CONNECTED_STATE,
};

class Satellite1 : public Component,
                   public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                         spi::DATA_RATE_1KHZ> {
 public:
  Satellite1State state{SAT_DETACHED_STATE};
  uint8_t xmos_fw_version[5];
  std::string status_string();
  uint8_t connection_attempts{0};

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }
  void loop() override;

  /**
   * @brief Communicates with the XMOS device controller for resource-specific operations.
   *
   * This function facilitates communication with the XMOS device controller, which exposes
   * various resources that can be accessed via SPI commands. The function supports both
   * write and read operations:
   * - **Write Commands:** Data in the `payload` buffer is sent to the specified resource.
   * - **Read Commands:** The function expects a response from the resource, which is written
   *   back into the `payload` buffer.
   *
   * The behavior is determined by the `command` parameter:
   * - Commands with the highest bit set (`0x80`) indicate a **read operation**, and the
   *   device's response will overwrite the `payload` buffer.
   * - Commands without the `0x80` bit perform a **write operation**, sending the `payload`
   *   data to the device without expecting a response in the `payload` buffer.
   *
   * The function also processes status reports from the device and updates the internal
   * status register if applicable.
   *
   * @param resource_id  Identifier for the target resource within the XMOS device controller.
   * @param command      Command specifying the operation (read or write) to be executed. The
   *                     highest bit (`0x80`) determines whether a response is expected.
   * @param payload      Pointer to the buffer containing the data to be sent. For read commands,
   *                     this buffer is updated with the response from the device.
   * @param payload_len  Length of the payload buffer in bytes.
   *
   * @return             A boolean value indicating the success or failure of the operation:
   *                     - `true`: The operation was successful. The payload buffer and/or status
   *                       register have been updated as needed.
   *                     - `false`: The operation failed, possibly due to ignored commands, lack
   *                       of response from the device, or disabled SPI communication.
   *
   * @details
   * - The XMOS device may return status reports during communication. If the response indicates
   *   a status report (resource ID matches `DC_RESOURCE::CNTRL_ID`), the internal status register
   *   is updated.
   */
  bool transfer(uint8_t resource_id, uint8_t command, uint8_t *payload, uint8_t payload_len);

  /**
   * @brief Requests an update to the XMOS device controller's status registers.
   *
   * This function sends a zero-byte command to the XMOS device controller to trigger
   * a status register update. The device responds by sending its current status
   * registers.
   *
   * @return             A boolean value indicating the success or failure of the operation:
   *                     - `true`: The status registers were successfully updated.
   *                     - `false`: The update request failed, potentially due to communication
   *                       issues or ignored commands.
   */
  bool request_status_register_update();

  /**
   * @brief Retrieves the cached value of a specific status register.
   *
   * This function returns the current value of a specific status register from the
   * locally cached `dc_status_register_` buffer. It does not trigger a status
   * update from the XMOS device controller. To ensure the cached values are up to
   * date, call `request_status_register_update` before using this function.
   *
   * @param reg          The identifier of the status register to query as defined in
   *                     `DC_STATUS_REGISTER`.
   *
   * @return             The cached value of the requested status register as an 8-bit
   *                     unsigned integer.
   */
  uint8_t get_dc_status(DC_STATUS_REGISTER::register_id reg) {
    assert(reg < DC_STATUS_REGISTER::REGISTER_LEN);
    return this->dc_status_register_[reg];
  }

  void set_spi_flash_direct_access_mode(bool enable);

  void set_xmos_rst_pin(GPIOPin *xmos_rst_pin) { this->xmos_rst_pin_ = xmos_rst_pin; }
  template<typename F> void add_on_state_callback(F &&callback) {
    this->state_callback_.add(std::forward<F>(callback));
  }

  void xmos_hardware_reset();

 protected:
  bool dfu_get_fw_version_();
  bool check_for_xmos_();
  CallbackManager<void()> state_callback_{};

  uint32_t last_attempt_timestamp_{0};

  uint8_t dc_status_register_[DC_STATUS_REGISTER::REGISTER_LEN];
  bool spi_flash_direct_access_enabled_{false};

  GPIOPin *xmos_rst_pin_{nullptr};
};

class Satellite1SPIService : public Parented<Satellite1> {
 public:
  virtual bool handle_response(uint8_t status, uint8_t res_id, uint8_t cmd, uint8_t *payload, uint8_t payload_len) {
    return false;
  }

 protected:
  uint8_t transfer_byte(uint8_t byte) { return this->parent_->transfer_byte(byte); }
  void enable() { this->parent_->enable(); }
  void disable() { this->parent_->disable(); }
  uint8_t servicer_id_;
};

}  // namespace satellite1
}  // namespace esphome