#pragma once

#include "esphome/components/satellite1/satellite1.h"

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace satellite1 {

const uint8_t ECHO_RES_ID = 230;

const uint8_t ECHO_SERVICER_CMD_ECHO_1 = 0;
const uint8_t ECHO_SERVICER_CMD_ECHO_64 = 1;
const uint8_t ECHO_SERVICER_CMD_ECHO_128 = 2;

class SPIErrorRate : public Component, public SatelliteSPIService {
 public:
  void setup() override;
  void loop() override;

  void start_test();
  void stop_test();

  bool send_test_frame();
  bool read_test_frame();
  bool handle_response(uint8_t status, uint8_t res_id, uint8_t cmd, uint8_t *payload, uint8_t payload_len) override;
  void report();

 protected:
  uint8_t resource_ids_[1] = {ECHO_RES_ID};

  uint8_t bytes_per_frame_{128};
  uint8_t echo_cmd_{ECHO_SERVICER_CMD_ECHO_128};

  uint8_t last_sent_[256];

  uint32_t start_time_{0};
  uint32_t frames_received_{0};
  uint32_t ignored_cmds_{0};
  uint32_t incorrect_bytes_{0};
  bool waiting_{false};
};

}  // namespace satellite1
}  // namespace esphome