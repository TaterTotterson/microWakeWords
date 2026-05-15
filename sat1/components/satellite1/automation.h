#pragma once
#include "satellite1.h"

#include "esphome/core/automation.h"

namespace esphome {
namespace satellite1 {

template<typename... Ts> class XMOSHardwareResetAction : public Action<Ts...>, public Parented<Satellite1> {
 public:
  void play(const Ts &...x) override { this->parent_->xmos_hardware_reset(); }
};

template<Satellite1State State> class Satellite1StateTrigger : public Trigger<> {
 public:
  explicit Satellite1StateTrigger(Satellite1 *sat1) {
    sat1->add_on_state_callback([this, sat1]() {
      if (sat1->state == State)
        this->trigger();
    });
  }
};

using XMOSConnectedStateTrigger = Satellite1StateTrigger<SAT_XMOS_CONNECTED_STATE>;
using FlashConnectedStateTrigger = Satellite1StateTrigger<SAT_FLASH_CONNECTED_STATE>;

class XMOSNoResponseStateTrigger : public Trigger<> {
 public:
  explicit XMOSNoResponseStateTrigger(Satellite1 *sat1) {
    sat1->add_on_state_callback([this, sat1]() {
      if (sat1->state == SAT_DETACHED_STATE && sat1->connection_attempts == MAX_CONNECTION_ATTEMPTS)
        this->trigger();
    });
  }
};

}  // namespace satellite1
}  // namespace esphome