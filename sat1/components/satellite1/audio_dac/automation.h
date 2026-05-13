#pragma once

#include "esphome/core/automation.h"
#include "dac_proxy.h"

namespace esphome {

namespace satellite1 {

template<typename... Ts> class ActivateAction : public Action<Ts...>, public Parented<DACProxy> {
 public:
  void play(const Ts &...x) override { this->parent_->activate(); }
};

template<typename... Ts> class ActivateLineOutAction : public Action<Ts...>, public Parented<DACProxy> {
 public:
  void play(const Ts &...x) override { this->parent_->activate_line_out(); }
};

template<typename... Ts> class ActivateSpeakerAction : public Action<Ts...>, public Parented<DACProxy> {
 public:
  void play(const Ts &...x) override { this->parent_->activate_speaker(); }
};

class StateTrigger : public Trigger<> {
 public:
  explicit StateTrigger(DACProxy *proxy) {
    proxy->add_on_state_callback([this]() { this->trigger(); });
  }
};

template<DacOutput output_dac> class DACProxyStateTrigger : public Trigger<> {
 public:
  explicit DACProxyStateTrigger(DACProxy *proxy) {
    proxy->add_on_state_callback([this, proxy]() {
      if (proxy->active_dac == output_dac)
        this->trigger();
    });
  }
};

using SpeakerActivatedTrigger = DACProxyStateTrigger<SPEAKER>;
using LineOutActivatedTrigger = DACProxyStateTrigger<LINE_OUT>;

}  // namespace satellite1
}  // namespace esphome