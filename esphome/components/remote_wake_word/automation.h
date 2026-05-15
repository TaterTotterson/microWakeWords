#pragma once

#include "remote_wake_word.h"

#ifdef USE_ESP32
namespace esphome {
namespace remote_wake_word {

template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<RemoteWakeWord> {
 public:
  void play(const Ts &...x) override { this->parent_->start(); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<RemoteWakeWord> {
 public:
  void play(const Ts &...x) override { this->parent_->stop(); }
};

template<typename... Ts> class IsRunningCondition : public Condition<Ts...>, public Parented<RemoteWakeWord> {
 public:
  bool check(const Ts &...x) override { return this->parent_->is_running(); }
};

}  // namespace remote_wake_word
}  // namespace esphome
#endif  // USE_ESP32
