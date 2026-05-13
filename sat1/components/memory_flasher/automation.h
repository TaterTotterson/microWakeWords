#pragma once
#include "memory_flasher.h"

#include "esphome/core/automation.h"

namespace esphome {
namespace memory_flasher {

template<typename... Ts> class FlashAction : public Action<Ts...> {
 public:
  FlashAction(MemoryFlasher *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, md5_url)
  TEMPLATABLE_VALUE(std::string, md5)
  TEMPLATABLE_VALUE(std::string, url)

  void play(const Ts &...x) override {
    if (this->md5_url_.has_value()) {
      this->parent_->set_md5_url(this->md5_url_.value(x...));
    }
    if (this->md5_.has_value()) {
      this->parent_->set_md5(this->md5_.value(x...));
    }
    this->parent_->set_url(this->url_.value(x...));

    this->parent_->flash_remote_image();
  }

 protected:
  MemoryFlasher *parent_;
};

template<typename... Ts> class FlashEmbeddedAction : public Action<Ts...>, public Parented<MemoryFlasher> {
 public:
  void play(const Ts &...x) override { this->parent_->flash_embedded_image(); }
};

template<typename... Ts> class EraseMemoryAction : public Action<Ts...>, public Parented<MemoryFlasher> {
 public:
  void play(const Ts &...x) override { this->parent_->erase_memory(); }
};

template<FlasherState State> class FlasherStateTrigger : public Trigger<> {
 public:
  explicit FlasherStateTrigger(MemoryFlasher *xflash) {
    xflash->add_on_state_callback([this, xflash]() {
      if (xflash->state == State)
        this->trigger();
    });
  }
};

class FlashingStartedTrigger : public Trigger<> {
 public:
  explicit FlashingStartedTrigger(MemoryFlasher *xflash) {
    xflash->add_on_state_callback([this, xflash]() {
      if (xflash->state == FLASHER_ERASING && this->last_reported_ != FLASHER_ERASING) {
        this->last_reported_ = FLASHER_ERASING;
        this->trigger();
      } else {
        this->last_reported_ = xflash->state;
      }
    });
  }

 protected:
  FlasherState last_reported_{FLASHER_IDLE};
};

class ErasingDoneTrigger : public Trigger<> {
 public:
  explicit ErasingDoneTrigger(MemoryFlasher *xflash) {
    xflash->add_on_state_callback([this, xflash]() {
      if (xflash->state == FLASHER_SUCCESS_STATE && xflash->requested_action == ACTION_FULL_ERASE) {
        this->trigger();
      }
    });
  }
};

class FlashingProgressUpdateTrigger : public Trigger<> {
 public:
  explicit FlashingProgressUpdateTrigger(MemoryFlasher *xflash) {
    xflash->add_on_state_callback([this, xflash]() {
      if (xflash->state == FLASHER_FLASHING && xflash->flashing_progress != this->last_reported_)
        this->last_reported_ = xflash->flashing_progress;
      this->trigger();
    });
  }

 protected:
  uint8_t last_reported_{0};
};

using FlasherSuccessTrigger = FlasherStateTrigger<FLASHER_SUCCESS_STATE>;
using FlasherFailedTrigger = FlasherStateTrigger<FLASHER_ERROR_STATE>;

template<typename... Ts> class InProgressCondition : public Condition<Ts...>, public Parented<MemoryFlasher> {
 public:
  bool check(const Ts &...x) override { return this->parent_->state != FLASHER_IDLE; }
};

}  // namespace memory_flasher
}  // namespace esphome