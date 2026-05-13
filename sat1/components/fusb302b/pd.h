#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#define PD_MAX_NUM_DATA_OBJECTS 7

namespace esphome {
namespace power_delivery {

enum pd_spec_revision_t {
  PD_SPEC_REV_1 = 0,
  PD_SPEC_REV_2 = 1,
  PD_SPEC_REV_3 = 2
  /* 3 Reserved */
};

enum pd_data_msg_type {
  /* 0 Reserved */
  PD_DATA_SOURCE_CAP = 0x01,
  PD_DATA_REQUEST = 0x02,
  PD_DATA_BIST = 0x03,
  PD_DATA_SINK_CAP = 0x04,
  PD_DATA_BATTERY_STATUS = 0x05,
  PD_DATA_ALERT = 0x06,
  PD_DATA_GET_COUNTRY_INFO = 0x07,
  PD_DATA_ENTER_USB = 0x08,
  PD_DATA_EPR_REQUEST = 0x09,
  PD_DATA_EPR_MODE = 0x0A,
  PD_DATA_SOURCE_INFO = 0x0B,
  PD_DATA_REVISION = 0x0C,
  PD_DATA_VENDOR_DEF = 0x0F
};

enum pd_control_msg_type {
  PD_CNTRL_GOODCRC = 0x01,
  PD_CNTRL_GOTOMIN = 0x02,
  PD_CNTRL_ACCEPT = 0x03,
  PD_CNTRL_REJECT = 0x04,
  PD_CNTRL_PING = 0x05,
  PD_CNTRL_PS_RDY = 0x06,
  PD_CNTRL_GET_SOURCE_CAP = 0x07,
  PD_CNTRL_GET_SINK_CAP = 0x08,
  PD_CNTRL_DR_SWAP = 0x09,
  PD_CNTRL_PR_SWAP = 0x0A,
  PD_CNTRL_VCONN_SWAP = 0x0B,
  PD_CNTRL_WAIT = 0x0C,
  PD_CNTRL_SOFT_RESET = 0x0D,
  PD_CNTRL_NOT_SUPPORTED = 0x10,
  PD_CNTRL_GET_SOURCE_CAP_EXTENDED = 0x11,
  PD_CNTRL_GET_STATUS = 0x12,
  PD_CNTRL_FR_SWAP = 0x13,
  PD_CNTRL_GET_PPS_STATUS = 0x14,
  PD_CNTRL_GET_COUNTRY_CODES = 0x15,
  PD_CNTRL_GET_SINK_CAP_EXTENDED = 0x16,
  PD_CNTRL_GET_SOURCE_INFO = 0x17,
  PD_CNTRL_GET_REVISION = 0x18
};

enum pd_power_data_obj_type { /* Power data object type */
                              PD_PDO_TYPE_FIXED_SUPPLY = 0,
                              PD_PDO_TYPE_BATTERY = 1,
                              PD_PDO_TYPE_VARIABLE_SUPPLY = 2,
                              PD_PDO_TYPE_AUGMENTED_PDO = 3 /* USB PD 3.0 */
};

enum PowerDeliveryState : uint8_t {
  PD_STATE_DISCONNECTED,
  PD_STATE_PD_TIMEOUT,
  PD_STATE_DEFAULT_CONTRACT,
  PD_STATE_TRANSITION,
  PD_STATE_EXPLICIT_SPR_CONTRACT,
  PD_STATE_EXPLICIT_EPR_CONTRACT,
  PD_STATE_ERROR
};

enum PowerDeliveryEvent : uint8_t {
  PD_EVENT_ATTACHED,
  PD_EVENT_DETACHED,
  PD_EVENT_RECEIVED_MSG,
  PD_EVENT_SENDING_MSG_FAILED,
  PD_EVENT_SOFT_RESET,
  PD_EVENT_HARD_RESET
};

struct pd_contract_t {
  enum pd_power_data_obj_type type;
  uint16_t min_v; /* Voltage in 50mV units */
  uint16_t max_v; /* Voltage in 50mV units */
  uint16_t max_i; /* Current in 10mA units */
  uint16_t max_p; /* Power in 250mW units */

  bool operator==(const pd_contract_t &other) const {
    return max_v == other.max_v && max_i == other.max_i && type == other.type;
  }
  bool operator!=(const pd_contract_t &other) const { return !(*this == other); }
};

class PDMsg {
 public:
  PDMsg() = default;
  PDMsg(uint16_t header);
  PDMsg(pd_control_msg_type cntrl_msg_type);
  PDMsg(pd_control_msg_type cntrl_msg_type, uint8_t msg_id);
  PDMsg(pd_data_msg_type data_msg_type, const uint32_t *objects, uint8_t len);

  uint16_t get_coded_header() const;
  bool set_header(uint16_t header);

  uint8_t type;
  pd_spec_revision_t spec_rev;
  uint8_t id;
  uint8_t num_of_obj;
  bool extended;
  uint32_t data_objects[PD_MAX_NUM_DATA_OBJECTS];

  void debug_log() const;

  // protected:
  static uint8_t msg_cnter_;
  static pd_spec_revision_t spec_rev_;
};

class PDEventInfo {
 public:
  PowerDeliveryEvent event;
  PDMsg msg;
};

typedef uint32_t pd_pdo_t;

class PowerDelivery {
 public:
  PowerDeliveryState state{PD_STATE_DISCONNECTED};
  int contract_voltage{5};
  float contract_max_current{.5f};
  std::string contract{"0.5A (max) @ 5V"};
  PowerDeliveryState prev_state_{PD_STATE_DISCONNECTED};

  bool request_voltage(int voltage);

  virtual bool send_message_(const PDMsg &msg) = 0;
  virtual bool read_message_(PDMsg &msg) = 0;

  PDMsg create_fallback_request_message() const;
  bool handle_message_(const PDMsg &msg);

  void set_request_voltage(int voltage) { this->request_voltage_ = voltage; }
  std::string get_contract_string(pd_contract_t contract) const;

  template<typename F> void add_on_state_callback(F &&callback) {
    this->state_callback_.add(std::forward<F>(callback));
  }

  void set_ams(bool ams);
  bool check_ams();

 protected:
  uint32_t active_ams_timer_{0};
  bool active_ams_{false};
  void protocol_reset_();

  uint8_t last_received_msg_id_{255};

  bool handle_data_message_(const PDMsg &msg);
  bool handle_cntrl_message_(const PDMsg &msg);

  pd_contract_t parse_power_info_(pd_pdo_t &pdo) const;
  bool respond_to_src_cap_msg_(const PDMsg &msg);

  void set_contract_(pd_contract_t contract);
  pd_contract_t requested_contract_;
  pd_contract_t accepted_contract_;
  pd_contract_t previous_contract_;
  uint32_t contract_timer_{0};

  void set_state_(PowerDeliveryState new_state) {
    this->prev_state_ = this->state;
    this->state = new_state;
  }

  virtual void publish_() {}

  pd_spec_revision_t spec_revision_{pd_spec_revision_t::PD_SPEC_REV_2};

  bool wait_src_cap_{true};
  bool tried_soft_reset_{false};
  int get_src_cap_retry_count_{0};
  uint32_t get_src_cap_time_stamp_;

  int request_voltage_{5};

  CallbackManager<void()> state_callback_{};
};

inline PDMsg build_get_sink_cap_response() {
  /* Reference: 6.4.1.2.3 Sink Fixed Supply Power Data Object */
  constexpr uint32_t data =
      (((uint32_t) 500 << 0) |  /* B9...0     Operational Current in 10mA units */
       ((uint32_t) 100 << 10) | /* B19...10   Voltage in 50mV units */
                                //((uint32_t)  1 << 25) |                       /* B25        Dual-Role Data */
       ((uint32_t) 1 << 26) |   /* B26        USB Communications Capable */
       //((uint32_t)  1 << 27) |                       /* B27        Unconstrained Power support */
       //((uint32_t)  1 << 28) |                       /* B28        Higher Capability */
       //((uint32_t)  1 << 29) |                       /* B29        Dual Role Power */
       ((uint32_t) PD_PDO_TYPE_FIXED_SUPPLY << 30) /* B31...30   Fixed supply */
      );
  return PDMsg(pd_data_msg_type::PD_DATA_SINK_CAP, &data, 1);
}

inline bool PowerDelivery::handle_message_(const PDMsg &msg) {
  if (msg.num_of_obj == 0) {
    if (msg.type == PD_CNTRL_GOODCRC) {
      PDMsg::msg_cnter_++;
      return true;
    }
    return this->handle_cntrl_message_(msg);
  } else {
    return this->handle_data_message_(msg);
  }
}

inline bool PowerDelivery::handle_data_message_(const PDMsg &msg) {
  if (msg.id == this->last_received_msg_id_) {
    return false;
  }
  this->last_received_msg_id_ = msg.id;
  switch (msg.type) {
    case PD_DATA_SOURCE_CAP:
      this->set_ams(true);
      this->wait_src_cap_ = false;
#if 0        
        if( PDMsg::spec_rev_ == pd_spec_revision_t::PD_SPEC_REV_1 ){
          if( msg.spec_rev >= pd_spec_revision_t::PD_SPEC_REV_3 ){
            PDMsg::spec_rev_ = pd_spec_revision_t::PD_SPEC_REV_3;
          } else {
            PDMsg::spec_rev_ = pd_spec_revision_t::PD_SPEC_REV_2;
          }
        }
        PDMsg::spec_rev_ = msg.spec_rev;
#endif
      this->respond_to_src_cap_msg_(msg);
      break;
    case PD_DATA_ALERT:
      break;
    default:
      break;
  }
  return true;
}

inline bool PowerDelivery::handle_cntrl_message_(const PDMsg &msg) {
  if (msg.id == this->last_received_msg_id_) {
    return false;
  }
  this->last_received_msg_id_ = msg.id;
  switch (msg.type) {
    case PD_CNTRL_GOODCRC:
      break;
    case PD_CNTRL_ACCEPT:
      if (this->active_ams_) {
        if (this->requested_contract_ != this->accepted_contract_) {
          this->set_state_(PD_STATE_TRANSITION);
        }
        this->set_contract_(this->requested_contract_);
      }
      break;
    case PD_CNTRL_PS_RDY:
      this->set_ams(false);
      this->set_state_(PD_STATE_EXPLICIT_SPR_CONTRACT);
      break;
    case PD_CNTRL_SOFT_RESET:
      this->send_message_(PDMsg(pd_control_msg_type::PD_CNTRL_ACCEPT, 0));
      this->set_state_(PD_STATE_DEFAULT_CONTRACT);
      PDMsg::msg_cnter_ = 0;
      break;
    case PD_CNTRL_GET_SINK_CAP:
      this->send_message_(build_get_sink_cap_response());
      break;
    default:
      this->send_message_(PDMsg(pd_control_msg_type::PD_CNTRL_NOT_SUPPORTED));
      break;
      break;
  }
  return true;
}

}  // namespace power_delivery
}  // namespace esphome