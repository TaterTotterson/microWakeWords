#include "fusb302b.h"

#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "esp_rom_gpio.h"
#include <driver/gpio.h>
#include <hal/gpio_types.h>

#include "fusb302_defines.h"

namespace esphome {
namespace power_delivery {

static const char *const TAG = "fusb302b";

enum FUSB302_transmit_data_tokens_t {
  TX_TOKEN_TXON = 0xA1,
  TX_TOKEN_SOP1 = 0x12,
  TX_TOKEN_SOP2 = 0x13,
  TX_TOKEN_SOP3 = 0x1B,
  TX_TOKEN_RESET1 = 0x15,
  TX_TOKEN_RESET2 = 0x16,
  TX_TOKEN_PACKSYM = 0x80,
  TX_TOKEN_JAM_CRC = 0xFF,
  TX_TOKEN_EOP = 0x14,
  TX_TOKEN_TXOFF = 0xFE,
};

TaskHandle_t xReaderTaskHandle = NULL;
TaskHandle_t xProcessTaskHandle = NULL;
QueueHandle_t pd_message_queue = NULL;

// ISR handler
void IRAM_ATTR fusb302b_isr_handler(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Notify the task from the ISR, using xTaskNotifyFromISR
  xTaskNotifyFromISR(xReaderTaskHandle, 0x01, eSetBits, &xHigherPriorityTaskWoken);

  // Perform a context switch if needed
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

static void msg_reader_task(void *params) {
  FUSB302B *fusb302b = (FUSB302B *) params;
  uint32_t ulNotificationValue;

  fusb_status regs;

  PDEventInfo event_info;
  PDMsg &msg = event_info.msg;
#if FUSB_DEBUG_PRINT
  printf("MSG READER TASK STARTED \n");
#endif

  fusb302b->enable_auto_crc();
  fusb302b->fusb_reset_();

  while (true) {
    xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY);  // 500  / portTICK_PERIOD_MS portMAX_DELAY
    fusb302b->read_status(regs);
    if (regs.interruptb & FUSB_INTERRUPTB_I_GCRCSENT) {
      event_info.event = PD_EVENT_RECEIVED_MSG;
      while (!(regs.status1 & FUSB_STATUS1_RX_EMPTY)) {
        if (fusb302b->read_message_(msg)) {
          xQueueSend(pd_message_queue, &event_info, 0);
        }
#if FUSB_DEBUG_PRINT
        else {
          printf("Reading failed\n");
        }
#endif
        fusb302b->read_status_register(FUSB_STATUS1, regs.status1);
      }
    }
#if FUSB_DEBUG_PRINT
    if (regs.interrupta & FUSB_INTERRUPTA_I_HARDRST) {
      event_info.event = PD_EVENT_HARD_RESET;
      printf(">>>FUSB_STATUS0A_HARDRST<<<\n");
    } else if (regs.interrupta & FUSB_INTERRUPTA_I_SOFTRST) {
      printf(">>>SOFT_RESET_REQUEST<<<\n");
      event_info.event = PD_EVENT_SOFT_RESET;
    } else if (regs.interrupta & FUSB_INTERRUPTA_I_RETRYFAIL) {
      event_info.event = PD_EVENT_SENDING_MSG_FAILED;
      printf("Message did not get acknowledged.\n");
    }
#endif
  }
  fusb302b->disable_auto_crc();
}

static void trigger_task(void *params) {
  FUSB302B *fusb302b = (FUSB302B *) params;
  uint32_t ulNotificationValue;

  pd_message_queue = xQueueCreate(5, sizeof(PDEventInfo));

  gpio_num_t irq_gpio_pin = static_cast<gpio_num_t>(fusb302b->irq_pin_);

  gpio_config_t io_conf;
  io_conf.pin_bit_mask = (1ULL << irq_gpio_pin);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_NEGEDGE;

  gpio_config(&io_conf);

  // Install ISR service and attach the ISR handler
  gpio_set_intr_type((gpio_num_t) irq_gpio_pin, GPIO_INTR_NEGEDGE);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(irq_gpio_pin, fusb302b_isr_handler, NULL);

  // Create the task that will wait for notifications
  xTaskCreatePinnedToCore(msg_reader_task, "fusb3202b_read_task", 4096, fusb302b, configMAX_PRIORITIES / 2,
                          &xReaderTaskHandle, 1);
  PDEventInfo event_info;

  while (true) {
    if (xQueueReceive(pd_message_queue, &event_info, portMAX_DELAY) == pdTRUE) {
      // delay needed for getting fusb302b ready for receiving i2c again, is this the right place though?
      // vTaskDelay(pdMS_TO_TICKS(1));
      void taskENTER_CRITICAL(void);
      PDMsg &msg = event_info.msg;
      // printf( "PD-Received new message with id: %d (%d, %d) [%u].\n", msg.id, msg.type, msg.num_of_obj, millis());
      fusb302b->handle_message_(msg);
      void taskEXIT_CRITICAL(void);
    }
  }
}

void FUSB302B::setup() {
  this->i2c_lock_ = xSemaphoreCreateBinary();
  if (this->i2c_lock_ == NULL) {
    ESP_LOGD(TAG, "Failed to create semaphore.");
    this->mark_failed();
    return;
  }

  // Release the semaphore initially
  xSemaphoreGive(this->i2c_lock_);

  if (this->check_chip_id()) {
    ESP_LOGD(TAG, "FUSB302 found, initializing...");
  } else {
    ESP_LOGD(TAG, "FUSB302 not found.");
    this->mark_failed();
    return;
  }

  if (!init_fusb_settings_()) {
    ESP_LOGE(TAG, "Couldn't setup FUSB302.");
    this->mark_failed();
    return;
  }
  this->startup_delay_ = millis();
}

void FUSB302B::dump_config() {}

void FUSB302B::loop() {
  this->check_status_();
  if (this->contract_timer_ && millis() - this->contract_timer_ > 1000) {
    this->publish_();
    this->contract_timer_ = 0;
  }
}

bool FUSB302B::cc_line_selection_() {
  /* Measure CC1 */
  this->reg(FUSB_SWITCHES0) = FUSB_SWITCHES0_PDWN_1 | FUSB_SWITCHES0_PDWN_2 | FUSB_SWITCHES0_MEAS_CC1;
  this->reg(FUSB_SWITCHES1) = 0x01 << FUSB_SWITCHES1_SPECREV_SHIFT;
  this->reg(FUSB_MEASURE) = 49;
  delay(5);

  uint8_t cc1 = this->reg(FUSB_STATUS0).get() & FUSB_STATUS0_BC_LVL;
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t tmp = this->reg(FUSB_STATUS0).get() & FUSB_STATUS0_BC_LVL;
    if (cc1 != tmp) {
      return false;
    }
  }

  /* Measure CC2 */
  this->reg(FUSB_SWITCHES0) = FUSB_SWITCHES0_PDWN_1 | FUSB_SWITCHES0_PDWN_2 | FUSB_SWITCHES0_MEAS_CC2;
  delay(5);
  uint8_t cc2 = this->reg(FUSB_STATUS0).get() & FUSB_STATUS0_BC_LVL;
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t tmp = this->reg(FUSB_STATUS0).get() & FUSB_STATUS0_BC_LVL;
    if (cc2 != tmp) {
      return false;
    }
  }

  /* Select the correct CC line for BMC signaling; also enable AUTO_CRC */
  if (cc1 > 0 && cc2 == 0) {
    ESP_LOGD(TAG, "CC select: 1");

    // PWDN1 | PWDN2 | MEAS_CC1
    this->reg(FUSB_SWITCHES0) = 0x07;

    this->reg(FUSB_SWITCHES1) = (FUSB_SWITCHES1_TXCC1 |
                                 // FUSB_SWITCHES1_AUTO_CRC |
                                 (0x01 << FUSB_SWITCHES1_SPECREV_SHIFT));

  } else if (cc1 == 0 && cc2 > 0) {
    ESP_LOGD(TAG, "CC select: 2");
    // PWDN1 | PWDN2 | MEAS_CC2
    this->reg(FUSB_SWITCHES0) = 0x0B;

    this->reg(FUSB_SWITCHES1) = (FUSB_SWITCHES1_TXCC2 |
                                 // FUSB_SWITCHES1_AUTO_CRC |
                                 (0x01 << FUSB_SWITCHES1_SPECREV_SHIFT));
  } else {
    return false;
  }

  return true;
}

void FUSB302B::fusb_reset_() {
  if (xSemaphoreTake(this->i2c_lock_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }
  /* Flush the TX buffer */
  this->reg(FUSB_CONTROL0) = FUSB_CONTROL0_TX_FLUSH;

  /* Flush the RX buffer */
  this->reg(FUSB_CONTROL1) = FUSB_CONTROL1_RX_FLUSH;

  /* Reset the PD logic */
  this->reg(FUSB_RESET) = FUSB_RESET_PD_RESET;

  xSemaphoreGive(this->i2c_lock_);
  this->last_received_msg_id_ = 255;
  PDMsg::msg_cnter_ = 0;
  return;
}

bool FUSB302B::read_status(fusb_status &status) {
  if (xSemaphoreTake(this->i2c_lock_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }
  int err = this->read_register(FUSB_STATUS0A, status.bytes, 7);
  xSemaphoreGive(this->i2c_lock_);
  return err == 0;
}

void FUSB302B::check_status_() {
  switch (this->state_) {
    case FUSB302_STATE_UNATTACHED: {
      if (this->startup_delay_ && millis() - this->startup_delay_ < 2000) {
        return;
      }

      if (xSemaphoreTake(this->i2c_lock_, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
      }
      /* enable internal oscillator */
      this->reg(FUSB_POWER) = PWR_BANDGAP | PWR_RECEIVER | PWR_MEASURE | PWR_INT_OSC;

      bool connected = this->cc_line_selection_();
      xSemaphoreGive(this->i2c_lock_);

      if (!connected) {
        this->state_ = FUSB302_STATE_FAILED;
        return;
      }

      if (this->startup_delay_) {
        ESP_LOGD(TAG, "Statup delay reached!");
        this->startup_delay_ = 0;

        // Create the task that will wait for notifications
        xTaskCreatePinnedToCore(trigger_task, "fusb3202b_task", 4096, this, 18, &xProcessTaskHandle, 1);
        delay(1);
      } else {
        this->enable_auto_crc();
        this->fusb_reset_();
      }

      this->get_src_cap_time_stamp_ = millis();
      this->get_src_cap_retry_count_ = 0;
      this->wait_src_cap_ = true;

      this->state_ = FUSB302_STATE_ATTACHED;
      this->set_state_(PD_STATE_DEFAULT_CONTRACT);
      ESP_LOGD(TAG, "USB-C attached");
      break;
    }
    case FUSB302_STATE_ATTACHED:

      if (this->check_ams()) {
        return;
      }

      if (this->wait_src_cap_) {
        if (get_src_cap_retry_count_ && millis() - get_src_cap_time_stamp_ < 5000) {
          return;
        }
        if (!get_src_cap_retry_count_) {
          get_src_cap_retry_count_++;
          get_src_cap_time_stamp_ = millis();
          return;
        }
        get_src_cap_retry_count_++;
        get_src_cap_time_stamp_ = millis();
        if (get_src_cap_retry_count_ < 2) {
          /* clear interrupts */
          this->read_status();
          this->send_message_(PDMsg(pd_control_msg_type::PD_CNTRL_GET_SOURCE_CAP));
        } else {
          ESP_LOGD(TAG, "send get_source_cap reached max count.");
          if (!this->tried_soft_reset_) {
            this->fusb_reset_();
            this->send_message_(PDMsg(pd_control_msg_type::PD_CNTRL_SOFT_RESET));
            this->get_src_cap_retry_count_ = 2;
            this->tried_soft_reset_ = true;
          } else {
            ESP_LOGD(TAG, "PD-Negotiaton failed. Staying with default 5V supply.");
            this->wait_src_cap_ = false;
            this->active_ams_ = false;
            this->set_state_(PD_STATE_PD_TIMEOUT);
            this->publish_();
          }
        }
      }
      break;
    case FUSB302_STATE_FAILED:
      break;
  }
}

bool FUSB302B::check_chip_id() {
  if (xSemaphoreTake(this->i2c_lock_, pdMS_TO_TICKS(1000)) != pdTRUE) {
    return false;
  }
  uint8_t dev_id = this->reg(FUSB_DEVICE_ID).get();
  xSemaphoreGive(this->i2c_lock_);
  ESP_LOGD(TAG, "reported device id: %d", dev_id);
  return (dev_id == 0x81) || (dev_id == 0x91);
}

bool FUSB302B::enable_auto_crc() {
  if (xSemaphoreTake(this->i2c_lock_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }
  uint8_t sw1 = this->reg(FUSB_SWITCHES1).get();
  this->reg(FUSB_SWITCHES1) = sw1 | FUSB_SWITCHES1_AUTO_CRC;
  xSemaphoreGive(this->i2c_lock_);
  return true;
}

bool FUSB302B::disable_auto_crc() {
  if (xSemaphoreTake(this->i2c_lock_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }
  uint8_t sw1 = this->reg(FUSB_SWITCHES1).get();
  this->reg(FUSB_SWITCHES1) = sw1;
  xSemaphoreGive(this->i2c_lock_);
  return true;
}

bool FUSB302B::read_status_register(uint8_t reg, uint8_t &value) {
  if (xSemaphoreTake(this->i2c_lock_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }
  int err = this->read_register(reg, &value, 1);
  xSemaphoreGive(this->i2c_lock_);
  return err == 0;
}

bool FUSB302B::init_fusb_settings_() {
  if (xSemaphoreTake(this->i2c_lock_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }

  // set all registers back to default
  this->reg(FUSB_RESET) = FUSB_RESET_SW_RES;
  this->fusb_reset_();

  /* Set interrupt masks */
  // this->reg(FUSB_MASK1) = 0xFF; //~FUSB_MASK1_M_CRC_CHK;
  this->reg(FUSB_MASK1) = 0x51;
  this->reg(FUSB_MASKA) = ~(FUSB_MASKA_M_RETRYFAIL | FUSB_MASKA_M_TXSENT | FUSB_MASKA_M_SOFTRST | FUSB_MASKA_M_HARDRST);
  this->reg(FUSB_MASKA) = 0;
  // Mask the I_GCRCSENT interrupt
  this->reg(FUSB_MASKB) = 0;  // FUSB_MASKB_M_GCRCSENT;

  /* disable global interrupt masking*/
  uint8_t cntrl0 = this->reg(FUSB_CONTROL0).get();
  this->reg(FUSB_CONTROL0) = cntrl0 & ~FUSB_CONTROL0_INT_MASK;

  /* Enable automatic retransmission */
  uint8_t cntrl3 = this->reg(FUSB_CONTROL3).get();
  cntrl3 &= ~FUSB_CONTROL3_N_RETRIES_MASK;
  cntrl3 |= (0x03 << FUSB_CONTROL3_N_RETRIES_SHIFT) | FUSB_CONTROL3_AUTO_RETRY;
  this->reg(FUSB_CONTROL3) = cntrl3;

  this->reg(FUSB_POWER) = 0x0F;
  this->fusb_reset_();
  xSemaphoreGive(this->i2c_lock_);
  return true;
}

bool FUSB302B::read_message_(PDMsg &msg) {
  if (xSemaphoreTake(this->i2c_lock_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }

  uint8_t fifo_byte = this->reg(FUSB_FIFOS).get();
  uint8_t ret = 0;

  if ((fifo_byte & FUSB_FIFO_RX_TOKEN_BITS) != FUSB_FIFO_RX_SOP) {
    ret = 1;
  }

  uint16_t header;
  ret |= this->read_register(FUSB_FIFOS, (uint8_t *) &header, 2);
  msg.set_header(header);

  if (msg.num_of_obj > 7) {
    xSemaphoreGive(this->i2c_lock_);
    return false;
  } else if (msg.num_of_obj > 0) {
    ret |= this->read_register(FUSB_FIFOS, (uint8_t *) msg.data_objects, msg.num_of_obj * sizeof(uint32_t));
  }

  /* Read CRC32 only, the PHY already checked it. */
  uint8_t dummy[4];
  ret |= this->read_register(FUSB_FIFOS, dummy, 4);

  xSemaphoreGive(this->i2c_lock_);
  return (ret == 0);
}

bool FUSB302B::send_message_(const PDMsg &msg) {
  if (xSemaphoreTake(this->i2c_lock_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }

  uint8_t buf[40];
  uint8_t *pbuf = buf;

  uint16_t header = msg.get_coded_header();
  uint8_t obj_count = msg.num_of_obj;

  *pbuf++ = (uint8_t) TX_TOKEN_SOP1;
  *pbuf++ = (uint8_t) TX_TOKEN_SOP1;
  *pbuf++ = (uint8_t) TX_TOKEN_SOP1;
  *pbuf++ = (uint8_t) TX_TOKEN_SOP2;
  *pbuf++ = (uint8_t) TX_TOKEN_PACKSYM | ((obj_count << 2) + 2);
  *pbuf++ = header & 0xFF;
  header >>= 8;
  *pbuf++ = header & 0xFF;
  for (uint8_t i = 0; i < obj_count; i++) {
    uint32_t d = msg.data_objects[i];
    *pbuf++ = d & 0xFF;
    d >>= 8;
    *pbuf++ = d & 0xFF;
    d >>= 8;
    *pbuf++ = d & 0xFF;
    d >>= 8;
    *pbuf++ = d & 0xFF;
  }
  *pbuf++ = (uint8_t) TX_TOKEN_JAM_CRC;
  *pbuf++ = (uint8_t) TX_TOKEN_EOP;
  *pbuf++ = (uint8_t) TX_TOKEN_TXOFF;
  *pbuf++ = (uint8_t) TX_TOKEN_TXON;

  int err = this->write_register(FUSB_FIFOS, buf, pbuf - buf);
#if FUSB_DEBUG_PRINT
  if (err != i2c::ERROR_OK) {
    printf("Sending Message (%d) failed err: %d.\n", (int) msg.type, err);
  }
#endif
  // else {
  //    printf("Sent Message (%d) id: %d. [%d] \n", (int) msg.type, msg.id, millis() );
  // }

  // msg.debug_log();
  xSemaphoreGive(this->i2c_lock_);
  return true;
}

}  // namespace power_delivery
}  // namespace esphome