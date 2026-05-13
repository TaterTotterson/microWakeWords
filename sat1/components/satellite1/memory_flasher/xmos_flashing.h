#pragma once

#include "esphome/components/http_request/http_request.h"
#include "esphome/components/ota/ota_backend.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/md5/md5.h"

#include "esphome/components/memory_flasher/memory_flasher.h"
#include "esphome/components/satellite1/satellite1.h"

namespace esphome {
using namespace memory_flasher;
namespace satellite1 {

class XMOSFlasher : public MemoryFlasher, public Satellite1SPIService {
 public:
  void loop() override;

  bool init_flasher() override;
  bool deinit_flasher() override;
  void dump_flash_info() override;

  void erase_memory() override;
  void flash_remote_image() override;
  void flash_embedded_image() override;

  bool flash_accessible() override {
    this->parent_->set_spi_flash_direct_access_mode(true);
    bool got_id = this->read_JEDECID_();
    this->parent_->set_spi_flash_direct_access_mode(false);
    return got_id;
  }

 protected:
  bool read_JEDECID_();
  bool enable_writing_();
  bool disable_writing_();
  bool chip_erase_();
  bool erase_sector_(int sector);
  bool wait_while_flash_busy_(uint32_t timeout_ms);
  bool read_page_(uint32_t byte_addr, uint8_t *buffer);
  bool write_page_(uint32_t byte_addr, uint8_t *buffer);

  uint8_t manufacturerID_;
  uint8_t memoryTypeID_;
  uint8_t capacityID_;
  int32_t capacity_;
  size_t total_number_of_sectors_;

  bool init_flashing_();
  void deinit_flashing_();
  int flashing_step_();
  int erasing_step_();
  void publish_progress_() override;

  bool http_flash_{false};
  bool embedded_flash_{false};
  FlashImageReader *reader_;
  md5::MD5Digest md5_receive_;

  uint32_t flashing_start_time_{0};
  uint32_t last_published_{0};
  size_t total_sectors_to_erase_{0};
  int current_sector_{-1};
  size_t total_number_of_bytes_{0};
  size_t bytes_remaining_;
  int page_pos_{0};

  uint8_t *reader_buffer_;
  uint8_t *compare_buffer_;
};

}  // namespace satellite1
}  // namespace esphome