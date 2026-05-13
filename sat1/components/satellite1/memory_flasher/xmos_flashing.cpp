#include "xmos_flashing.h"
#include "esphome/core/log.h"

#include <endian.h>

namespace esphome {
namespace satellite1 {

static const char *const TAG = "xmos_flasher";

static const size_t FLASH_PAGE_SIZE = 256;
static const size_t FLASH_SECTOR_SIZE = 4096;
constexpr size_t FLASH_TOTAL_NUMBER_OF_SECTORS = 8388608 / FLASH_SECTOR_SIZE;

void XMOSFlasher::loop() {
  switch (this->state) {
    case FLASHER_IDLE:
      break;

    case FLASHER_INITIALIZING:
      if (this->init_flashing_()) {
        this->state = FLASHER_ERASING;
      } else {
        this->deinit_flashing_();
        this->state = FLASHER_ERROR_STATE;
      }
      break;

    case FLASHER_ERASING: {
      int remaining = this->erasing_step_();
      this->publish_progress_();
      if (remaining == 0 && this->requested_action == ACTION_FULL_ERASE) {
        this->deinit_flashing_();
        this->state = FLASHER_SUCCESS_STATE;
      } else if (remaining == 0) {
        this->state = FLASHER_FLASHING;
      } else if (remaining < 0) {
        this->state = FLASHER_ERROR_STATE;
      }
      break;
    }

    case FLASHER_FLASHING: {
      int remaining = this->flashing_step_();
      this->publish_progress_();
      if (remaining == 0) {
        this->deinit_flashing_();
        this->state = FLASHER_SUCCESS_STATE;
      } else if (remaining < 0) {
        this->deinit_flashing_();
        this->state = FLASHER_ERROR_STATE;
      }
      break;
    }

    case FLASHER_SUCCESS_STATE:
      this->publish();
      this->state = FLASHER_IDLE;
      break;

    case FLASHER_ERROR_STATE:
      this->publish();
      this->state = FLASHER_IDLE;
      break;

    default:
      break;
  }
}

void XMOSFlasher::publish_progress_() {
  uint32_t now = millis();

  if ((now - this->last_published_) > 1000) {
    if (this->requested_action == ACTION_FULL_ERASE) {
      this->flashing_progress = this->current_sector_ * 100 / this->total_sectors_to_erase_;
    } else {
      this->flashing_progress = ((this->current_sector_ + this->total_number_of_bytes_ - this->bytes_remaining_) * 100 /
                                 (this->total_number_of_bytes_ + this->total_sectors_to_erase_));
    }
    this->last_published_ = now;
    ESP_LOGD(TAG, "Progress: %d%%", this->flashing_progress);
    this->publish();
  }
}

bool XMOSFlasher::init_flasher() {
  ESP_LOGD(TAG, "Setting up XMOS flasher...");
  this->parent_->set_spi_flash_direct_access_mode(true);
  this->read_JEDECID_();
  this->dump_flash_info();
  this->total_number_of_sectors_ = FLASH_TOTAL_NUMBER_OF_SECTORS;
  return true;
}

bool XMOSFlasher::deinit_flasher() {
  ESP_LOGD(TAG, "Stopping XMOS flasher...");
  this->parent_->set_spi_flash_direct_access_mode(false);
  return true;
}

void XMOSFlasher::dump_flash_info() {
  ESP_LOGCONFIG(TAG, "Satellite1-Flasher:");
  ESP_LOGCONFIG(TAG, "	JEDEC-manufacturerID %hhu", this->manufacturerID_);
  ESP_LOGCONFIG(TAG, "	JEDEC-memoryTypeID %hhu", this->memoryTypeID_);
  ESP_LOGCONFIG(TAG, "	JEDEC-capacityID %hhu", this->capacityID_);
  ESP_LOGCONFIG(TAG, "	JEDEC-capacityID %hhu", this->capacityID_);
  ESP_LOGCONFIG(TAG, "	JEDEC-capacity: %hhu", 1 << this->capacityID_);
}

void XMOSFlasher::erase_memory() {
  if (this->state != FLASHER_IDLE) {
    ESP_LOGE(TAG, "XMOS flasher is busy, can't inititate erasing");
    return;
  }

  this->requested_action = ACTION_FULL_ERASE;
  this->state = FLASHER_INITIALIZING;
}

void XMOSFlasher::flash_remote_image() {
  if (this->state != FLASHER_IDLE) {
    ESP_LOGE(TAG, "XMOS flasher is busy, can't initiate new flash");
    return;
  }

  if (this->url_.empty()) {
    ESP_LOGE(TAG, "URL not set; cannot start flashing");
    this->error_code = BAD_URL;
    this->state = FLASHER_ERROR_STATE;
    return;
  }

  if (this->md5_expected_.empty() && !this->http_get_md5_()) {
    ESP_LOGE(TAG, "Couldn't receive expected md5 sum.");
    this->error_code = MD5_INVALID;
    this->state = FLASHER_ERROR_STATE;
    return;
  }

  this->requested_action = ACTION_FLASH_REMOTE_IMAGE;
  this->state = FLASHER_INITIALIZING;
}

void XMOSFlasher::flash_embedded_image() {
  if (this->state != FLASHER_IDLE) {
    ESP_LOGE(TAG, "XMOS flasher is busy, can't inititate new flash");
    return;
  }

  if (this->embedded_image_.length == 0) {
    ESP_LOGE(TAG, "Didn't find embedded image!");
    this->error_code = NO_EMBEDDED_IMAGE_ERROR;
    this->state = FLASHER_ERROR_STATE;
    return;
  }

  this->md5_expected_ = this->embedded_image_.md5;
  this->requested_action = ACTION_FLASH_EMBEDDED_IMAGE;
  this->state = FLASHER_INITIALIZING;
}

bool XMOSFlasher::read_JEDECID_() {
  uint8_t manufacturer = 0;
  uint8_t memoryType = 0;
  uint8_t capcacity = 0;
  this->enable();
  this->transfer_byte(0x9F);
  manufacturer = this->transfer_byte(0);
  memoryType = this->transfer_byte(0);
  capcacity = this->transfer_byte(0);
  this->disable();

  if (manufacturer && memoryType && capcacity) {
    this->manufacturerID_ = manufacturer;
    this->memoryTypeID_ = memoryType;
    this->capacityID_ = capcacity;
    return true;
  }
  return false;
}

bool XMOSFlasher::wait_while_flash_busy_(uint32_t timeout_ms) {
  int32_t timeout_invoke = millis();
  const uint8_t WEL = 2;
  const uint8_t BUSY = 1;

  while ((millis() - timeout_invoke) < timeout_ms) {
    this->enable();
    this->transfer_byte(0x05);
    uint8_t status = this->transfer_byte(0x00);
    this->disable();
    if ((status & BUSY) == 0) {
      return true;
    }
  }
  return false;
}

bool XMOSFlasher::enable_writing_() {
  // enable writing
  this->enable();
  this->transfer_byte(0x06);
  this->disable();

  this->enable();
  this->transfer_byte(0x05);
  uint8_t status = this->transfer_byte(0x00);
  this->disable();
  const uint8_t WEL = 2;
  if (!(status & WEL)) {
    return false;
  }
  return true;
}

bool XMOSFlasher::disable_writing_() {
  // disable writing
  this->enable();
  this->transfer_byte(0x04);
  this->disable();
  return true;
}

bool XMOSFlasher::erase_sector_(int sector) {
  // erase 4kB sector
  assert(FLASH_SECTOR_SIZE == 4096);
  uint32_t u32 = htole32(sector * FLASH_SECTOR_SIZE);
  uint8_t *u8_ptr = (uint8_t *) &u32;

  if (!this->enable_writing_()) {
    return false;
  }

  this->enable();
  this->transfer_byte(0x20);
  this->transfer_byte(*(u8_ptr + 2));
  this->transfer_byte(*(u8_ptr + 1));
  this->transfer_byte(*(u8_ptr));
  this->disable();

  // this->disable_writing_();
  return true;
}

bool XMOSFlasher::chip_erase_() {
  if (!this->enable_writing_()) {
    return false;
  }

  this->enable();
  this->transfer_byte(0xc7);
  this->disable();

  // this->disable_writing_();
  return true;
}

bool XMOSFlasher::write_page_(uint32_t byte_addr, uint8_t *buffer) {
  if ((byte_addr & (FLASH_PAGE_SIZE - 1)) != 0) {
    ESP_LOGE(TAG, "Address needs to be page aligned (%d).", FLASH_PAGE_SIZE);
    return false;
  }
  if (!this->enable_writing_()) {
    ESP_LOGE(TAG, "Couldn't enable writing");
    return false;
  }

  uint32_t u32 = htole32(byte_addr);
  uint8_t *u8_ptr = (uint8_t *) &u32;
  this->enable();
  this->transfer_byte(0x02);
  this->transfer_byte(*(u8_ptr + 2));
  this->transfer_byte(*(u8_ptr + 1));
  this->transfer_byte(*(u8_ptr));
  for (int pos = 0; pos < FLASH_PAGE_SIZE; pos++) {
    this->transfer_byte(*(buffer + pos));
  }
  this->disable();

  if (!this->wait_while_flash_busy_(15)) {
    ESP_LOGE(TAG, "Writing page timeout");
    return false;
  }
  this->disable_writing_();
  return true;
}

bool XMOSFlasher::read_page_(uint32_t byte_addr, uint8_t *buffer) {
  if ((byte_addr & (FLASH_PAGE_SIZE - 1)) != 0) {
    return false;
  }
  uint32_t u32 = htole32(byte_addr);
  uint8_t *u8_ptr = (uint8_t *) &u32;
  this->enable();
  this->transfer_byte(0x0B);
  this->transfer_byte(*(u8_ptr + 2));
  this->transfer_byte(*(u8_ptr + 1));
  this->transfer_byte(*(u8_ptr));
  this->transfer_byte(0x00);
  for (int pos = 0; pos < FLASH_PAGE_SIZE; pos++) {
    *(buffer + pos) = this->transfer_byte(0x00);
  }
  this->disable();
  return true;
}

bool XMOSFlasher::init_flashing_() {
  if (!this->init_flasher()) {
    this->error_code = INIT_FLASH_ERROR;
    return false;
  }

  this->flashing_start_time_ = millis();

  switch (this->requested_action) {
    case ACTION_FLASH_EMBEDDED_IMAGE:
      this->reader_ = new EmbeddedImageReader(this->embedded_image_);
      break;
    case ACTION_FLASH_REMOTE_IMAGE:
      this->reader_ = new HttpImageReader(this->http_request_, this->url_);
      break;
    case ACTION_FULL_ERASE:
      this->total_sectors_to_erase_ = this->total_number_of_sectors_;
      this->current_sector_ = -1;
      return true;
  };

  if (!this->reader_->init_reader()) {
    this->error_code = INIT_READER_ERROR;
    return false;
  }

  this->reader_buffer_ = (uint8_t *) malloc(FLASH_PAGE_SIZE);
  if (this->reader_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Couldn't allocate memory");
    this->error_code = INIT_FLASH_ERROR;
    return false;
  }

  this->compare_buffer_ = (uint8_t *) malloc(FLASH_PAGE_SIZE);
  if (this->compare_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Couldn't allocate memory");
    this->error_code = INIT_FLASH_ERROR;
    return false;
  }

  ESP_LOGD(TAG, "MD5 expected: %s", this->md5_expected_.c_str());

  this->flashing_progress = 0;
  this->md5_receive_.init();

  size_t size_in_bytes = this->reader_->get_image_size();
  size_t size_in_pages = ((size_in_bytes + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE);
  size_t size_in_sectors = ((size_in_pages * FLASH_PAGE_SIZE + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE);

  this->total_number_of_bytes_ = size_in_bytes;
  this->bytes_remaining_ = size_in_bytes;
  this->page_pos_ = 0;

  this->total_sectors_to_erase_ = size_in_sectors;
  this->current_sector_ = -1;

  return true;
}

void XMOSFlasher::deinit_flashing_() {
  if (this->reader_buffer_) {
    free(this->reader_buffer_);
    this->reader_buffer_ = nullptr;
  }

  if (this->compare_buffer_) {
    free(this->compare_buffer_);
    this->compare_buffer_ = nullptr;
  }

  if (this->reader_) {
    this->reader_->deinit_reader();
    delete this->reader_;
    this->reader_ = nullptr;
  }

  this->md5_computed_.clear();
  this->md5_expected_.clear();

  delay(5);
  this->deinit_flasher();
}

int XMOSFlasher::erasing_step_() {
  if (!this->wait_while_flash_busy_(1)) {
    return this->total_sectors_to_erase_ - this->current_sector_;
  }

  this->current_sector_++;
  if (this->current_sector_ < this->total_sectors_to_erase_) {
    if (!this->erase_sector_(this->current_sector_)) {
      this->error_code = WRITE_TO_FLASH_ERROR;
      return -1;
    }
  }

  return this->total_sectors_to_erase_ - this->current_sector_;
}

int XMOSFlasher::flashing_step_() {
  // read a maximum of chunk_size bytes into buf. (real read size returned)
  int bytes_read = this->reader_->read_image_block(this->reader_buffer_, FLASH_PAGE_SIZE);
  if (bytes_read < 0) {
    ESP_LOGE(TAG, "Stream closed");
    this->error_code = CONNECTION_ERROR;
    return -1;
  }

  this->md5_receive_.add(this->reader_buffer_, bytes_read);
  this->bytes_remaining_ -= bytes_read;
  if (bytes_read != FLASH_PAGE_SIZE) {
    if (this->bytes_remaining_ != 0) {
      this->error_code = CONNECTION_ERROR;
      return -1;
    }
    // it's the last page to flash
    // fill it up with zeros
    memset(this->reader_buffer_ + bytes_read, 0, FLASH_PAGE_SIZE - bytes_read);
  }

  int page_pos = this->page_pos_;
  if (!this->write_page_(page_pos, this->reader_buffer_)) {
    ESP_LOGE(TAG, "Error while writing page %d, retrying...", page_pos);
  }

  // read back the page that has just been written
  this->read_page_(page_pos, this->compare_buffer_);

  if (memcmp(this->reader_buffer_, this->compare_buffer_, FLASH_PAGE_SIZE) != 0) {
    // not equal, give it a second try
    if (!this->write_page_(page_pos, this->reader_buffer_)) {
      ESP_LOGE(TAG, "Error while writing page %d, giving up...", page_pos);
      this->error_code = WRITE_TO_FLASH_ERROR;
      return -1;
    }

    this->read_page_(page_pos, this->compare_buffer_);
    if (memcmp(this->reader_buffer_, this->compare_buffer_, FLASH_PAGE_SIZE) != 0) {
      ESP_LOGE(TAG, "Read page mismatch, page addr: %d", page_pos);
      this->error_code = WRITE_TO_FLASH_ERROR;
      return -1;
    }
  }

  this->page_pos_ += FLASH_PAGE_SIZE;

  if (this->bytes_remaining_ == 0) {
    std::unique_ptr<char[]> md5_receive_str(new char[33]);
    this->md5_receive_.calculate();
    this->md5_receive_.get_hex(md5_receive_str.get());
    this->md5_computed_ = md5_receive_str.get();

    if (strncmp(this->md5_computed_.c_str(), this->md5_expected_.c_str(), MD5_SIZE) != 0) {
      ESP_LOGE(TAG, "MD5 computed: %s - Aborting due to MD5 mismatch", this->md5_computed_.c_str());
      this->error_code = MD5_MISMATCH_ERROR;
      return -1;
    } else {
      ESP_LOGD(TAG, "MD5 computed: %s - Matches!", this->md5_computed_.c_str());
    }
  }

  return this->bytes_remaining_;
}

}  // namespace satellite1
}  // namespace esphome