#pragma once

#include "esphome/components/http_request/http_request.h"

namespace esphome {
namespace memory_flasher {

static const uint8_t MD5_SIZE = 32;

enum FlashImagePreRelease : uint8_t {
  PRE_RELEASE_NONE = 0,
  PRE_RELEASE_ALPHA = 1,
  PRE_RELEASE_BETA = 2,
  PRE_RELEASE_RC = 3,
  PRE_RELEASE_DEV = 4
};

using ImageVersion = union version_union {
  uint8_t bytes[5];
  struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    FlashImagePreRelease pre_release;
    uint8_t counter;
  };
};

struct FlashImage {
  const uint8_t *data{nullptr};
  size_t length{0};
  std::string md5;
  ImageVersion version;
};

class FlashImageReader {
 public:
  virtual ~FlashImageReader() {}
  virtual bool init_reader() { return true; }
  virtual bool deinit_reader() { return true; }

  virtual size_t get_image_size() = 0;
  virtual int read_image_block(uint8_t *buffer, size_t bock_size) = 0;
};

class HttpImageReader : public FlashImageReader {
 public:
  HttpImageReader(http_request::HttpRequestComponent *http_request, std::string url)
      : http_request_(http_request), url_(url) {}
  bool init_reader() override;
  bool deinit_reader() override;

  size_t get_image_size() override {
    if (this->container_ == nullptr) {
      return 0;
    }
    return this->container_->content_length;
  }
  int read_image_block(uint8_t *buffer, size_t block_size) override;

 protected:
  http_request::HttpRequestComponent *http_request_{nullptr};
  std::shared_ptr<esphome::http_request::HttpContainer> container_{nullptr};
  std::string url_{};
};

class EmbeddedImageReader : public FlashImageReader {
 public:
  EmbeddedImageReader(FlashImage img) : image_(img) {}
  bool init_reader() override {
    this->read_pos_ = 0;
    return true;
  }
  size_t get_image_size() override { return this->image_.length; }

  int read_image_block(uint8_t *buffer, size_t block_size) override {
    size_t to_read = ((this->image_.length - read_pos_) >= block_size) ? block_size : this->image_.length - read_pos_;
    memcpy(buffer, this->image_.data + read_pos_, to_read);
    this->read_pos_ += to_read;
    return to_read;
  }

 protected:
  FlashImage image_;
  size_t read_pos_ = 0;
};

enum FlasherAction : uint8_t { ACTION_FULL_ERASE, ACTION_FLASH_REMOTE_IMAGE, ACTION_FLASH_EMBEDDED_IMAGE };

enum FlasherError : uint8_t {
  FLASHER_OK,
  INIT_READER_ERROR,
  INIT_FLASH_ERROR,
  WRITE_TO_FLASH_ERROR,
  MD5_MISMATCH_ERROR,

  MD5_INVALID,
  BAD_URL,
  CONNECTION_ERROR,

  NO_EMBEDDED_IMAGE_ERROR
};

enum FlasherState : uint8_t {
  FLASHER_IDLE,
  FLASHER_RECEIVING_IMG_INFO,
  FLASHER_INITIALIZING,
  FLASHER_ERASING,
  FLASHER_FLASHING,
  FLASHER_SUCCESS_STATE,
  FLASHER_ERROR_STATE
};

class MemoryFlasher : public Component {
 public:
  uint8_t flashing_progress{0};
  FlasherState state{FLASHER_IDLE};
  FlasherError error_code{FLASHER_OK};
  FlasherAction requested_action;

  virtual void dump_config() override;

  virtual bool init_flasher() { return true; }
  virtual bool deinit_flasher() { return true; }
  virtual void dump_flash_info() {}

  virtual void erase_memory() {}
  virtual void flash_remote_image() {}
  virtual void flash_embedded_image() {}

  virtual bool flash_accessible() { return false; }
  bool has_image_embedded() { return this->embedded_image_.length > 0; }

  bool match_embedded(uint8_t to_compare[5]) { return memcmp(this->embedded_image_.version.bytes, to_compare, 5) == 0; }

  void set_embedded_image(const uint8_t *pgm_pointer, size_t length, std::string expected_md5,
                          const char version_bytes[5]) {
    this->embedded_image_.data = pgm_pointer;
    this->embedded_image_.length = length;
    this->embedded_image_.md5 = expected_md5;
    memcpy(this->embedded_image_.version.bytes, version_bytes, 5);
  }

  void set_http_request_component(http_request::HttpRequestComponent *http_request) {
    this->http_request_ = http_request;
  }

  void set_md5_url(const std::string &md5_url);
  void set_md5(const std::string &md5) { this->md5_expected_ = md5; }
  void set_url(const std::string &url);
  template<typename F> void add_on_state_callback(F &&callback) {
    this->state_callback_.add(std::forward<F>(callback));
  }
  void publish() { this->state_callback_.call(); }

 protected:
  virtual void publish_progress_() {}
  CallbackManager<void()> state_callback_{};

  /* flashing embedded image*/
  FlashImage embedded_image_;

  /* flashing remote image */
  bool http_get_md5_();
  bool validate_url_(const std::string &url);
  void cleanup_(const std::shared_ptr<http_request::HttpContainer> &container);

  http_request::HttpRequestComponent *http_request_;

  std::string md5_computed_{};
  std::string md5_expected_{};
  std::string md5_url_{};
  std::string password_{};
  std::string username_{};
  std::string url_{};
};

}  // namespace memory_flasher
}  // namespace esphome