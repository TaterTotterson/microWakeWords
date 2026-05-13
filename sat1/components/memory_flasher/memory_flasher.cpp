#include "memory_flasher.h"

#include "esphome/core/log.h"
#include "esphome/components/md5/md5.h"

namespace esphome {
namespace memory_flasher {

static const char *const TAG = "memory_flasher";

void MemoryFlasher::dump_config() {
  if (this->has_image_embedded()) {
    ESP_LOGCONFIG(TAG, "Embedded Image:");
    ImageVersion v = this->embedded_image_.version;
    ESP_LOGCONFIG(TAG, "    Version: %d.%d.%d", v.major, v.minor, v.patch);
  }
}

bool MemoryFlasher::http_get_md5_() {
  if (this->md5_url_.empty()) {
    return false;
  }

  ESP_LOGI(TAG, "Connecting to: %s", this->md5_url_.c_str());
  auto container = this->http_request_->get(this->md5_url_);
  if (container == nullptr) {
    ESP_LOGE(TAG, "Failed to connect to MD5 URL");
    return false;
  }
  size_t length = container->content_length;
  if (length == 0) {
    container->end();
    return false;
  }
  if (length < MD5_SIZE) {
    ESP_LOGE(TAG, "MD5 file must be %u bytes; %u bytes reported by HTTP server. Aborting", MD5_SIZE, length);
    container->end();
    return false;
  }

  this->md5_expected_.resize(MD5_SIZE);
  int read_len = 0;
  while (container->get_bytes_read() < MD5_SIZE) {
    read_len = container->read((uint8_t *) this->md5_expected_.data(), MD5_SIZE);
    App.feed_wdt();
    yield();
  }
  container->end();

  ESP_LOGV(TAG, "Read len: %u, MD5 expected: %u", read_len, MD5_SIZE);
  return read_len == MD5_SIZE;
}

bool MemoryFlasher::validate_url_(const std::string &url) {
  // t.b.d.
  return true;
}

void MemoryFlasher::set_url(const std::string &url) {
  if (!this->validate_url_(url)) {
    this->url_.clear();  // URL was not valid; prevent flashing until it is
    return;
  }
  this->url_ = url;
}

void MemoryFlasher::set_md5_url(const std::string &url) {
  if (!this->validate_url_(url)) {
    this->md5_url_.clear();  // URL was not valid; prevent flashing until it is
    return;
  }
  this->md5_url_ = url;
  this->md5_expected_.clear();  // to be retrieved later
}

bool HttpImageReader::init_reader() {
  auto url_with_auth = this->url_;
  if (url_with_auth.empty() || this->http_request_ == nullptr) {
    return false;
  }

  ESP_LOGVV(TAG, "url_with_auth: %s", url_with_auth.c_str());
  ESP_LOGI(TAG, "Connecting to: %s", this->url_.c_str());

  this->container_ = this->http_request_->get(url_with_auth);
  if (this->container_ == nullptr) {
    return false;
  }
  return true;
}

bool HttpImageReader::deinit_reader() {
  if (this->container_ != nullptr) {
    this->container_->end();
  }
  return true;
}

int HttpImageReader::read_image_block(uint8_t *buffer, size_t block_size) {
  int bytes_read = this->container_->read(buffer, block_size);
  ESP_LOGVV(TAG, "bytes_read_ = %u, body_length_ = %u, bufsize = %i", this->container_->get_bytes_read(),
            this->container_->content_length, bytes_read);
  return bytes_read;
}

}  // namespace memory_flasher
}  // namespace esphome