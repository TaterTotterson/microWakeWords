#pragma once

#include <esp_http_server.h>
#include <string>
#include <functional>
#include "ld2410_handler.h"
#include "ld2450_handler.h"

namespace esphome {
namespace satellite1_radar {

static const int RT_NUM_TARGETS = 3;
static const int RT_NUM_GATES = 9;

class RadarTunerServer {
 public:
  void start();
  void stop();
  bool is_running() const { return server_ != nullptr; }

  void set_html_content(const uint8_t *data, unsigned int len) {
    html_gz_ = data;
    html_gz_len_ = len;
  }

  void set_ld2410_handler(LD2410Handler *handler) { ld2410_ = handler; }
  void set_ld2450_handler(LD2450Handler *handler) { ld2450_ = handler; }
  void set_ld2410_apply_callback(std::function<void()> cb) { on_ld2410_apply_ = std::move(cb); }

  void update_target(int index, float x, float y);

  void clear_registrations();

 private:
  httpd_handle_t server_{nullptr};
  LD2410Handler *ld2410_{nullptr};
  LD2450Handler *ld2450_{nullptr};

  const uint8_t *html_gz_{nullptr};
  unsigned int html_gz_len_{0};
  std::function<void()> on_ld2410_apply_;

  struct TargetData {
    float x{0};
    float y{0};
  };
  TargetData targets_[RT_NUM_TARGETS]{};

  static esp_err_t handle_root_(httpd_req_t *req);
  static esp_err_t handle_ld2410_get_config_(httpd_req_t *req);
  static esp_err_t handle_ld2410_patch_config_(httpd_req_t *req);
  static esp_err_t handle_ld2410_apply_(httpd_req_t *req);
  static esp_err_t handle_ld2410_live_(httpd_req_t *req);
  static esp_err_t handle_ld2450_get_config_(httpd_req_t *req);
  static esp_err_t handle_ld2450_patch_config_(httpd_req_t *req);
  static esp_err_t handle_ld2450_live_(httpd_req_t *req);
  static esp_err_t handle_save_(httpd_req_t *req);
  static esp_err_t handle_reboot_(httpd_req_t *req);
};

}  // namespace satellite1_radar
}  // namespace esphome
