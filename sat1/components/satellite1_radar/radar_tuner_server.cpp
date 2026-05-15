#include "radar_tuner_server.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include <cJSON.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <esp_http_server.h>

namespace esphome {
namespace satellite1_radar {

static const char *const TAG_RT = "radar_tuner_server";

static void set_common_headers_(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Connection", "close");
}

static esp_err_t send_json_ok_(httpd_req_t *req, const char *body) {
  httpd_resp_set_type(req, "application/json");
  set_common_headers_(req);
  return httpd_resp_sendstr(req, body);
}

static bool read_json_body_(httpd_req_t *req, std::string &out) {
  if (req->content_len <= 0 || req->content_len > 8192)
    return false;
  out.resize(static_cast<size_t>(req->content_len));
  int total_read = 0;
  while (total_read < req->content_len) {
    int read_len = httpd_req_recv(req, &out[total_read], req->content_len - total_read);
    if (read_len <= 0)
      return false;
    total_read += read_len;
  }
  return true;
}

static bool parse_bool_field_(cJSON *root, const char *name, bool &out, bool &present) {
  present = false;
  cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
  if (item == nullptr)
    return true;
  if (!cJSON_IsBool(item))
    return false;
  out = cJSON_IsTrue(item);
  present = true;
  return true;
}

static bool parse_uint_field_(cJSON *root, const char *name, uint32_t &out, bool &present) {
  present = false;
  cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
  if (item == nullptr)
    return true;
  if (!cJSON_IsNumber(item) || item->valuedouble < 0)
    return false;
  out = static_cast<uint32_t>(item->valuedouble);
  present = true;
  return true;
}

void RadarTunerServer::update_target(int index, float x, float y) {
  if (index >= 0 && index < RT_NUM_TARGETS) {
    targets_[index].x = x;
    targets_[index].y = y;
  }
}

void RadarTunerServer::clear_registrations() {
  ld2410_ = nullptr;
  ld2450_ = nullptr;
  on_ld2410_apply_ = nullptr;
}

esp_err_t RadarTunerServer::handle_root_(httpd_req_t *req) {
  auto *self = static_cast<RadarTunerServer *>(req->user_ctx);
  if (self->html_gz_ == nullptr || self->html_gz_len_ == 0) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No HTML content");
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  httpd_resp_set_hdr(req, "Connection", "close");
  return httpd_resp_send(req, reinterpret_cast<const char *>(self->html_gz_), self->html_gz_len_);
}

esp_err_t RadarTunerServer::handle_ld2410_get_config_(httpd_req_t *req) {
  auto *self = static_cast<RadarTunerServer *>(req->user_ctx);
  if (self->ld2410_ == nullptr) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "LD2410 handler unavailable");
    return ESP_FAIL;
  }

  const auto &cfg = self->ld2410_->get_backend_config();
  char buf[1024];
  int pos = 0;
  pos += snprintf(buf + pos, sizeof(buf) - pos,
                  "{\"timeout\":%u,\"max_move_gate\":%u,\"max_still_gate\":%u,\"distance_resolution\":\"%s\","
                  "\"bluetooth\":%s,\"gate_move_thresholds\":[",
                  static_cast<unsigned int>(cfg.timeout_seconds), static_cast<unsigned int>(cfg.max_move_gate),
                  static_cast<unsigned int>(cfg.max_still_gate), cfg.distance_resolution ? "0.2m" : "0.75m",
                  cfg.bluetooth_enabled ? "true" : "false");
  for (size_t g = 0; g < LD2410Handler::NUM_GATES; g++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%s%u", g ? "," : "",
                    static_cast<unsigned int>(cfg.gate_move_threshold[g]));
  }
  pos += snprintf(buf + pos, sizeof(buf) - pos, "],\"gate_still_thresholds\":[");
  for (size_t g = 0; g < LD2410Handler::NUM_GATES; g++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%s%u", g ? "," : "",
                    static_cast<unsigned int>(cfg.gate_still_threshold[g]));
  }
  pos += snprintf(buf + pos, sizeof(buf) - pos, "]}");
  return send_json_ok_(req, buf);
}

esp_err_t RadarTunerServer::handle_ld2410_patch_config_(httpd_req_t *req) {
  auto *self = static_cast<RadarTunerServer *>(req->user_ctx);
  if (self->ld2410_ == nullptr) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "LD2410 handler unavailable");
    return ESP_FAIL;
  }

  std::string body;
  if (!read_json_body_(req, body)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body");
    return ESP_FAIL;
  }
  cJSON *root = cJSON_ParseWithLength(body.c_str(), body.size());
  if (root == nullptr || !cJSON_IsObject(root)) {
    if (root != nullptr)
      cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return ESP_FAIL;
  }

  auto cfg = self->ld2410_->get_backend_config();
  uint32_t uval = 0;
  bool present = false;

  if (!parse_uint_field_(root, "timeout", uval, present) || (present && uval > 65535)) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid timeout");
    return ESP_FAIL;
  }
  if (present)
    cfg.timeout_seconds = static_cast<uint16_t>(uval);

  if (!parse_uint_field_(root, "max_move_gate", uval, present) || (present && uval > 8)) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid max_move_gate");
    return ESP_FAIL;
  }
  if (present)
    cfg.max_move_gate = static_cast<uint8_t>(uval);

  if (!parse_uint_field_(root, "max_still_gate", uval, present) || (present && uval > 8)) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid max_still_gate");
    return ESP_FAIL;
  }
  if (present)
    cfg.max_still_gate = static_cast<uint8_t>(uval);

  bool bool_val = false;
  if (!parse_bool_field_(root, "bluetooth", bool_val, present)) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid bluetooth");
    return ESP_FAIL;
  }
  if (present)
    cfg.bluetooth_enabled = bool_val;

  cJSON *distance_resolution = cJSON_GetObjectItemCaseSensitive(root, "distance_resolution");
  if (distance_resolution != nullptr) {
    if (!cJSON_IsString(distance_resolution) || distance_resolution->valuestring == nullptr) {
      cJSON_Delete(root);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid distance_resolution");
      return ESP_FAIL;
    }
    if (strcmp(distance_resolution->valuestring, "0.2m") == 0) {
      cfg.distance_resolution = 1;
    } else if (strcmp(distance_resolution->valuestring, "0.75m") == 0) {
      cfg.distance_resolution = 0;
    } else {
      cJSON_Delete(root);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid distance_resolution value");
      return ESP_FAIL;
    }
  }

  cJSON *move_thresholds = cJSON_GetObjectItemCaseSensitive(root, "gate_move_thresholds");
  if (move_thresholds != nullptr) {
    if (!cJSON_IsArray(move_thresholds) ||
        cJSON_GetArraySize(move_thresholds) != static_cast<int>(LD2410Handler::NUM_GATES)) {
      cJSON_Delete(root);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid gate_move_thresholds");
      return ESP_FAIL;
    }
    for (size_t g = 0; g < LD2410Handler::NUM_GATES; g++) {
      cJSON *item = cJSON_GetArrayItem(move_thresholds, static_cast<int>(g));
      if (!cJSON_IsNumber(item) || item->valuedouble < 0 || item->valuedouble > 100) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid move threshold value");
        return ESP_FAIL;
      }
      cfg.gate_move_threshold[g] = static_cast<uint8_t>(item->valuedouble);
    }
  }

  cJSON *still_thresholds = cJSON_GetObjectItemCaseSensitive(root, "gate_still_thresholds");
  if (still_thresholds != nullptr) {
    if (!cJSON_IsArray(still_thresholds) ||
        cJSON_GetArraySize(still_thresholds) != static_cast<int>(LD2410Handler::NUM_GATES)) {
      cJSON_Delete(root);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid gate_still_thresholds");
      return ESP_FAIL;
    }
    for (size_t g = 0; g < LD2410Handler::NUM_GATES; g++) {
      cJSON *item = cJSON_GetArrayItem(still_thresholds, static_cast<int>(g));
      if (!cJSON_IsNumber(item) || item->valuedouble < 0 || item->valuedouble > 100) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid still threshold value");
        return ESP_FAIL;
      }
      cfg.gate_still_threshold[g] = static_cast<uint8_t>(item->valuedouble);
    }
  }

  cJSON_Delete(root);
  if (!self->ld2410_->set_backend_config(cfg)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Config validation failed");
    return ESP_FAIL;
  }
  return send_json_ok_(req, "{\"status\":\"ok\"}");
}

esp_err_t RadarTunerServer::handle_ld2410_apply_(httpd_req_t *req) {
  auto *self = static_cast<RadarTunerServer *>(req->user_ctx);
  if (self->on_ld2410_apply_ != nullptr)
    self->on_ld2410_apply_();
  return send_json_ok_(req, "{\"status\":\"ok\"}");
}

esp_err_t RadarTunerServer::handle_ld2410_live_(httpd_req_t *req) {
  auto *self = static_cast<RadarTunerServer *>(req->user_ctx);
  if (self->ld2410_ == nullptr) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "LD2410 handler unavailable");
    return ESP_FAIL;
  }
  char buf[512];
  int pos = 0;
  pos += snprintf(buf + pos, sizeof(buf) - pos, "{\"gates\":{\"move\":[");
  for (int g = 0; g < RT_NUM_GATES; g++) {
    float val = self->ld2410_->get_gate_move_energy(static_cast<size_t>(g));
    if (std::isnan(val))
      val = 0;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%s%.0f", g ? "," : "", val);
  }
  pos += snprintf(buf + pos, sizeof(buf) - pos, "],\"still\":[");
  for (int g = 0; g < RT_NUM_GATES; g++) {
    float val = self->ld2410_->get_gate_still_energy(static_cast<size_t>(g));
    if (std::isnan(val))
      val = 0;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%s%.0f", g ? "," : "", val);
  }
  pos += snprintf(buf + pos, sizeof(buf) - pos, "]}}");
  return send_json_ok_(req, buf);
}

esp_err_t RadarTunerServer::handle_ld2450_get_config_(httpd_req_t *req) {
  auto *self = static_cast<RadarTunerServer *>(req->user_ctx);
  if (self->ld2450_ == nullptr) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "LD2450 handler unavailable");
    return ESP_FAIL;
  }

  const auto &cfg = self->ld2450_->get_backend_config();
  char buf[3072];
  int pos = 0;
  pos += snprintf(buf + pos, sizeof(buf) - pos,
                  "{\"detection_range\":%u,\"stability\":%u,\"timeout\":%u,\"bluetooth\":%s,\"multi_target\":%s,"
                  "\"reboot_required\":%s,\"zones\":[",
                  static_cast<unsigned int>(cfg.detection_range_cm), static_cast<unsigned int>(cfg.stability),
                  static_cast<unsigned int>(cfg.timeout_seconds), cfg.bluetooth_enabled ? "true" : "false",
                  cfg.multi_target_enabled ? "true" : "false", self->ld2450_->is_reboot_required() ? "true" : "false");

  for (size_t z = 0; z < LD2450Handler::NUM_ZONES; z++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%s[", z ? "," : "");
    for (size_t p = 0; p < cfg.zones[z].points_count; p++) {
      pos += snprintf(buf + pos, sizeof(buf) - pos, "%s{\"x\":%d,\"y\":%d}", p ? "," : "",
                      static_cast<int>(cfg.zones[z].points[p].x), static_cast<int>(cfg.zones[z].points[p].y));
    }
    pos += snprintf(buf + pos, sizeof(buf) - pos, "]");
  }
  pos += snprintf(buf + pos, sizeof(buf) - pos, "],\"exclusion\":[");
  for (size_t p = 0; p < cfg.exclusion.points_count; p++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%s{\"x\":%d,\"y\":%d}", p ? "," : "",
                    static_cast<int>(cfg.exclusion.points[p].x), static_cast<int>(cfg.exclusion.points[p].y));
  }
  pos += snprintf(buf + pos, sizeof(buf) - pos, "]}");
  return send_json_ok_(req, buf);
}

static bool parse_polygon_points_(cJSON *array, LD2450Handler::Polygon &polygon) {
  if (!cJSON_IsArray(array))
    return false;
  int count = cJSON_GetArraySize(array);
  if (count < 0 || count > static_cast<int>(LD2450Handler::MAX_ZONE_POINTS))
    return false;
  polygon.points_count = static_cast<uint8_t>(count);
  for (int i = 0; i < count; i++) {
    cJSON *point = cJSON_GetArrayItem(array, i);
    if (!cJSON_IsObject(point))
      return false;
    cJSON *x = cJSON_GetObjectItemCaseSensitive(point, "x");
    cJSON *y = cJSON_GetObjectItemCaseSensitive(point, "y");
    if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y))
      return false;
    polygon.points[i].x = static_cast<int16_t>(x->valueint);
    polygon.points[i].y = static_cast<int16_t>(y->valueint);
  }
  for (size_t i = static_cast<size_t>(count); i < LD2450Handler::MAX_ZONE_POINTS; i++) {
    polygon.points[i].x = 0;
    polygon.points[i].y = 0;
  }
  return true;
}

esp_err_t RadarTunerServer::handle_ld2450_patch_config_(httpd_req_t *req) {
  auto *self = static_cast<RadarTunerServer *>(req->user_ctx);
  if (self->ld2450_ == nullptr) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "LD2450 handler unavailable");
    return ESP_FAIL;
  }

  std::string body;
  if (!read_json_body_(req, body)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body");
    return ESP_FAIL;
  }
  cJSON *root = cJSON_ParseWithLength(body.c_str(), body.size());
  if (root == nullptr || !cJSON_IsObject(root)) {
    if (root != nullptr)
      cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return ESP_FAIL;
  }

  auto cfg = self->ld2450_->get_backend_config();
  uint32_t uval = 0;
  bool present = false;

  if (!parse_uint_field_(root, "detection_range", uval, present) || (present && uval > 600)) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid detection_range");
    return ESP_FAIL;
  }
  if (present)
    cfg.detection_range_cm = static_cast<uint16_t>(uval);

  if (!parse_uint_field_(root, "stability", uval, present) || (present && uval > 10)) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid stability");
    return ESP_FAIL;
  }
  if (present)
    cfg.stability = static_cast<uint8_t>(uval);

  if (!parse_uint_field_(root, "timeout", uval, present) || (present && uval > 65535)) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid timeout");
    return ESP_FAIL;
  }
  if (present)
    cfg.timeout_seconds = static_cast<uint16_t>(uval);

  bool bool_val = false;
  if (!parse_bool_field_(root, "bluetooth", bool_val, present)) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid bluetooth");
    return ESP_FAIL;
  }
  if (present)
    cfg.bluetooth_enabled = bool_val;

  if (!parse_bool_field_(root, "multi_target", bool_val, present)) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid multi_target");
    return ESP_FAIL;
  }
  if (present)
    cfg.multi_target_enabled = bool_val;

  cJSON *zones = cJSON_GetObjectItemCaseSensitive(root, "zones");
  if (zones != nullptr) {
    if (!cJSON_IsArray(zones) || cJSON_GetArraySize(zones) != static_cast<int>(LD2450Handler::NUM_ZONES)) {
      cJSON_Delete(root);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid zones");
      return ESP_FAIL;
    }
    for (size_t z = 0; z < LD2450Handler::NUM_ZONES; z++) {
      cJSON *zone_points = cJSON_GetArrayItem(zones, static_cast<int>(z));
      if (!parse_polygon_points_(zone_points, cfg.zones[z])) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid zone points");
        return ESP_FAIL;
      }
    }
  }

  cJSON *exclusion = cJSON_GetObjectItemCaseSensitive(root, "exclusion");
  if (exclusion != nullptr) {
    if (!parse_polygon_points_(exclusion, cfg.exclusion)) {
      cJSON_Delete(root);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid exclusion points");
      return ESP_FAIL;
    }
  }

  cJSON_Delete(root);
  if (!self->ld2450_->set_backend_config(cfg)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Config validation failed");
    return ESP_FAIL;
  }
  return send_json_ok_(req, self->ld2450_->is_reboot_required() ? "{\"status\":\"ok\",\"reboot_required\":true}"
                                                                : "{\"status\":\"ok\",\"reboot_required\":false}");
}

esp_err_t RadarTunerServer::handle_ld2450_live_(httpd_req_t *req) {
  auto *self = static_cast<RadarTunerServer *>(req->user_ctx);
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"targets\":["
           "{\"x\":%.1f,\"y\":%.1f},"
           "{\"x\":%.1f,\"y\":%.1f},"
           "{\"x\":%.1f,\"y\":%.1f}]}",
           self->targets_[0].x, self->targets_[0].y, self->targets_[1].x, self->targets_[1].y, self->targets_[2].x,
           self->targets_[2].y);
  return send_json_ok_(req, buf);
}

esp_err_t RadarTunerServer::handle_save_(httpd_req_t *req) {
  global_preferences->sync();
  ESP_LOGI(TAG_RT, "Preferences flushed to NVS");
  return send_json_ok_(req, "{\"status\":\"ok\"}");
}

esp_err_t RadarTunerServer::handle_reboot_(httpd_req_t *req) {
  global_preferences->sync();
  ESP_LOGI(TAG_RT, "Reboot requested by tuner UI");
  send_json_ok_(req, "{\"status\":\"ok\"}");
  App.safe_reboot();
  return ESP_OK;
}

void RadarTunerServer::start() {
  if (server_ != nullptr)
    return;

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 12;
  config.max_open_sockets = 2;
  config.backlog_conn = 5;
  config.stack_size = 8192;
  config.lru_purge_enable = true;
  config.uri_match_fn = httpd_uri_match_wildcard;

  esp_err_t err = httpd_start(&server_, &config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG_RT, "Failed to start httpd: %s", esp_err_to_name(err));
    server_ = nullptr;
    return;
  }

  httpd_uri_t root = {};
  root.uri = "/";
  root.method = HTTP_GET;
  root.handler = handle_root_;
  root.user_ctx = this;
  httpd_register_uri_handler(server_, &root);

  httpd_uri_t ld2410_cfg_get = {};
  ld2410_cfg_get.uri = "/api/v1/ld2410/config";
  ld2410_cfg_get.method = HTTP_GET;
  ld2410_cfg_get.handler = handle_ld2410_get_config_;
  ld2410_cfg_get.user_ctx = this;
  httpd_register_uri_handler(server_, &ld2410_cfg_get);

  httpd_uri_t ld2410_cfg_patch = {};
  ld2410_cfg_patch.uri = "/api/v1/ld2410/config";
  ld2410_cfg_patch.method = HTTP_PATCH;
  ld2410_cfg_patch.handler = handle_ld2410_patch_config_;
  ld2410_cfg_patch.user_ctx = this;
  httpd_register_uri_handler(server_, &ld2410_cfg_patch);

  httpd_uri_t ld2410_apply = {};
  ld2410_apply.uri = "/api/v1/ld2410/apply";
  ld2410_apply.method = HTTP_POST;
  ld2410_apply.handler = handle_ld2410_apply_;
  ld2410_apply.user_ctx = this;
  httpd_register_uri_handler(server_, &ld2410_apply);

  httpd_uri_t ld2410_live = {};
  ld2410_live.uri = "/api/v1/ld2410/live";
  ld2410_live.method = HTTP_GET;
  ld2410_live.handler = handle_ld2410_live_;
  ld2410_live.user_ctx = this;
  httpd_register_uri_handler(server_, &ld2410_live);

  httpd_uri_t ld2450_cfg_get = {};
  ld2450_cfg_get.uri = "/api/v1/ld2450/config";
  ld2450_cfg_get.method = HTTP_GET;
  ld2450_cfg_get.handler = handle_ld2450_get_config_;
  ld2450_cfg_get.user_ctx = this;
  httpd_register_uri_handler(server_, &ld2450_cfg_get);

  httpd_uri_t ld2450_cfg_patch = {};
  ld2450_cfg_patch.uri = "/api/v1/ld2450/config";
  ld2450_cfg_patch.method = HTTP_PATCH;
  ld2450_cfg_patch.handler = handle_ld2450_patch_config_;
  ld2450_cfg_patch.user_ctx = this;
  httpd_register_uri_handler(server_, &ld2450_cfg_patch);

  httpd_uri_t ld2450_live = {};
  ld2450_live.uri = "/api/v1/ld2450/live";
  ld2450_live.method = HTTP_GET;
  ld2450_live.handler = handle_ld2450_live_;
  ld2450_live.user_ctx = this;
  httpd_register_uri_handler(server_, &ld2450_live);

  httpd_uri_t save = {};
  save.uri = "/api/v1/save";
  save.method = HTTP_POST;
  save.handler = handle_save_;
  save.user_ctx = this;
  httpd_register_uri_handler(server_, &save);

  httpd_uri_t reboot = {};
  reboot.uri = "/api/v1/reboot";
  reboot.method = HTTP_POST;
  reboot.handler = handle_reboot_;
  reboot.user_ctx = this;
  httpd_register_uri_handler(server_, &reboot);

  ESP_LOGI(TAG_RT, "Radar tuner server started on port 80");
}

void RadarTunerServer::stop() {
  if (server_ == nullptr)
    return;

  httpd_stop(server_);
  server_ = nullptr;
  ESP_LOGI(TAG_RT, "Radar tuner server stopped");
}

}  // namespace satellite1_radar
}  // namespace esphome
