/**
 * crid_web_api.h — HTTP API 处理器声明
 *
 * 每个函数对应一个 JSON API 端点，由 crid_web.c 注册。
 */

#ifndef CRID_WEB_API_H
#define CRID_WEB_API_H

#include <esp_http_server.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t crid_web_api_system_info(httpd_req_t *req);
esp_err_t crid_web_api_device_status(httpd_req_t *req);
esp_err_t crid_web_api_ota_progress(httpd_req_t *req);
esp_err_t crid_web_api_system_status(httpd_req_t *req);
esp_err_t crid_web_api_uav_tracking(httpd_req_t *req);
esp_err_t crid_web_api_network_status(httpd_req_t *req);
esp_err_t crid_web_api_scan_stats(httpd_req_t *req);

#ifdef __cplusplus
}
#endif

#endif // CRID_WEB_API_H
