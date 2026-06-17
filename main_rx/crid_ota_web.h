/**
 * crid_ota_web.h — Web OTA 更新服务公共接口
 */

#ifndef CRID_OTA_WEB_H
#define CRID_OTA_WEB_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 版本与构建信息（由编译系统注入） */
#ifndef CRID_VERSION_STRING
#define CRID_VERSION_STRING "1.0.0"
#endif
#ifndef CRID_BUILD_DATE
#define CRID_BUILD_DATE     __DATE__
#endif
#ifndef CRID_BUILD_TIME
#define CRID_BUILD_TIME     __TIME__
#endif

/** OTA 状态枚举 */
typedef enum {
    OTA_IDLE,
    OTA_STARTED,
    OTA_IN_PROGRESS,
    OTA_COMPLETED,
    OTA_FAILED
} ota_state_t;

/* ---------- Web 服务器生命周期 ---------- */

/** 启动 HTTP 服务器并注册所有路由 */
esp_err_t crid_ota_web_init(void);

/** 停止 HTTP 服务器 */
void crid_ota_web_deinit(void);

/* ---------- OTA 状态查询 ---------- */

/** 正在执行 OTA 上传？ */
bool crid_ota_is_in_progress(void);

/** 上传进度百分比 (0-100) */
int crid_ota_get_progress(void);

/** 当前 OTA 状态 */
ota_state_t crid_ota_get_state(void);

/** 最近一次错误信息 */
const char *crid_ota_get_last_error(void);

/** 设置预期的 MD5 校验值（保留） */
void crid_ota_set_expected_md5(const uint8_t *md5);

/* ---------- 供路由注册使用的 OTA 处理函数 ---------- */

esp_err_t crid_ota_upload_handler(httpd_req_t *req);

#ifdef __cplusplus
}
#endif

#endif // CRID_OTA_WEB_H
