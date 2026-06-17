/**
 * crid_ota_web.h — Web OTA 更新服务模块头文件
 *
 * 提供基于HTTP的OTA更新功能，允许通过Web界面进行固件升级
 */

#ifndef CRID_OTA_WEB_H
#define CRID_OTA_WEB_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * OTA状态枚举
 */
typedef enum {
    OTA_IDLE,
    OTA_STARTED,
    OTA_IN_PROGRESS,
    OTA_COMPLETED,
    OTA_FAILED
} ota_state_t;

/**
 * 初始化OTA Web服务
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t crid_ota_web_init(void);

/**
 * 反初始化OTA Web服务
 */
void crid_ota_web_deinit(void);

/**
 * 检查OTA是否正在进行
 * @return true 如果正在更新，false 否则
 */
bool crid_ota_is_in_progress(void);

/**
 * 获取OTA更新进度百分比
 * @return 进度百分比 (0-100)
 */
int crid_ota_get_progress(void);

/**
 * 获取当前OTA状态
 * @return 当前OTA状态
 */
ota_state_t crid_ota_get_state(void);

/**
 * 获取最后一次错误信息
 * @return 错误信息字符串
 */
const char* crid_ota_get_last_error(void);

/**
 * 设置预期的MD5校验值用于固件验证
 * @param md5 16字节的MD5哈希值，如果为NULL则禁用校验
 */
void crid_ota_set_expected_md5(const uint8_t *md5);

/**
 * 获取当前系统状态信息
 * @return 系统状态JSON字符串
 */
const char* crid_ota_get_system_status(void);

/**
 * 获取当前无人机追踪信息
 * @return 无人机追踪状态JSON字符串
 */
const char* crid_ota_get_uav_tracking_status(void);

/**
 * 获取当前网络状态
 * @return 网络状态JSON字符串
 */
const char* crid_ota_get_network_status(void);

/**
 * 获取当前扫描统计信息
 * @return 扫描统计JSON字符串
 */
const char* crid_ota_get_scan_stats(void);

#ifdef __cplusplus
}
#endif

#endif // CRID_OTA_WEB_H