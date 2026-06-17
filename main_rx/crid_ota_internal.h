/**
 * crid_ota_internal.h — OTA 模块内部接口
 *
 * 供 Web API 处理器访问 OTA 运行时状态
 */

#ifndef CRID_OTA_INTERNAL_H
#define CRID_OTA_INTERNAL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool        crid_ota_is_active(void);
int         crid_ota_get_total_size(void);
int         crid_ota_get_received_size(void);
const char *crid_ota_get_error_string(void);
const char *crid_ota_state_name(void);

#ifdef __cplusplus
}
#endif

#endif // CRID_OTA_INTERNAL_H
