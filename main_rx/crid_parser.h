/**
 * crid_parser.h — Remote ID 消息解析模块接口
 */

#ifndef CRID_PARSER_H
#define CRID_PARSER_H

#include "crid_rx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 使用 opendroneid 库解码一条消息（支持 Message Pack 和单消息格式）
 * @param uav   目标无人机追踪条目
 * @param data  原始数据
 * @param len   数据长度
 * @return      检测到的协议类型（RID_PROTOCOL_UNKNOWN 表示解码失败）
 */
rid_protocol_t crid_parser_decode(uav_track_t *uav, const uint8_t *data, uint8_t len);

/**
 * 从 ODID_UAS_Data 提取数据到分层结构体 (rid_location_t 等)
 * 应在每次解码成功后调用，供显示层使用
 * @param uav  目标无人机追踪条目
 */
void crid_parser_extract_layered(uav_track_t *uav);

#ifdef __cplusplus
}
#endif

#endif // CRID_PARSER_H
