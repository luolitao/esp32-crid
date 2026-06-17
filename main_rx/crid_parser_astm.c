/**
 * crid_parser_astm.c — ASTM F3411 协议解析模块
 *
 * 专门处理 ASTM F3411 协议的数据解析
 */
#include <string.h>
#include "esp_log.h"
#include "opendroneid.h"
#include "odid_wifi.h"
#include "crid_parser.h"
#include "crid_json.h"
#include "crid_rx_types.h"

#define ASTM_MSG_SIZE           25
#define ASTM_PACK_MAX_MSGS      ODID_PACK_MAX_MESSAGES


/* ================================================================
 * 内部辅助函数
 * ================================================================ */
static inline uint16_t le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline int32_t le32s(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/* ================================================================
 * ASTM F3411 Packed 格式解析
 * ================================================================ */

/**
 * 解析 ASTM F3411 协议数据
 */
bool crid_parser_decode_astm(uav_track_t *uav, const uint8_t *data, uint8_t len) {
    if (!data || len < 1) return false;

    /* 策略 2: ASTM F3411 Packed 格式 */
    if (len > 1) {
        const uint8_t *pack_data = &data[1];
        uint8_t proto_msg_type   = pack_data[0];
        if (((proto_msg_type >> 4) & 0x0F) == ODID_MESSAGETYPE_PACKED) {
            uint8_t msg_count = pack_data[2];
            size_t pack_size  = sizeof(ODID_MessagePack_encoded) -
                                ASTM_MSG_SIZE * (ASTM_PACK_MAX_MSGS - msg_count);
            if (len - 1 >= pack_size) {
                int ret = odid_message_process_pack(&uav->uas_data, (uint8_t *)pack_data, len - 1);
                if (ret > 0) {
                    return true;
                }
            }
        }
    }

    /* 策略 4: ASTM 单消息格式 (Fallback) */
    {
        ODID_messagetype_t t0 = decodeMessageType(data[0]);
        if (t0 >= ODID_MESSAGETYPE_BASIC_ID && t0 <= ODID_MESSAGETYPE_OPERATOR_ID) {
            if (len >= ASTM_MSG_SIZE) {
                if (decodeOpenDroneID(&uav->uas_data, (uint8_t *)data) == ODID_SUCCESS) {
                    return true;
                }
            }
        }
    }

    return false;
}