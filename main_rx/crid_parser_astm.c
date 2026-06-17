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

static const char *TAG = "RID_ASTM";

/* ================================================================
 * 常量与宏定义
 * ================================================================ */
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
static bool decode_gb_format(uav_track_t *uav, const uint8_t *data, uint8_t len) {
    if (len < 4) return false;

    uint8_t gb_single_msg_size = data[2];
    uint8_t gb_msg_count       = data[3];
    if (gb_single_msg_size != ASTM_MSG_SIZE || gb_msg_count < 1 || gb_msg_count > ASTM_PACK_MAX_MSGS) {
        return false;
    }

    const uint8_t *gb_messages     = &data[4];
    uint8_t gb_msg_data_len        = len - 4;
    uint8_t gb_expected_len        = gb_msg_count * ASTM_MSG_SIZE;
    if (gb_msg_data_len < gb_expected_len) return false;

    // 构造 ASTM 兼容的 ODID_MessagePack_encoded 头部
    uint8_t tmp_pack[sizeof(ODID_MessagePack_encoded)];
    size_t tmp_pack_size = sizeof(ODID_MessagePack_encoded) -
                           ASTM_MSG_SIZE * (ASTM_PACK_MAX_MSGS - gb_msg_count);
    if (tmp_pack_size < 3 + gb_expected_len || tmp_pack_size > sizeof(tmp_pack)) {
        return false;
    }

    tmp_pack[0] = (ODID_MESSAGETYPE_PACKED << 4) | 0x01; // 0xF1
    tmp_pack[1] = ASTM_MSG_SIZE;
    tmp_pack[2] = gb_msg_count;
    memcpy(&tmp_pack[3], gb_messages, gb_expected_len);

    int ret = odid_message_process_pack(&uav->uas_data, tmp_pack, tmp_pack_size);
    if (ret > 0) {
        return true;
    }
    return false;
}

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