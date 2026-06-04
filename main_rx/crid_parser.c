/**
 * crid_parser.c — Remote ID 消息解析模块
 *
 * 使用 opendroneid 库的 odid_wifi 接口解码，支持：
 *   - ASTM F3411 ODID_service_info 格式 (Message Counter + Message Pack)
 *   - GB 42590-2023 国标格式 (Message Counter + 3字节管理信息 + 消息)
 *   - 单消息格式 (每个 IE 携带一条 25 字节消息)
 *
 * 数据布局 (data[0] 开始，sniffer 已跳过 ID+Len+OUI+OUI_Type=6 字节)：
 *
 *   ASTM 格式:
 *     [MessageCounter(1)] [MessagePack: 0xF2|ProtoVer(1)] [SingleMsgSize(1)] [MsgCount(1)] [messages...]
 *
 *   GB 42590-2023 国标格式:
 *     [MessageCounter(1)] [0xF1(1)] [SingleMsgSize(1)] [MsgCount(1)] [messages...]
 *     注意：国标 message_counter 之后有 3 字节管理信息 (0xF1 + SingleMsgSize + MsgCount)，
 *           然后才是消息体。与 ASTM 的区别是 Message Pack 的第一个字节：0xF1 vs 0xF2。
 *
 *   单消息格式:
 *     [SingleMsgHeader(0x02~0x52)] [25-byte message]
 *
 * 策略：优先用 odid_message_process_pack() 解析 Packed 格式（ASTM 0xF2）；
 *       再检测国标格式（0xF1）；否则尝试单消息格式。
 */

#include <string.h>
#include "esp_log.h"
#include "opendroneid.h"
#include "odid_wifi.h"
#include "crid_parser.h"
#include "crid_json.h"

rid_protocol_t crid_parser_decode(uav_track_t *uav, const uint8_t *data, uint8_t len) {
    if (len < 1) return RID_PROTOCOL_UNKNOWN;

    // 策略 1: 作为 ODID_service_info 解析 (ASTM 格式)
    //         格式: [MessageCounter(1)] [ODID_MessagePack_encoded(...)]
    //         Message Pack 第一个字节高4位 = 0xF (ODID_MESSAGETYPE_PACKED = 0xF)
    //         使用 opendroneid 库的 odid_message_process_pack()
    if (len > sizeof(uint8_t)) {
        struct ODID_service_info *si = (struct ODID_service_info *)data;
        ODID_messagetype_t t0 = decodeMessageType(si->odid_message_pack[0].MessageType);
        if (t0 == ODID_MESSAGETYPE_PACKED) {
            // 计算 Pack 大小 (MsgPackSize 表示消息数量，每条 25 字节)
            uint8_t msg_count = si->odid_message_pack[0].MsgPackSize;
            size_t pack_size = sizeof(ODID_MessagePack_encoded)
                             - ODID_MESSAGE_SIZE * (ODID_PACK_MAX_MESSAGES - msg_count);
            if (len >= sizeof(si->message_counter) + pack_size) {
                int ret = odid_message_process_pack(&uav->uas_data,
                                                     (uint8_t *)si->odid_message_pack,
                                                     len - sizeof(si->message_counter));
                if (ret > 0) {
                    uav->last_seen_ms = esp_log_timestamp();
                    uav->msg_count++;
                    return RID_PROTOCOL_ASTM_F3411;
                }
            }
        }
    }

    // 策略 2: GB 42590-2023 国标格式
    //         格式: [MessageCounter(1)] [0xF1(1)] [SingleMsgSize(1)] [MsgCount(1)] [messages...]
    //         前面 3 字节管理信息 (0xF1 + SingleMsgSize + MsgCount) 不属于 Message Pack，
    //         需要跳过这 3 字节，然后构造 ODID_MessagePack_encoded 头部再解析。
    if (len >= 1 + 1 + 1 + 1) {  // counter + 0xF1 + size + count
        uint8_t gb_magic = data[1];  // data[0] = message_counter, data[1] = 0xF1
        if (gb_magic == 0xF1) {
            uint8_t gb_single_msg_size = data[2];
            uint8_t gb_msg_count = data[3];
            if (gb_single_msg_size == ODID_MESSAGE_SIZE && gb_msg_count >= 1 && gb_msg_count <= ODID_PACK_MAX_MESSAGES) {
                // 构造 ASTM 兼容的 ODID_MessagePack_encoded 头部
                // ODID_MessagePack_encoded:
                //   Byte 0: [MessageType(4)][ProtoVersion(4)]
                //   Byte 1: SingleMessageSize
                //   Byte 2: MsgPackSize
                //   Byte 3+: Messages[]
                // 国标消息从 data[4] 开始
                const uint8_t *gb_messages = &data[4];
                uint8_t gb_msg_data_len = len - 4;
                uint8_t gb_expected_len = gb_msg_count * ODID_MESSAGE_SIZE;
                if (gb_msg_data_len >= gb_expected_len) {
                    // 构建临时 buffer，前面放 ASTM 兼容头部，后面放消息数据
                    uint8_t tmp_pack[sizeof(ODID_MessagePack_encoded)];
                    size_t tmp_pack_size = sizeof(ODID_MessagePack_encoded)
                                         - ODID_MESSAGE_SIZE * (ODID_PACK_MAX_MESSAGES - gb_msg_count);
                    if (tmp_pack_size >= 3 + gb_expected_len && tmp_pack_size <= sizeof(tmp_pack)) {
                        tmp_pack[0] = (ODID_MESSAGETYPE_PACKED << 4) | 0x01; // MessageType=PACKED, ProtoVer=1
                        tmp_pack[1] = ODID_MESSAGE_SIZE;
                        tmp_pack[2] = gb_msg_count;
                        memcpy(&tmp_pack[3], gb_messages, gb_expected_len);
                        int ret = odid_message_process_pack(&uav->uas_data, tmp_pack, tmp_pack_size);
                        if (ret > 0) {
                            uav->last_seen_ms = esp_log_timestamp();
                            uav->msg_count++;
                            return RID_PROTOCOL_GB42590;
                        }
                    }
                }
            }
        }
    }

    // 策略 3 (fallback): 尝试 data[0] 作为单消息头
    ODID_messagetype_t t0 = decodeMessageType(data[0]);
    if (t0 >= ODID_MESSAGETYPE_BASIC_ID && t0 <= ODID_MESSAGETYPE_OPERATOR_ID) {
        if (len >= ODID_MESSAGE_SIZE) {
            int ret = decodeOpenDroneID(&uav->uas_data, (uint8_t *)data);
            if (ret == ODID_SUCCESS) {
                uav->last_seen_ms = esp_log_timestamp();
                uav->msg_count++;
                return RID_PROTOCOL_ASTM_F3411;  // 单消息格式默认按 ASTM 处理
            }
        }
    }

    // 所有策略都失败，打印诊断
    static uint32_t s_fail_count = 0;
    s_fail_count++;
    if ((s_fail_count & 0x1F) == 0) {
        json_decode_fail(data[0], (len > 1 ? data[1] : 0), len);
    }
    return RID_PROTOCOL_UNKNOWN;
}

/* ================================================================
 * 分层数据提取 (参照 ORIP 设计，从 ODID_UAS_Data 提取到 rid_*_t)
 * ================================================================ */

void crid_parser_extract_layered(uav_track_t *uav) {
    if (!uav) return;

    // -- Basic ID --
    uav->basic_id.valid = false;
    if (uav->uas_data.BasicIDValid[0]) {
        const ODID_BasicID_data *b = &uav->uas_data.BasicID[0];
        uav->basic_id.valid   = true;
        uav->basic_id.id_type = (uint8_t)b->IDType;
        uav->basic_id.ua_type = (uint8_t)b->UAType;
        strncpy(uav->basic_id.uas_id, b->UASID, sizeof(uav->basic_id.uas_id) - 1);
        uav->basic_id.uas_id[sizeof(uav->basic_id.uas_id) - 1] = '\0';
    }

    // -- Location --
    uav->location.valid = false;
    if (uav->uas_data.LocationValid) {
        const ODID_Location_data *l = &uav->uas_data.Location;
        uav->location.valid           = true;
        uav->location.latitude        = l->Latitude;
        uav->location.longitude       = l->Longitude;
        uav->location.altitude_baro   = l->AltitudeBaro;
        uav->location.altitude_geo    = l->AltitudeGeo;
        uav->location.height          = l->Height;
        uav->location.height_ref      = (uint8_t)l->HeightType;
        uav->location.speed_horizontal = l->SpeedHorizontal;
        uav->location.speed_vertical  = l->SpeedVertical;
        uav->location.direction       = l->Direction;
        uav->location.status          = (uint8_t)l->Status;
        uav->location.h_accuracy      = (uint8_t)l->HorizAccuracy;
        uav->location.v_accuracy      = (uint8_t)l->VertAccuracy;
        uav->location.baro_accuracy   = (uint8_t)l->BaroAccuracy;
        uav->location.speed_accuracy  = (uint8_t)l->SpeedAccuracy;
        uav->location.ts_accuracy     = (uint8_t)l->TSAccuracy;
        uav->location.timestamp       = l->TimeStamp;
    }

    // -- System Info --
    uav->system.valid = false;
    if (uav->uas_data.SystemValid) {
        const ODID_System_data *s = &uav->uas_data.System;
        uav->system.valid                  = true;
        uav->system.operator_location_type = (uint8_t)s->OperatorLocationType;
        uav->system.operator_latitude      = s->OperatorLatitude;
        uav->system.operator_longitude     = s->OperatorLongitude;
        uav->system.operator_altitude_geo  = s->OperatorAltitudeGeo;
        uav->system.area_count             = s->AreaCount;
        uav->system.area_radius            = s->AreaRadius;
        uav->system.area_ceiling           = s->AreaCeiling;
        uav->system.area_floor             = s->AreaFloor;
        uav->system.classification_type    = (uint8_t)s->ClassificationType;
        uav->system.category_eu            = (uint8_t)s->CategoryEU;
        uav->system.class_eu               = (uint8_t)s->ClassEU;
        uav->system.timestamp              = s->Timestamp;
    }

    // -- Self ID --
    uav->self_id.valid = false;
    if (uav->uas_data.SelfIDValid) {
        const ODID_SelfID_data *s = &uav->uas_data.SelfID;
        uav->self_id.valid            = true;
        uav->self_id.description_type = (uint8_t)s->DescType;
        strncpy(uav->self_id.description, s->Desc, sizeof(uav->self_id.description) - 1);
        uav->self_id.description[sizeof(uav->self_id.description) - 1] = '\0';
    }

    // -- Operator ID --
    uav->operator_id.valid = false;
    if (uav->uas_data.OperatorIDValid) {
        const ODID_OperatorID_data *o = &uav->uas_data.OperatorID;
        uav->operator_id.valid   = true;
        uav->operator_id.id_type = (uint8_t)o->OperatorIdType;
        strncpy(uav->operator_id.id, o->OperatorId, sizeof(uav->operator_id.id) - 1);
        uav->operator_id.id[sizeof(uav->operator_id.id) - 1] = '\0';
    }
}
