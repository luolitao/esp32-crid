/**
 * crid_parser.c — Remote ID 消息解析模块
 *
 * 使用 opendroneid 库解码，支持：
 *   - Message Pack 格式 (ASTM F3411-22 / GB 42590-2023)
 *   - 单消息格式 (每个 IE 携带一条 25 字节消息)
 *
 * 数据布局 (data[0] 开始，sniffer 已跳过 ID+Len+OUI+OUI_Type=6 字节)：
 *   ASTM F3411:      [MessagePackHeader(0xF2)] [SingleMsgSize(25)] [MsgCount] [messages...]
 *   GB 42590 Beacon:  [Counter] [MessagePackHeader(0xF2)] [SingleMsgSize(25)] [MsgCount] [messages...]
 *   单消息格式:       [SingleMsgHeader(0x02~0x52)] [25-byte message]
 *
 * 策略：优先检查 data[0] 是否为 PACKED header；再尝试跳过 Counter 后解码；
 * 最后尝试单消息格式。避免 Counter 字节被误判为单消息类型产生噪声告警。
 */

#include "esp_log.h"
#include <string.h>
#include "opendroneid.h"
#include "crid_parser.h"

static const char *TAG = "RID_PARSE";

// 尝试以 data[0] 为起点解析（静默失败，由调用者决定是否告警）
static int try_decode(uav_track_t *uav, const uint8_t *data, uint8_t len) {
    if (len < 1) return ODID_FAIL;

    ODID_messagetype_t msg_type = decodeMessageType(data[0]);

    if (msg_type == ODID_MESSAGETYPE_PACKED) {
        if (len < 28) return ODID_FAIL;
        ODID_MessagePack_encoded *pack = (ODID_MessagePack_encoded *)data;
        // 快速验证：SingleMessageSize 必须是 25（标准 RID 消息大小）
        if (pack->SingleMessageSize != ODID_MESSAGE_SIZE) return ODID_FAIL;
        int ret = decodeMessagePack(&uav->uas_data, pack);
        if (ret != ODID_SUCCESS) {
            ESP_LOGW(TAG, "Message Pack decode failed (err=%d) SingleMsgSize=%u MsgPackSize=%u",
                     ret, pack->SingleMessageSize, pack->MsgPackSize);
        }
        return ret;
    } else if (msg_type >= ODID_MESSAGETYPE_BASIC_ID && msg_type <= ODID_MESSAGETYPE_OPERATOR_ID) {
        if (len < ODID_MESSAGE_SIZE) return ODID_FAIL;
        int ret = decodeOpenDroneID(&uav->uas_data, (uint8_t *)data);
        // 不在此处打印单消息失败告警——调用者知道上下文（可能是 Counter 误判）
        return ret;
    }
    return ODID_FAIL;
}

void crid_parser_decode(uav_track_t *uav, const uint8_t *data, uint8_t len) {
    if (len < 1) return;

    // 策略 1: 检查 data[0] 是否为 PACKED header (0xF2)
    //         ASTM F3411 格式，数据直接以 Message Pack 开始
    ODID_messagetype_t t0 = decodeMessageType(data[0]);
    if (t0 == ODID_MESSAGETYPE_PACKED) {
        if (try_decode(uav, data, len) == ODID_SUCCESS) {
            uav->last_seen_ms = esp_log_timestamp();
            uav->msg_count++;
            return;
        }
    }

    // 策略 2: 跳过 1 字节 (Counter)，用于 GB 42590 Beacon 格式
    //         data: [Counter] [MessagePackHeader(0xF2)] [SingleMsgSize] [MsgCount] ...
    if (len > 1) {
        ODID_messagetype_t t1 = decodeMessageType(data[1]);
        if (t1 == ODID_MESSAGETYPE_PACKED) {
            if (try_decode(uav, data + 1, len - 1) == ODID_SUCCESS) {
                uav->last_seen_ms = esp_log_timestamp();
                uav->msg_count++;
                return;
            }
        }
    }

    // 策略 3: 跳过 2 字节 (Counter + Reserved)，用于 GB 42590 变体
    //         data: [Counter] [Reserved/0x00] [MessagePackHeader(0xF2)] ...
    if (len > 2) {
        ODID_messagetype_t t2 = decodeMessageType(data[2]);
        if (t2 == ODID_MESSAGETYPE_PACKED) {
            if (try_decode(uav, data + 2, len - 2) == ODID_SUCCESS) {
                uav->last_seen_ms = esp_log_timestamp();
                uav->msg_count++;
                return;
            }
        }
    }

    // 策略 4 (fallback): 如果以上都未找到 PACKED header，尝试 data[0] 作为单消息
    //         注意：此分支也可能是 Counter 字节被误判，但至少尝试一次
    if (t0 >= ODID_MESSAGETYPE_BASIC_ID && t0 <= ODID_MESSAGETYPE_OPERATOR_ID) {
        if (try_decode(uav, data, len) == ODID_SUCCESS) {
            uav->last_seen_ms = esp_log_timestamp();
            uav->msg_count++;
            return;
        } else {
            // 单消息解码失败，可能是 Counter 误判，不打印告警
            // 真正的单消息格式解码失败才需要关注，但目前无法区分
        }
    }

    // 所有策略都失败，打印诊断
    static uint32_t s_fail_count = 0;
    s_fail_count++;
    if ((s_fail_count & 0x1F) == 0) {
        ESP_LOGW(TAG, "All decode strategies failed: [0]=0x%02X [1]=0x%02X [2]=0x%02X len=%u",
                 data[0], (len > 1 ? data[1] : 0), (len > 2 ? data[2] : 0), len);
    }
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
