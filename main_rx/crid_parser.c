/**
 * crid_parser.c — Remote ID 消息解析模块
 *
 * 使用 opendroneid 库的 odid_wifi 接口解码，支持：
 *   - ASTM F3411 Packed 格式
 *     (Message Counter + ODID_MessagePack_encoded)
 *   - GB 42590-2023 国标格式
 *     (Message Counter + 3 字节管理信息 + 消息体)
 *   - GB 46750-2023 国标格式
 *     (Message Counter + 0xFF + 版本号 + 数据长度 + 数据标识 + 数据内容)
 *   - 单消息格式 (每个 IE 携带一条 25 字节消息)
 *
 * 数据布局 (data[0] 开始，sniffer 已跳过 ID+Len+OUI+OUI_Type=6 字节)：
 *
 *   ASTM 格式:
 *     [MessageCounter(1)] [ODID_MessagePack_encoded(...)]
 *
 *     ODID_MessagePack_encoded 结构:
 *       Byte 0: [ProtoVersion:4][MessageType:4]
 *               MessageType=PACKED=0xF, ProtoVersion=1 → 0xF1
 *       Byte 1: SingleMessageSize (= 25)
 *       Byte 2: MsgPackSize (消息数量)
 *       Byte 3+: Messages[]
 *
 *   GB 42590-2023 格式:
 *     [MessageCounter(1)] [0xF1 magic(1)] [SingleMsgSize(1)] [MsgCount(1)] [messages...]
 *
 *     注：国标在 MessageCounter 之后有独立的 3 字节管理信息
 *         (magic + SingleMsgSize + MsgCount)，消息体从 data[4] 开始。
 *
 *   GB 46750-2023 格式:
 *     [MessageCounter(1)] [0xFF data_type(1)] [版本号(1)] [数据内容长度(1)] [数据标识(3)] [数据内容(变长)]
 *
 *     版本号: 高3位 = 主版本(0x1)，低5位 = 子版本号
 *     数据标识固定 3 字节，数据内容长度 = data[3] 的值
 *
 *   单消息格式:
 *     [SingleMsgHeader(0x00~0x5F)] [25-byte message]
 *
 * 策略：先尝试 GB 46750 格式 (magic=0xFF)；
 *       再尝试 ASTM Packed 格式；
 *       再尝试 GB 42590 格式 (magic=0xF1)；
 *       最后尝试单消息格式。
 */

#include <string.h>
#include "esp_log.h"
#include "opendroneid.h"
#include "odid_wifi.h"
#include "crid_parser.h"
#include "crid_json.h"

/* ================================================================
 * 内部辅助：解析 GB 46750-2023 数据内容
 *
 * 格式: [MessageCounter(1)] [0xFF(1)] [版本号(1)] [数据内容长度(1)] [数据标识(3)] [数据内容(变长)]
 *
 * 数据标识位表（GB 46750 表2）定义了 21 个数据内容项（001-021），
 * 按位解析数据标识字节，顺序提取对应的数据内容。
 *
 * 数据内容按标识位从高到低（0x80→0x01）顺序排列。
 * 当标识字节的最低位 0x01 为 1 时，表示有扩展标识字节。
 *
 * @param gb       目标 GB 46750 数据结构
 * @param flags    数据标识字节数组（至少 3 字节）
 * @param num_flags 标识字节数（= N+3）
 * @param content  数据内容起始指针
 * @param content_len 数据内容总长度
 * @return         成功解析的数据项数量
 * ================================================================ */
/**
 * 小端序解码辅助：从字节数组读取 uint16
 */
static inline uint16_t le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/**
 * 小端序解码辅助：从字节数组读取 int32
 */
static inline int32_t le32s(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8)
                   | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/**
 * 高度解码：编码值 = (实际值 + 1000) × 2，分辨率 0.5m
 * 编码值 == 0 表示未知
 */
static inline bool decode_alt_2byte(const uint8_t *p, float *out) {
    uint16_t raw = le16(p);
    if (raw == 0) { *out = 0; return false; }  // 未知
    *out = (raw / 2.0f) - 1000.0f;
    return true;
}

static int decode_gb46750_payload(gb46750_data_t *gb,
                                   const uint8_t *flags, uint8_t num_flags,
                                   const uint8_t *content, uint8_t content_len) {
    int offset = 0;
    int items_parsed = 0;

    // 遍历所有标识字节
    for (int byte_idx = 0; byte_idx < num_flags && byte_idx < 4; byte_idx++) {
        uint8_t flag = flags[byte_idx];

        // 按位从高到低 (0x80→0x01) 解析，bit 0 (0x01) 是扩展标志位
        for (int bit = 7; bit >= 1; bit--) {
            if (!(flag & (1 << bit))) continue;
            if (offset >= content_len) return items_parsed;

            if (byte_idx == 0) {
                // ---- 标识字节1 ----
                switch (bit) {
                    case 7: // 0x80: 001 唯一产品识别码 (M) — 固定20字节，大端序，ASCII编码
                        if (offset + 20 > content_len) return items_parsed;
                        memcpy(gb->unique_id, &content[offset], 20);
                        gb->unique_id[20] = '\0';
                        gb->has_unique_id = true;
                        offset += 20;
                        break;
                    case 6: // 0x40: 002 实名登记标志 (M) — 固定8字节，大端序，ASCII编码
                        if (offset + 8 > content_len) return items_parsed;
                        memcpy(gb->realname_id, &content[offset], 8);
                        gb->realname_id[8] = '\0';
                        gb->has_realname_flag = true;
                        offset += 8;
                        break;
                    case 5: // 0x20: 003 运行类别 (O) — 1 字节
                        gb->operation_category = content[offset];
                        gb->has_operation_category = true;
                        offset += 1;
                        break;
                    case 4: // 0x10: 004 民用无人驾驶航空器分类 (M) — 1 字节
                        gb->ua_category = content[offset];
                        gb->has_ua_category = true;
                        offset += 1;
                        break;
                    case 3: // 0x08: 005 遥控站位置类型 (M) — 1 字节
                        gb->rcs_loc_type = content[offset];
                        gb->has_rcs_loc_type = true;
                        offset += 1;
                        break;
                    case 2: // 0x04: 006 遥控站位置 (M) — 8字节，小端序，int32×1e7 (lat|lon)
                        if (offset + 8 > content_len) return items_parsed;
                        gb->rcs_latitude  = le32s(&content[offset])     / 1e7;
                        gb->rcs_longitude = le32s(&content[offset + 4]) / 1e7;
                        gb->has_rcs_location = true;
                        offset += 8;
                        break;
                    case 1: // 0x02: 007 遥控站高度 (M) — 2字节，小端序，(val+1000)×2，分辨率0.5m
                        if (offset + 2 > content_len) return items_parsed;
                        gb->has_rcs_altitude = decode_alt_2byte(&content[offset], &gb->rcs_altitude);
                        offset += 2;
                        break;
                    default: break;
                }
            } else if (byte_idx == 1) {
                // ---- 标识字节2 ----
                switch (bit) {
                    case 7: // 0x80: 008 民用无人驾驶航空器位置 (M) — 8字节，小端序，int32×1e7 (lat|lon)
                        if (offset + 8 > content_len) return items_parsed;
                        gb->uav_latitude  = le32s(&content[offset])     / 1e7;
                        gb->uav_longitude = le32s(&content[offset + 4]) / 1e7;
                        gb->has_uav_location = true;
                        offset += 8;
                        break;
                    case 6: // 0x40: 009 航迹角 (M) — 2字节，小端序，uint16，分辨率0.1°
                        {
                            if (offset + 2 > content_len) return items_parsed;
                            uint16_t raw = le16(&content[offset]);
                            if (raw != 0xFFFF) {
                                gb->track_angle = raw / 10.0f;
                                gb->has_track_angle = true;
                            }
                            offset += 2;
                        }
                        break;
                    case 5: // 0x20: 010 地速 (M) — 2字节，小端序，uint16，分辨率0.1m/s
                        {
                            if (offset + 2 > content_len) return items_parsed;
                            uint16_t raw = le16(&content[offset]);
                            if (raw != 0xFFFF) {
                                gb->ground_speed = raw / 10.0f;
                                gb->has_ground_speed = true;
                            }
                            offset += 2;
                        }
                        break;
                    case 4: // 0x10: 011 相对高度 (O) — 2字节，小端序，(val+9000)×2，分辨率0.5m
                        if (offset + 2 > content_len) return items_parsed;
                        {
                            uint16_t raw = le16(&content[offset]);
                            if (raw != 0) {
                                gb->relative_height = (raw / 2.0f) - 9000.0f;
                                gb->has_relative_height = true;
                            }
                            offset += 2;
                        }
                        break;
                    case 3: // 0x08: 012 垂直速度 (O) — 1字节，bit7=方向(0上升/1下降)，bit6-0=实际值×2，分辨率0.5m/s
                        {
                            uint8_t raw = content[offset];
                            if (raw != 0xFF) {
                                float v = (raw & 0x7F) / 2.0f;
                                gb->vertical_speed = (raw & 0x80) ? -v : v;
                                gb->has_vertical_speed = true;
                            }
                            offset += 1;
                        }
                        break;
                    case 2: // 0x04: 013 大地高度 (M) — 2字节，小端序，(val+1000)×2，分辨率0.5m
                        if (offset + 2 > content_len) return items_parsed;
                        gb->has_geo_altitude = decode_alt_2byte(&content[offset], &gb->geo_altitude);
                        offset += 2;
                        break;
                    case 1: // 0x02: 014 气压高度 (O) — 2字节，小端序，(val+1000)×2，分辨率0.5m
                        if (offset + 2 > content_len) return items_parsed;
                        gb->has_baro_altitude = decode_alt_2byte(&content[offset], &gb->baro_altitude);
                        offset += 2;
                        break;
                    default: break;
                }
            } else if (byte_idx == 2) {
                // ---- 标识字节3 ----
                switch (bit) {
                    case 7: // 0x80: 015 运行状态 (M) — 1 字节
                        gb->operation_status = content[offset];
                        gb->has_operation_status = true;
                        offset += 1;
                        break;
                    case 6: // 0x40: 016 坐标系类型 (M) — 1 字节
                        gb->coord_system = content[offset];
                        gb->has_coord_system = true;
                        offset += 1;
                        break;
                    case 5: // 0x20: 017 水平精度 (M) — 1 字节
                        gb->h_accuracy = content[offset];
                        gb->has_h_accuracy = true;
                        offset += 1;
                        break;
                    case 4: // 0x10: 018 垂直精度 (M) — 1 字节
                        gb->v_accuracy = content[offset];
                        gb->has_v_accuracy = true;
                        offset += 1;
                        break;
                    case 3: // 0x08: 019 速度精度 (M) — 1 字节
                        gb->speed_accuracy = content[offset];
                        gb->has_speed_accuracy = true;
                        offset += 1;
                        break;
                    case 2: // 0x04: 020 时间戳 (M) — 6字节，小端序，Unix毫秒时间戳
                        {
                            if (offset + 6 > content_len) return items_parsed;
                            uint64_t ts = 0;
                            for (int i = 0; i < 6; i++) {
                                ts |= ((uint64_t)content[offset + i]) << (i * 8);
                            }
                            gb->timestamp_ms = ts;
                            gb->has_timestamp = true;
                            offset += 6;
                        }
                        break;
                    case 1: // 0x02: 021 时间戳精度 (M) — 1 字节
                        gb->ts_accuracy = content[offset];
                        gb->has_ts_accuracy = true;
                        offset += 1;
                        break;
                    default: break;
                }
            }
            items_parsed++;
        }

        // 检查扩展标志位 (bit 0 = 0x01)
        if (flag & 0x01) {
            if (byte_idx == 0) gb->has_ext_byte1 = true;
            else if (byte_idx == 1) gb->has_ext_byte2 = true;
            else if (byte_idx == 2) gb->has_ext_byte3 = true;
        }
    }

    gb->valid = true;
    return items_parsed;
}

/* ================================================================
 * 内部辅助：解析 GB 42590-2023 Packed 格式
 *
 * 格式: [MessageCounter(1)] [0xF1 magic(1)] [SingleMsgSize(1)] [MsgCount(1)] [messages...]
 * 与 ASTM 的区别：GB 42590 有一个独立的 0xF1 magic byte，
 * 而 ASTM 的 ODID_MessagePack_encoded Byte 0 本身就是 0xF1（位域编码）。
 * 两者消息体偏移相同（data[4]），但在语义上需要区分协议类型。
 *
 * @param uav      目标无人机追踪条目
 * @param data     原始数据
 * @param len      数据长度
 * @param protocol 成功时返回的协议类型
 * @return         true=解码成功，false=不是该格式或解码失败
 * ================================================================ */
static bool decode_gb_format(uav_track_t *uav, const uint8_t *data, uint8_t len,
                              rid_protocol_t protocol) {
    if (len < 4) return false;

    uint8_t gb_single_msg_size = data[2];
    uint8_t gb_msg_count = data[3];

    if (gb_single_msg_size != ODID_MESSAGE_SIZE || gb_msg_count < 1 || gb_msg_count > ODID_PACK_MAX_MESSAGES) {
        return false;
    }

    const uint8_t *gb_messages = &data[4];
    uint8_t gb_msg_data_len = len - 4;
    uint8_t gb_expected_len = gb_msg_count * ODID_MESSAGE_SIZE;

    if (gb_msg_data_len < gb_expected_len) return false;

    // 构造 ASTM 兼容的 ODID_MessagePack_encoded 头部
    uint8_t tmp_pack[sizeof(ODID_MessagePack_encoded)];
    size_t tmp_pack_size = sizeof(ODID_MessagePack_encoded)
                         - ODID_MESSAGE_SIZE * (ODID_PACK_MAX_MESSAGES - gb_msg_count);
    if (tmp_pack_size < 3 + gb_expected_len || tmp_pack_size > sizeof(tmp_pack)) {
        return false;
    }

    tmp_pack[0] = (ODID_MESSAGETYPE_PACKED << 4) | 0x01; // MessageType=PACKED, ProtoVer=1 → 0xF1
    tmp_pack[1] = ODID_MESSAGE_SIZE;
    tmp_pack[2] = gb_msg_count;
    memcpy(&tmp_pack[3], gb_messages, gb_expected_len);

    int ret = odid_message_process_pack(&uav->uas_data, tmp_pack, tmp_pack_size);
    if (ret > 0) {
        uav->last_seen_ms = esp_log_timestamp();
        uav->msg_count++;
        return true;
    }
    return false;
}

rid_protocol_t crid_parser_decode(uav_track_t *uav, const uint8_t *data, uint8_t len) {
    if (len < 1) return RID_PROTOCOL_UNKNOWN;

    // 策略 1: GB 46750-2023 国标格式
    //         格式: [MessageCounter(1)] [0xFF data_type(1)] [版本号(1)] [数据内容长度(1)] [数据标识(3)] [数据内容(变长)]
    //         检测条件: data[1] == 0xFF 且版本号高3位 == 0x1
    //         注: data[3] = 数据内容长度（字节数），不是数据标识长度
    if (len >= 5 && data[1] == 0xFF) {
        uint8_t version = data[2];
        uint8_t major_ver = (version >> 5) & 0x07;  // 高3位
        uint8_t minor_ver = version & 0x1F;          // 低5位
        uint8_t content_len = data[3];                // 数据内容长度（字节数）

        // 数据标识固定 3 字节，从 data[4] 开始；数据内容从 data[7] 开始
        const uint8_t *flags = &data[4];
        const uint8_t *content = &data[7];
        uint8_t actual_content_len = len - 7;

        if (major_ver == 0x1 && len >= 7 && content_len <= actual_content_len) {
            int items = decode_gb46750_payload(&uav->gb46750, flags, 3,
                                                content, content_len);

            //ESP_LOGI("CRID_PARSER", "GB 46750 v%d.%d, items_parsed=%d",
            //         major_ver, minor_ver, items);

            uav->last_seen_ms = esp_log_timestamp();
            uav->msg_count++;
            return RID_PROTOCOL_GB46750;
        }
    }

    // 策略 2: ASTM F3411 Packed 格式
    //         格式: [MessageCounter(1)] [ODID_MessagePack_encoded(...)]
    //         ODID_MessagePack_encoded Byte 0:
    //           [ProtoVersion:4][MessageType:4] — MessageType=PACKED=0xF, ProtoVersion=1 → 0xF1
    //         使用 opendroneid 库的 odid_message_process_pack()
    if (len > sizeof(uint8_t)) {
        struct ODID_service_info *si = (struct ODID_service_info *)data;
        ODID_messagetype_t t0 = decodeMessageType(si->odid_message_pack[0].MessageType);
        if (t0 == ODID_MESSAGETYPE_PACKED) {
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

    // 策略 3: GB 42590-2023 国标格式 (magic=0xF1)
    //         格式: [MessageCounter(1)] [0xF1(1)] [SingleMsgSize(1)] [MsgCount(1)] [messages...]
    if (len >= 4 && data[1] == 0xF1) {
        if (decode_gb_format(uav, data, len, RID_PROTOCOL_GB42590)) {
            return RID_PROTOCOL_GB42590;
        }
    }

    // 策略 4 (fallback): 尝试 data[0] 作为单消息头
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

    // -- GB 46750-2023 数据提取 --
    if (uav->protocol == RID_PROTOCOL_GB46750 && uav->gb46750.valid) {
        gb46750_data_t *gb = &uav->gb46750;

        // 映射 Basic ID: 唯一产品识别码
        if (gb->has_unique_id) {
            uav->basic_id.valid = true;
            uav->basic_id.id_type = ODID_IDTYPE_SERIAL_NUMBER;  // 默认按序列号
            uav->basic_id.ua_type = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;  // 默认多旋翼
            strncpy(uav->basic_id.uas_id, gb->unique_id, sizeof(uav->basic_id.uas_id) - 1);
            uav->basic_id.uas_id[sizeof(uav->basic_id.uas_id) - 1] = '\0';
        }

        // 如果有分类信息，更新 UA 类型
        if (gb->has_ua_category) {
            uav->basic_id.ua_type = gb->ua_category;
        }

        // 映射 Location: 无人机位置
        if (gb->has_uav_location) {
            uav->location.valid = true;
            uav->location.latitude = gb->uav_latitude;
            uav->location.longitude = gb->uav_longitude;
        }
        if (gb->has_geo_altitude) {
            uav->location.altitude_geo = gb->geo_altitude;
        }
        if (gb->has_baro_altitude) {
            uav->location.altitude_baro = gb->baro_altitude;
        }
        if (gb->has_relative_height) {
            uav->location.height = gb->relative_height;
            uav->location.height_ref = ODID_HEIGHT_REF_OVER_TAKEOFF;
        }
        if (gb->has_ground_speed) {
            uav->location.speed_horizontal = gb->ground_speed;
        }
        if (gb->has_vertical_speed) {
            uav->location.speed_vertical = gb->vertical_speed;
        }
        if (gb->has_track_angle) {
            uav->location.direction = gb->track_angle;
        }
        if (gb->has_operation_status) {
            uav->location.status = gb->operation_status;
        }
        if (gb->has_h_accuracy) {
            uav->location.h_accuracy = gb->h_accuracy;
        }
        if (gb->has_v_accuracy) {
            uav->location.v_accuracy = gb->v_accuracy;
        }
        if (gb->has_speed_accuracy) {
            uav->location.speed_accuracy = gb->speed_accuracy;
        }
        if (gb->has_ts_accuracy) {
            uav->location.ts_accuracy = gb->ts_accuracy;
        }
        if (gb->has_timestamp) {
            // GB 46750 时间戳是 Unix 毫秒，location.timestamp 是秒（float）
            uav->location.timestamp = gb->timestamp_ms / 1000.0f;
        }

        // 映射 System Info: 遥控站信息
        if (gb->has_rcs_location || gb->has_rcs_altitude || gb->has_rcs_loc_type) {
            uav->system.valid = true;
        }
        if (gb->has_rcs_loc_type) {
            uav->system.operator_location_type = gb->rcs_loc_type;
        }
        if (gb->has_rcs_location) {
            uav->system.operator_latitude = gb->rcs_latitude;
            uav->system.operator_longitude = gb->rcs_longitude;
        }
        if (gb->has_rcs_altitude) {
            uav->system.operator_altitude_geo = gb->rcs_altitude;
        }
        if (gb->has_operation_category) {
            uav->system.classification_type = gb->operation_category;
        }

        return;  // GB 46750 处理完毕，跳过 ASTM/GB 42590 的提取
    }

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
