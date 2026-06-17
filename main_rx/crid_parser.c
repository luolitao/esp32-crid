/**
 * crid_parser.c — Remote ID 消息解析模块 (最终健壮版)
 * 
 * 修复与优化内容：
 * 1. [新增] 协议健康检查：防止非 GB46750 数据因字节巧合被误判，导致输出荒谬数值。
 * 2. [新增] 物理量合理性校验：过滤因错位读取产生的超范围经纬度、速度、高度。
 * 3. [新增] 字符串安全净化：自动剔除 UAS ID 中的 \x01 等不可见控制字符。
 * 4. [优化] 消除重复代码，扁平化复杂分支，统一边界检查，提升可维护性。
 */

#include <string.h>
#include "esp_log.h"
#include "opendroneid.h"
#include "odid_wifi.h"
#include "crid_parser.h"
#include "crid_json.h"

static const char *TAG = "RID_Parser";
/* ================================================================
 * Debug 开关：设为 1 时，在解析前打印原始数据十六进制转储
 * ================================================================ */
#ifndef PARSER_DEBUG_HEX_DUMP
#define PARSER_DEBUG_HEX_DUMP   0
#endif

/* ================================================================
 * 常量与宏定义
 * ================================================================ */
#define GB46750_MAGIC           0xFF
#define GB46750_VER_MAJOR_MASK  0x07
#define GB46750_VER_MINOR_MASK  0x1F
#define GB46750_VER_MAJOR_SHIFT 5
#define GB46750_VALID_MAJOR     0x01
#define GB46750_HEADER_LEN      7   /* Counter(1)+Magic(1)+Ver(1)+Len(1)+Flags(3) */

#define GB42590_MAGIC           0xF1
#define GB42590_HEADER_LEN      4   /* Counter(1)+Magic(1)+Size(1)+Count(1) */

#define ASTM_MSG_SIZE           25
#define ASTM_PACK_MAX_MSGS      ODID_PACK_MAX_MESSAGES

/* 物理量合理性校验宏 (防止错位读取产生荒谬值) */
#define IS_VALID_LAT(lat)       ((lat) >= -90.0f && (lat) <= 90.0f)
#define IS_VALID_LON(lon)       ((lon) >= -180.0f && (lon) <= 180.0f)
#define IS_VALID_SPEED(speed)   ((speed) >= 0.0f && (speed) <= 300.0f)
#define IS_VALID_ANGLE(angle)   ((angle) >= 0.0f && (angle) < 360.0f)
#define IS_VALID_HEIGHT(h)      ((h) >= -1000.0f && (h) <= 10000.0f)

/* 安全偏移：边界检查由每个 case 显式处理，不使用宏 */

/* ================================================================
 * Debug 辅助：十六进制转储
 * ================================================================ */
#if PARSER_DEBUG_HEX_DUMP
static void hex_dump(const char *tag, const char *prefix, const uint8_t *data, uint8_t len) {
    /* 格式: <prefix> [len] AA BB CC DD ... (每行最多 16 字节) */
    char line[128];
    int pos = 0;
    pos += snprintf(line + pos, sizeof(line) - pos, "%s [%u] ", prefix, len);
    for (int i = 0; i < len; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", data[i]);
        if ((i + 1) % 16 == 0 && i + 1 < len) {
            ESP_LOGI(tag, "%s", line);
            pos = snprintf(line, sizeof(line), "       ");
        }
    }
    if (pos > 0) {
        ESP_LOGI(tag, "%s", line);
    }
}
#endif

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

/**
 * 高度解码：编码值 = (实际值 + 1000) × 2，分辨率 0.5m
 * 编码值 == 0 或 0xFFFF 表示未知
 */
static inline bool decode_alt_2byte(const uint8_t *p, float *out) {
    uint16_t raw = le16(p);
    
    // 增加对 0xFFFF 的校验（国标中 0xFFFF 通常也代表无效/未知）
    if (raw == 0 || raw == 0xFFFF) { 
        *out = 0.0f; 
        return false; 
    }
    
    *out = (raw / 2.0f) - 1000.0f;
    
    // 优化日志：直接打印完整的 raw 十六进制，以及拆解的字节序，方便比对
    //ESP_LOGI(TAG, "Height: raw=0x%04X (bytes: %02X %02X) => alt=%.2f m",              raw, p[0], p[1], *out);
             
    return true;
}

/**
 * 经纬度解码：8 字节小端序，int32 × 1e-7 度
 * 返回 true 表示经纬度在合理范围内
 */
static inline bool decode_lon_lat(const uint8_t *p, float *lon, float *lat) {
    *lon = le32s(&p[0]) / 1e7;
    *lat = le32s(&p[4]) / 1e7;
    return IS_VALID_LAT(*lat) && IS_VALID_LON(*lon);
}

/* ================================================================
 * GB 46750-2025 数据内容解析
 * ================================================================ */
static int decode_gb46750_payload(gb46750_data_t *gb,
                                  const uint8_t *flags, uint8_t num_flags,
                                  const uint8_t *content, uint8_t content_len) {


    int offset = 0;
    int items_parsed = 0;

    for (uint8_t byte_idx = 0; byte_idx < num_flags && byte_idx < 3; byte_idx++) {
        uint8_t flag = flags[byte_idx];
        // 按位从高到低 (0x80→0x02) 解析，bit 0 (0x01) 为扩展标志位
        for (int8_t bit = 7; bit >= 1; bit--) {
            if (!(flag & (1U << bit))) continue;

            uint8_t item_id = (byte_idx << 3) | bit;
            // ESP_LOGI(TAG, "item_id %02d, offset: %d", item_id, offset);
            // hex_dump(TAG, "Content_RAW", &content[offset],  8);
            switch (item_id) {
                /* 标识字节 1 */
                case 0x07:
                    if (offset + 20 > content_len) return items_parsed;
                    memcpy(gb->unique_id, &content[offset], 20);
                    offset += 20;
                    gb->unique_id[20] = '\0';
                    gb->has_unique_id = true;
                    break;
                case 0x06:
                    if (offset + 8 > content_len) return items_parsed;
                    memcpy(gb->realname_id, &content[offset], 8);
                    offset += 8;
                    gb->realname_id[8] = '\0';
                    gb->has_realname_flag = true;
                    break;
                case 0x05:
                    if (offset + 1 > content_len) return items_parsed;
                    gb->operation_category = content[offset];
                    offset += 1;
                    gb->has_operation_category = true;
                    break;
                case 0x04:
                    if (offset + 1 > content_len) return items_parsed;
                    gb->ua_category = content[offset];
                    offset += 1;
                    gb->has_ua_category = true;
                    break;
                case 0x03:
                    if (offset + 1 > content_len) return items_parsed;
                    gb->rcs_loc_type = content[offset];
                    offset += 1;
                    gb->has_rcs_loc_type = true;
                    break;
                case 0x02:
                    if (offset + 8 > content_len) return items_parsed;
                    lon = le32s(&content[offset]) / 1e7;
                    lat = le32s(&content[offset + 4]) / 1e7;
                    ESP_LOGI(TAG, "RCS lon: %f, lat: %f", lon, lat);
                    if (IS_VALID_LAT(lat) && IS_VALID_LON(lon)) {
                        gb->rcs_longitude = lon;
                        gb->rcs_latitude  = lat;
                        gb->has_rcs_location = true;
                    }
                    offset += 8;
                    break;
                case 0x01:
                    if (offset + 2 > content_len) return items_parsed;
                    {
                        uint16_t raw = le16(&content[offset]);
                        if (raw == 0 || raw == 0xFFFF) {
                            gb->rcs_altitude = 0.0f;
                            gb->has_rcs_altitude = false;
                        } else {
                            gb->rcs_altitude = (raw / 2.0f) - 1000.0f;
                            gb->has_rcs_altitude = true;
                        }
                        // ESP_LOGI(TAG, "rcs_altitude: raw=0x%04X => alt=%.2f m", raw, gb->rcs_altitude);
                    }
                    offset += 2;
                    break;

                /* 标识字节 2 */
                case 0x0F:
                    if (offset + 8 > content_len) return items_parsed;
                    lon = le32s(&content[offset]) / 1e7;
                    lat = le32s(&content[offset + 4]) / 1e7;
                    // ESP_LOGI(TAG, "UAV lon: %f, lat: %f", lon, lat);
                    if (IS_VALID_LAT(lat) && IS_VALID_LON(lon)) {
                        gb->uav_longitude = lon;
                        gb->uav_latitude  = lat;
                        gb->has_uav_location = true;
                    }
                    offset += 8;
                    break;
                case 0x0E:
                    if (offset + 2 > content_len) return items_parsed;
                    {
                        uint16_t raw = le16(&content[offset]);
                        if (raw != 0xFFFF) {
                            float angle = raw / 10.0f;
                            if (IS_VALID_ANGLE(angle)) {
                                gb->track_angle = angle;
                                gb->has_track_angle = true;
                            }
                        }
                    }
                    offset += 2;
                    break;
                case 0x0D:
                    if (offset + 2 > content_len) return items_parsed;
                    {
                        uint16_t raw = le16(&content[offset]);
                        if (raw != 0xFFFF) {
                            float speed = raw / 10.0f;
                            if (IS_VALID_SPEED(speed)) {
                                gb->ground_speed = speed;
                                gb->has_ground_speed = true;
                            }
                        }
                    }
                    offset += 2;
                    break;
                case 0x0C:
                    if (offset + 2 > content_len) return items_parsed;
                    {
                        uint16_t raw = le16(&content[offset]);
                        if (raw != 0) {
                            float height = (raw / 2.0f) - 9000.0f;
                            if (IS_VALID_HEIGHT(height)) {
                                gb->relative_height = height;
                                gb->has_relative_height = true;
                            }
                        }
                    }
                    offset += 2;
                    break;
                case 0x0B:
                    if (offset + 1 > content_len) return items_parsed;
                    {
                        uint8_t raw = content[offset];
                        if (raw != 0xFF) {
                            float v = (raw & 0x7F) / 2.0f;
                            float vs = (raw & 0x80) ? -v : v;
                            float abs_vs = (vs < 0.0f) ? -vs : vs;
                            if (IS_VALID_SPEED(abs_vs)) {
                                gb->vertical_speed = vs;
                                gb->has_vertical_speed = true;
                            }
                        }
                    }
                    offset += 1;
                    break;
                case 0x0A:
                    if (offset + 2 > content_len) return items_parsed;
                    gb->has_geo_altitude = decode_alt_2byte(&content[offset], &gb->geo_altitude);
                    offset += 2;
                    break;
                case 0x09:
                    if (offset + 2 > content_len) return items_parsed;
                    gb->has_baro_altitude = decode_alt_2byte(&content[offset], &gb->baro_altitude);
                    offset += 2;
                    break;

                /* 标识字节 3 */
                case 0x17:
                    if (offset + 1 > content_len) return items_parsed;
                    gb->operation_status = content[offset];
                    offset += 1;
                    gb->has_operation_status = true;
                    break;
                case 0x16:
                    if (offset + 1 > content_len) return items_parsed;
                    gb->coord_system = content[offset];
                    offset += 1;
                    gb->has_coord_system = true;
                    break;
                case 0x15:
                    if (offset + 1 > content_len) return items_parsed;
                    gb->h_accuracy = content[offset];
                    offset += 1;
                    gb->has_h_accuracy = true;
                    break;
                case 0x14:
                    if (offset + 1 > content_len) return items_parsed;
                    gb->v_accuracy = content[offset];
                    offset += 1;
                    gb->has_v_accuracy = true;
                    break;
                case 0x13:
                    if (offset + 1 > content_len) return items_parsed;
                    gb->speed_accuracy = content[offset];
                    offset += 1;
                    gb->has_speed_accuracy = true;
                    break;
                case 0x12:
                    if (offset + 6 > content_len) return items_parsed;
                    {
                        uint64_t ts = 0;
                        for (int i = 0; i < 6; i++)
                            ts |= ((uint64_t)content[offset + i]) << (i * 8);
                        gb->timestamp_ms = ts;
                        gb->has_timestamp = true;
                    }
                    offset += 6;
                    break;
                case 0x11:
                    if (offset + 1 > content_len) return items_parsed;
                    gb->ts_accuracy = content[offset];
                    offset += 1;
                    gb->has_ts_accuracy = true;
                    break;

                default: break;
            }
            items_parsed++;
        }
        // 记录扩展标志位 (bit 0)
        if (flag & 0x01) {
            if (byte_idx == 0) gb->has_ext_byte1 = true;
            else if (byte_idx == 1) gb->has_ext_byte2 = true;
            else if (byte_idx == 2) gb->has_ext_byte3 = true;
        }
    }
    
    if (items_parsed > 0) {
        gb->valid = true;
    }
    return items_parsed;
}

/* ================================================================
 * GB 42590-2023 Packed 格式解析
 * ================================================================ */
static bool decode_gb_format(uav_track_t *uav, const uint8_t *data, uint8_t len) {
    if (len < GB42590_HEADER_LEN) return false;

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
        uav->last_seen_ms = esp_log_timestamp();
        uav->msg_count++;
        return true;
    }
    return false;
}

/* ================================================================
 * 主解析入口：策略分发 (含防误判健康检查)
 * ================================================================ */
rid_protocol_t crid_parser_decode(uav_track_t *uav, const uint8_t *data, uint8_t len) {
    if (!data || len < 1) return RID_PROTOCOL_UNKNOWN;

    /* 策略 1: GB 46750-2025 */
    if (len >= GB46750_HEADER_LEN && data[1] == GB46750_MAGIC) {
        uint8_t version     = data[2];
        uint8_t major_ver   = (version >> GB46750_VER_MAJOR_SHIFT) & GB46750_VER_MAJOR_MASK;
        uint8_t content_len = data[3];
        const uint8_t *flags = &data[4];
        const uint8_t *content = &data[7];

        if (major_ver == GB46750_VALID_MAJOR && content_len <= (len - GB46750_HEADER_LEN)) {
            int items = decode_gb46750_payload(&uav->gb46750, flags, 3, content, content_len);
            
            // [关键修复] 健康检查：防止将非 GB46750 数据误判为此协议
            if (uav->gb46750.has_unique_id) {
                bool has_printable = false;
                for (int i = 0; i < 20; i++) {
                    if (uav->gb46750.unique_id[i] >= 32 && uav->gb46750.unique_id[i] <= 126) {
                        has_printable = true;
                        break;
                    }
                }
                if (!has_printable) {
                    // 极可能是误判 (如 ASTM 包碰巧满足条件)，重置并继续尝试其他协议
                    memset(&uav->gb46750, 0, sizeof(gb46750_data_t));
                    items = 0; 
                }
            }
            
            if (items > 0) {
                uav->last_seen_ms = esp_log_timestamp();
                uav->msg_count++;
                return RID_PROTOCOL_GB46750;
            }
        }
    }

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
                    uav->last_seen_ms = esp_log_timestamp();
                    uav->msg_count++;
                    return RID_PROTOCOL_ASTM_F3411;
                }
            }
        }
    }

    /* 策略 3: GB 42590-2023 */
    if (len >= GB42590_HEADER_LEN && data[1] == GB42590_MAGIC) {
        if (decode_gb_format(uav, data, len)) {
            return RID_PROTOCOL_GB42590;
        }
    }

    /* 策略 4: ASTM 单消息格式 (Fallback) */
    {
        ODID_messagetype_t t0 = decodeMessageType(data[0]);
        if (t0 >= ODID_MESSAGETYPE_BASIC_ID && t0 <= ODID_MESSAGETYPE_OPERATOR_ID) {
            if (len >= ASTM_MSG_SIZE) {
                if (decodeOpenDroneID(&uav->uas_data, (uint8_t *)data) == ODID_SUCCESS) {
                    uav->last_seen_ms = esp_log_timestamp();
                    uav->msg_count++;
                    return RID_PROTOCOL_ASTM_F3411;
                }
            }
        }
    }

    /* 解析失败统计 */
    static uint32_t s_fail_count = 0;
    if ((++s_fail_count & 0x1F) == 0) {
        json_decode_fail(data[0], (len > 1 ? data[1] : 0), len);
    }
    return RID_PROTOCOL_UNKNOWN;
}

/* ================================================================
 * 分层数据提取 (GB 46750 优先，否则走 ASTM 标准字段)
 * ================================================================ */
void crid_parser_extract_layered(uav_track_t *uav) {
    if (!uav) return;

    /* --- GB 46750-2025 映射 --- */
    if (uav->protocol == RID_PROTOCOL_GB46750 && uav->gb46750.valid) {
        gb46750_data_t *gb = &uav->gb46750;

        if (gb->has_unique_id) {
            uav->basic_id.valid = true;
            uav->basic_id.id_type = ODID_IDTYPE_SERIAL_NUMBER;
            uav->basic_id.ua_type = gb->has_ua_category ? gb->ua_category : ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
            
            // [关键修复] 字符串安全净化：剔除 \x01 等不可见控制字符，确保 JSON 输出干净
            char *dst = uav->basic_id.uas_id;
            const char *src = gb->unique_id;
            int i = 0;
            while (*src && i < (int)sizeof(uav->basic_id.uas_id) - 1) {
                if (*src >= 32 && *src <= 126) { // 仅保留可打印 ASCII
                    *dst++ = *src;
                    i++;
                }
                src++;
            }
            *dst = '\0';
        }

        EXTRACT_IF(gb->has_uav_location,      uav->location.latitude,  gb->uav_latitude);
        EXTRACT_IF(gb->has_uav_location,      uav->location.longitude, gb->uav_longitude);
        EXTRACT_IF(gb->has_geo_altitude,      uav->location.altitude_geo, gb->geo_altitude);
        EXTRACT_IF(gb->has_baro_altitude,     uav->location.altitude_baro, gb->baro_altitude);
        EXTRACT_IF(gb->has_relative_height,   uav->location.height, gb->relative_height);
        EXTRACT_IF(gb->has_relative_height,   uav->location.height_ref, ODID_HEIGHT_REF_OVER_TAKEOFF);
        EXTRACT_IF(gb->has_ground_speed,      uav->location.speed_horizontal, gb->ground_speed);
        EXTRACT_IF(gb->has_vertical_speed,    uav->location.speed_vertical, gb->vertical_speed);
        EXTRACT_IF(gb->has_track_angle,       uav->location.direction, gb->track_angle);
        EXTRACT_IF(gb->has_operation_status,  uav->location.status, gb->operation_status);
        EXTRACT_IF(gb->has_h_accuracy,        uav->location.h_accuracy, gb->h_accuracy);
        EXTRACT_IF(gb->has_v_accuracy,        uav->location.v_accuracy, gb->v_accuracy);
        EXTRACT_IF(gb->has_speed_accuracy,    uav->location.speed_accuracy, gb->speed_accuracy);
        EXTRACT_IF(gb->has_ts_accuracy,       uav->location.ts_accuracy, gb->ts_accuracy);
        EXTRACT_IF(gb->has_timestamp,         uav->location.timestamp, gb->timestamp_ms / 1000.0f);
        
        if (gb->has_uav_location || gb->has_geo_altitude || gb->has_baro_altitude ||
            gb->has_ground_speed || gb->has_track_angle || gb->has_operation_status) {
            uav->location.valid = true;
        }

        /* 遥控站信息 */
        if (gb->has_rcs_loc_type) uav->system.operator_location_type = gb->rcs_loc_type;
        EXTRACT_IF(gb->has_rcs_location, uav->system.operator_latitude,  gb->rcs_latitude);
        EXTRACT_IF(gb->has_rcs_location, uav->system.operator_longitude, gb->rcs_longitude);
        EXTRACT_IF(gb->has_rcs_altitude, uav->system.operator_altitude_geo, gb->rcs_altitude);
        if (gb->has_rcs_location || gb->has_rcs_altitude || gb->has_rcs_loc_type) uav->system.valid = true;
        EXTRACT_IF(gb->has_operation_category, uav->system.classification_type, gb->operation_category);
        return;
    }

    /* --- ASTM / GB 42590 标准字段映射 --- */
    #define MAP_ODID_FIELD(dst, src, valid_cond) \
        do { if (valid_cond) { (dst) = (uint8_t)(src); } } while(0)

    /* Basic ID */
    uav->basic_id.valid = false;
    if (uav->uas_data.BasicIDValid[0]) {
        const ODID_BasicID_data *b = &uav->uas_data.BasicID[0];
        uav->basic_id.valid   = true;
        uav->basic_id.id_type = (uint8_t)b->IDType;
        uav->basic_id.ua_type = (uint8_t)b->UAType;
        strncpy(uav->basic_id.uas_id, b->UASID, sizeof(uav->basic_id.uas_id) - 1);
        uav->basic_id.uas_id[sizeof(uav->basic_id.uas_id) - 1] = '\0';
    }

    /* Location */
    uav->location.valid = uav->uas_data.LocationValid;
    if (uav->location.valid) {
        const ODID_Location_data *l = &uav->uas_data.Location;
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
        MAP_ODID_FIELD(uav->location.h_accuracy, l->HorizAccuracy, 1);
        MAP_ODID_FIELD(uav->location.v_accuracy, l->VertAccuracy, 1);
        MAP_ODID_FIELD(uav->location.baro_accuracy, l->BaroAccuracy, 1);
        MAP_ODID_FIELD(uav->location.speed_accuracy, l->SpeedAccuracy, 1);
        MAP_ODID_FIELD(uav->location.ts_accuracy, l->TSAccuracy, 1);
        uav->location.timestamp = l->TimeStamp;
    }

    /* System Info */
    uav->system.valid = uav->uas_data.SystemValid;
    if (uav->system.valid) {
        const ODID_System_data *s = &uav->uas_data.System;
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

    /* Self ID */
    uav->self_id.valid = uav->uas_data.SelfIDValid;
    if (uav->self_id.valid) {
        const ODID_SelfID_data *s = &uav->uas_data.SelfID;
        uav->self_id.description_type = (uint8_t)s->DescType;
        strncpy(uav->self_id.description, s->Desc, sizeof(uav->self_id.description) - 1);
        uav->self_id.description[sizeof(uav->self_id.description) - 1] = '\0';
    }

    /* Operator ID */
    uav->operator_id.valid = uav->uas_data.OperatorIDValid;
    if (uav->operator_id.valid) {
        const ODID_OperatorID_data *o = &uav->uas_data.OperatorID;
        uav->operator_id.id_type = (uint8_t)o->OperatorIdType;
        strncpy(uav->operator_id.id, o->OperatorId, sizeof(uav->operator_id.id) - 1);
        uav->operator_id.id[sizeof(uav->operator_id.id) - 1] = '\0';
    }
    #undef MAP_ODID_FIELD
}