/**
 * crid_display.c — 无人机信息展示模块
 *
 * 包含：
 *   - 枚举值 → 可读名称映射表 (基于分层结构体 rid_*_t)
 *   - MAC 地址格式化
 *   - 摘要/详情/状态行打印
 *
 * 设计：使用 rid_*_t 分层结构体（与 ODID_UAS_Data 解耦），
 *       所有枚举值通过 uint8_t 传递，映射表参照 ASTM F3411 / ORIP 标准。
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "crid_display.h"
#include "opendroneid.h"  // 仅用于 ODID_* 枚举常量值引用

static const char *TAG = "RID_DISP";

/* ================================================================
 * 枚举值 → 可读名称映射表 (基于 rid_*_t 的 uint8_t 字段)
 * 参照 ASTM F3411 / ORIP types.h 完整枚举
 * ================================================================ */

static const char *get_id_type_name(uint8_t t) {
    switch (t) {
        case ODID_IDTYPE_NONE:                return "None";
        case ODID_IDTYPE_SERIAL_NUMBER:       return "Serial Number";
        case ODID_IDTYPE_CAA_REGISTRATION_ID: return "CAA Registration ID";
        case ODID_IDTYPE_UTM_ASSIGNED_UUID:   return "UTM Assigned UUID";
        case ODID_IDTYPE_SPECIFIC_SESSION_ID: return "Specific Session ID";
        default:                              return "Unknown";
    }
}

static const char *get_ua_type_name(uint8_t t) {
    switch (t) {
        case ODID_UATYPE_NONE:                     return "None";
        case ODID_UATYPE_AEROPLANE:                return "Aeroplane";
        case ODID_UATYPE_HELICOPTER_OR_MULTIROTOR: return "Helicopter/Multirotor";
        case ODID_UATYPE_GYROPLANE:                return "Gyroplane";
        case ODID_UATYPE_HYBRID_LIFT:              return "Hybrid Lift";
        case ODID_UATYPE_ORNITHOPTER:              return "Ornithopter";
        case ODID_UATYPE_GLIDER:                   return "Glider";
        case ODID_UATYPE_KITE:                     return "Kite";
        case ODID_UATYPE_FREE_BALLOON:             return "Free Balloon";
        case ODID_UATYPE_CAPTIVE_BALLOON:          return "Captive Balloon";
        case ODID_UATYPE_AIRSHIP:                  return "Airship";
        case ODID_UATYPE_FREE_FALL_PARACHUTE:      return "Free Fall/Parachute";
        case ODID_UATYPE_ROCKET:                   return "Rocket";
        case ODID_UATYPE_TETHERED_POWERED_AIRCRAFT: return "Tethered Powered";
        case ODID_UATYPE_GROUND_OBSTACLE:          return "Ground Obstacle";
        case ODID_UATYPE_OTHER:                    return "Other";
        default:                                   return "Unknown";
    }
}

static const char *get_status_name(uint8_t s) {
    switch (s) {
        case ODID_STATUS_UNDECLARED:                 return "Undeclared";
        case ODID_STATUS_GROUND:                     return "Ground";
        case ODID_STATUS_AIRBORNE:                   return "Airborne";
        case ODID_STATUS_EMERGENCY:                  return "Emergency";
        case ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE:   return "RID System Failure";
        default:                                     return "Unknown";
    }
}

static const char *get_height_ref_name(uint8_t h) {
    switch (h) {
        case ODID_HEIGHT_REF_OVER_TAKEOFF: return "Over Takeoff";
        case ODID_HEIGHT_REF_OVER_GROUND:  return "Over Ground";
        default:                           return "Unknown";
    }
}

static const char *get_horiz_acc_name(uint8_t a) {
    switch (a) {
        case ODID_HOR_ACC_UNKNOWN:  return "Unknown";
        case ODID_HOR_ACC_10NM:     return "18.52 km";
        case ODID_HOR_ACC_4NM:      return "7.408 km";
        case ODID_HOR_ACC_2NM:      return "3.704 km";
        case ODID_HOR_ACC_1NM:      return "1.852 km";
        case ODID_HOR_ACC_0_5NM:    return "926 m";
        case ODID_HOR_ACC_0_3NM:    return "555.6 m";
        case ODID_HOR_ACC_0_1NM:    return "185.2 m";
        case ODID_HOR_ACC_0_05NM:   return "92.6 m";
        case ODID_HOR_ACC_30_METER: return "30 m";
        case ODID_HOR_ACC_10_METER: return "10 m";
        case ODID_HOR_ACC_3_METER:  return "3 m";
        case ODID_HOR_ACC_1_METER:  return "1 m";
        default:                    return "Unknown";
    }
}

static const char *get_vert_acc_name(uint8_t a) {
    switch (a) {
        case ODID_VER_ACC_UNKNOWN:   return "Unknown";
        case ODID_VER_ACC_150_METER: return "150 m";
        case ODID_VER_ACC_45_METER:  return "45 m";
        case ODID_VER_ACC_25_METER:  return "25 m";
        case ODID_VER_ACC_10_METER:  return "10 m";
        case ODID_VER_ACC_3_METER:   return "3 m";
        case ODID_VER_ACC_1_METER:   return "1 m";
        default:                     return "Unknown";
    }
}

static const char *get_speed_acc_name(uint8_t a) {
    switch (a) {
        case ODID_SPEED_ACC_UNKNOWN:                   return "Unknown";
        case ODID_SPEED_ACC_10_METERS_PER_SECOND:      return "10 m/s";
        case ODID_SPEED_ACC_3_METERS_PER_SECOND:       return "3 m/s";
        case ODID_SPEED_ACC_1_METERS_PER_SECOND:       return "1 m/s";
        case ODID_SPEED_ACC_0_3_METERS_PER_SECOND:     return "0.3 m/s";
        default:                                       return "Unknown";
    }
}

static const char *get_desc_type_name(uint8_t d) {
    switch (d) {
        case ODID_DESC_TYPE_TEXT:            return "Text";
        case ODID_DESC_TYPE_EMERGENCY:       return "Emergency";
        case ODID_DESC_TYPE_EXTENDED_STATUS: return "Extended Status";
        default:                             return "Unknown";
    }
}

static const char *get_classification_name(uint8_t c) {
    switch (c) {
        case ODID_CLASSIFICATION_TYPE_UNDECLARED: return "Undeclared";
        case ODID_CLASSIFICATION_TYPE_EU:         return "EU";
        default:                                  return "Unknown";
    }
}

static const char *get_eu_category_name(uint8_t c) {
    switch (c) {
        case ODID_CATEGORY_EU_UNDECLARED: return "Undeclared";
        case ODID_CATEGORY_EU_OPEN:       return "Open";
        case ODID_CATEGORY_EU_SPECIFIC:   return "Specific";
        case ODID_CATEGORY_EU_CERTIFIED:  return "Certified";
        default:                          return "Unknown";
    }
}

static const char *get_eu_class_name(uint8_t c) {
    switch (c) {
        case ODID_CLASS_EU_UNDECLARED: return "Undeclared";
        case ODID_CLASS_EU_CLASS_0:    return "Class 0";
        case ODID_CLASS_EU_CLASS_1:    return "Class 1";
        case ODID_CLASS_EU_CLASS_2:    return "Class 2";
        case ODID_CLASS_EU_CLASS_3:    return "Class 3";
        case ODID_CLASS_EU_CLASS_4:    return "Class 4";
        case ODID_CLASS_EU_CLASS_5:    return "Class 5";
        case ODID_CLASS_EU_CLASS_6:    return "Class 6";
        default:                       return "Unknown";
    }
}

static const char *get_operator_loc_name(uint8_t t) {
    switch (t) {
        case ODID_OPERATOR_LOCATION_TYPE_TAKEOFF:   return "Takeoff";
        case ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS: return "Live GNSS";
        case ODID_OPERATOR_LOCATION_TYPE_FIXED:     return "Fixed";
        default:                                    return "Unknown";
    }
}

static const char *get_auth_type_name(uint8_t a) {
    switch (a) {
        case ODID_AUTH_NONE:                       return "None";
        case ODID_AUTH_UAS_ID_SIGNATURE:           return "UAS ID Signature";
        case ODID_AUTH_OPERATOR_ID_SIGNATURE:      return "Operator ID Signature";
        case ODID_AUTH_MESSAGE_SET_SIGNATURE:      return "Message Set Signature";
        case ODID_AUTH_NETWORK_REMOTE_ID:          return "Network Remote ID";
        case ODID_AUTH_SPECIFIC_AUTHENTICATION:    return "Specific Authentication";
        default:                                   return "Unknown";
    }
}

// GB 42590-2023 中国无人机分类名称 (参照 ORIP cn_rid.h)
static const char *get_cn_uav_category_name(uint8_t c) {
    switch (c) {
        case CN_UAV_CATEGORY_MICRO:  return "Micro (<250g)";
        case CN_UAV_CATEGORY_LIGHT:  return "Light (250g-4kg)";
        case CN_UAV_CATEGORY_SMALL:  return "Small (4-25kg)";
        case CN_UAV_CATEGORY_MEDIUM: return "Medium (25-150kg)";
        case CN_UAV_CATEGORY_LARGE:  return "Large (>150kg)";
        default:                     return "Unknown";
    }
}

// 传输方式名称
static const char *get_transport_name(uint8_t t) {
    switch (t) {
        case RID_TRANSPORT_BLUETOOTH_LEGACY:     return "BT Legacy";
        case RID_TRANSPORT_BLUETOOTH_LONG_RANGE: return "BT Long Range";
        case RID_TRANSPORT_WIFI_NAN:             return "Wi-Fi NAN";
        case RID_TRANSPORT_WIFI_BEACON:          return "Wi-Fi Beacon";
        default:                                  return "Unknown";
    }
}

// 协议类型名称
static const char *get_protocol_name(uint8_t p) {
    switch (p) {
        case RID_PROTOCOL_ASTM_F3411: return "ASTM F3411";
        case RID_PROTOCOL_ASD_STAN:   return "ASD-STAN";
        case RID_PROTOCOL_CN_RID:     return "GB 42590";
        default:                      return "Unknown";
    }
}

/* ================================================================
 * 公开名称映射函数 (供外部模块如 crid_scan_main.c 使用)
 * ================================================================ */

const char *crid_display_transport_name(uint8_t t) {
    return get_transport_name(t);
}

const char *crid_display_protocol_name(uint8_t p) {
    return get_protocol_name(p);
}

const char *crid_display_ua_type_name(uint8_t t) {
    return get_ua_type_name(t);
}

const char *crid_display_status_name(uint8_t s) {
    return get_status_name(s);
}

/* ================================================================
 * MAC 地址格式化
 * ================================================================ */

void crid_display_mac_str(const uint8_t *mac, char *buf, size_t size) {
    snprintf(buf, size, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* ================================================================
 * 摘要打印（带边框表格）
 * 使用 rid_*_t 分层结构体，与 ODID_UAS_Data 解耦
 * ================================================================ */

void crid_display_uav_summary(const uav_track_t *uav) {
    char mac_str[18];
    crid_display_mac_str(uav->mac, mac_str, sizeof(mac_str));

    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║          Remote ID UAV Detected                     ║\n");
    printf("╠══════════════════════════════════════════════════════╣\n");
    printf("║ MAC: %-47s ║\n", mac_str);
    printf("║ RSSI: %-4d dBm  Channel: %-2d  Msgs: %-6lu      ║\n",
           uav->last_rssi, uav->last_channel, (unsigned long)uav->msg_count);
    printf("║ Transport: %-42s ║\n", get_transport_name(uav->transport));
    printf("║ Protocol:  %-42s ║\n", get_protocol_name(uav->protocol));

    if (uav->basic_id.valid) {
        printf("║ UAS ID: %-45s ║\n", uav->basic_id.uas_id);
        printf("║ Type: %-49s ║\n", get_ua_type_name(uav->basic_id.ua_type));
    }

    if (uav->location.valid) {
        printf("║ Position: %11.7f°, %11.7f°       ║\n",
               uav->location.latitude, uav->location.longitude);
        printf("║ Altitude: %8.1f m   Speed: %6.1f m/s         ║\n",
               uav->location.altitude_baro, uav->location.speed_horizontal);
        printf("║ Status: %-46s ║\n", get_status_name(uav->location.status));
    }

    if (uav->self_id.valid) {
        printf("║ Self ID: %-45s ║\n", uav->self_id.description);
    }

    if (uav->operator_id.valid) {
        printf("║ Operator: %-44s ║\n", uav->operator_id.id);
    }

    if (uav->system.valid) {
        printf("║ Op Pos: %12.7f°, %12.7f°       ║\n",
               uav->system.operator_latitude, uav->system.operator_longitude);
    }

    uint32_t uptime = (esp_log_timestamp() - uav->first_seen_ms) / 1000;
    printf("║ Tracking: %lu sec                              ║\n", (unsigned long)uptime);
    printf("╚══════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/* ================================================================
 * 详细信息打印（所有字段）
 * 使用 rid_*_t 分层结构体
 * ================================================================ */

static void print_basic_id(const uav_track_t *uav) {
    // 分层视图：仅打印主 Basic ID
    if (uav->basic_id.valid) {
        ESP_LOGI(TAG, "  Basic ID:");
        ESP_LOGI(TAG, "    ID Type: %s", get_id_type_name(uav->basic_id.id_type));
        ESP_LOGI(TAG, "    UA Type: %s", get_ua_type_name(uav->basic_id.ua_type));
        ESP_LOGI(TAG, "    UAS ID:  '%s'", uav->basic_id.uas_id);
    }
    // 也打印额外的 Basic ID (index 1+)
    for (int i = 1; i < ODID_BASIC_ID_MAX_MESSAGES; i++) {
        if (!uav->uas_data.BasicIDValid[i]) continue;
        const ODID_BasicID_data *b = &uav->uas_data.BasicID[i];
        ESP_LOGI(TAG, "  Basic ID[%d]:", i);
        ESP_LOGI(TAG, "    ID Type: %s", get_id_type_name((uint8_t)b->IDType));
        ESP_LOGI(TAG, "    UA Type: %s", get_ua_type_name((uint8_t)b->UAType));
        ESP_LOGI(TAG, "    UAS ID:  '%s'", b->UASID);
    }
}

static void print_location(const uav_track_t *uav) {
    if (!uav->location.valid) return;

    ESP_LOGI(TAG, "  Location:");
    ESP_LOGI(TAG, "    Status:      %s", get_status_name(uav->location.status));
    ESP_LOGI(TAG, "    Latitude:    %.7f°", uav->location.latitude);
    ESP_LOGI(TAG, "    Longitude:   %.7f°", uav->location.longitude);
    ESP_LOGI(TAG, "    Alt Baro:    %.1f m", uav->location.altitude_baro);
    ESP_LOGI(TAG, "    Alt Geo:     %.1f m", uav->location.altitude_geo);
    ESP_LOGI(TAG, "    Height AGL:  %.1f m (%s)",
             uav->location.height, get_height_ref_name(uav->location.height_ref));
    ESP_LOGI(TAG, "    Direction:   %.1f°", uav->location.direction);
    ESP_LOGI(TAG, "    Speed H:     %.2f m/s", uav->location.speed_horizontal);
    ESP_LOGI(TAG, "    Speed V:     %.2f m/s", uav->location.speed_vertical);
    ESP_LOGI(TAG, "    Accuracy:");
    ESP_LOGI(TAG, "      Horiz: %s, Vert: %s, Baro: %s, Speed: %s",
             get_horiz_acc_name(uav->location.h_accuracy),
             get_vert_acc_name(uav->location.v_accuracy),
             get_vert_acc_name(uav->location.baro_accuracy),
             get_speed_acc_name(uav->location.speed_accuracy));
    ESP_LOGI(TAG, "    Timestamp:   %.1f s (accuracy: %.1f s)",
             uav->location.timestamp, decodeTimestampAccuracy((ODID_Timestamp_accuracy_t)uav->location.ts_accuracy));
}

static void print_system(const uav_track_t *uav) {
    if (!uav->system.valid) return;

    ESP_LOGI(TAG, "  System:");
    ESP_LOGI(TAG, "    Operator Location: %s", get_operator_loc_name(uav->system.operator_location_type));
    ESP_LOGI(TAG, "    Operator Position: %.7f°, %.7f°",
             uav->system.operator_latitude, uav->system.operator_longitude);
    ESP_LOGI(TAG, "    Operator Alt Geo: %.1f m", uav->system.operator_altitude_geo);
    ESP_LOGI(TAG, "    Classification:   %s", get_classification_name(uav->system.classification_type));
    if (uav->system.classification_type == ODID_CLASSIFICATION_TYPE_EU) {
        ESP_LOGI(TAG, "      EU Category: %s", get_eu_category_name(uav->system.category_eu));
        ESP_LOGI(TAG, "      EU Class:    %s", get_eu_class_name(uav->system.class_eu));
    }
    ESP_LOGI(TAG, "    Area Count: %d, Radius: %d m",
             uav->system.area_count, uav->system.area_radius);
    ESP_LOGI(TAG, "    Area Ceiling: %.1f m, Floor: %.1f m",
             uav->system.area_ceiling, uav->system.area_floor);
}

static void print_self_id(const uav_track_t *uav) {
    if (!uav->self_id.valid) return;
    ESP_LOGI(TAG, "  Self ID: %s - '%s'",
             get_desc_type_name(uav->self_id.description_type), uav->self_id.description);
}

static void print_operator_id(const uav_track_t *uav) {
    if (!uav->operator_id.valid) return;
    ESP_LOGI(TAG, "  Operator ID: Type=%d, ID='%s'",
             uav->operator_id.id_type, uav->operator_id.id);
}

static void print_auth(const uav_track_t *uav) {
    for (int i = 0; i < ODID_AUTH_MAX_PAGES; i++) {
        if (!uav->uas_data.AuthValid[i]) continue;
        const ODID_Auth_data *a = &uav->uas_data.Auth[i];
        ESP_LOGI(TAG, "  Auth Page %d: Type=%s", a->DataPage, get_auth_type_name((uint8_t)a->AuthType));
        if (a->DataPage == 0) {
            ESP_LOGI(TAG, "    Last Page: %d, Length: %d bytes", a->LastPageIndex, a->Length);
        }
    }
}

void crid_display_uav_detail(const uav_track_t *uav) {
    char mac_str[18];
    crid_display_mac_str(uav->mac, mac_str, sizeof(mac_str));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "=== UAV Detail: %s ===", mac_str);
    ESP_LOGI(TAG, "  RSSI: %d dBm, Channel: %d, Messages: %lu",
             uav->last_rssi, uav->last_channel, (unsigned long)uav->msg_count);
    ESP_LOGI(TAG, "  Transport: %s, Protocol: %s",
             get_transport_name(uav->transport), get_protocol_name(uav->protocol));

    print_basic_id(uav);
    print_location(uav);
    print_system(uav);
    print_self_id(uav);
    print_operator_id(uav);
    print_auth(uav);

    ESP_LOGI(TAG, "========================================");
}

/* ================================================================
 * 状态行打印（用于 monitor 列表）
 * ================================================================ */

void crid_display_uav_status(const uav_track_t *uav) {
    char mac_str[18];
    crid_display_mac_str(uav->mac, mac_str, sizeof(mac_str));
    uint32_t age_ms = esp_log_timestamp() - uav->last_seen_ms;

    if (uav->basic_id.valid && uav->location.valid) {
        ESP_LOGI(TAG, "  [%s] %s @ %.5f,%.5f (%.0fm, %.1fm/s) %s %lus ago",
                 mac_str, uav->basic_id.uas_id,
                 uav->location.latitude, uav->location.longitude,
                 uav->location.altitude_baro, uav->location.speed_horizontal,
                 get_transport_name(uav->transport),
                 (unsigned long)(age_ms / 1000));
    } else if (uav->basic_id.valid) {
        ESP_LOGI(TAG, "  [%s] %s (no location) %s %lus ago",
                 mac_str, uav->basic_id.uas_id,
                 get_transport_name(uav->transport),
                 (unsigned long)(age_ms / 1000));
    } else {
        ESP_LOGI(TAG, "  [%s] (no ID) %s %lus ago",
                 mac_str, get_transport_name(uav->transport),
                 (unsigned long)(age_ms / 1000));
    }
}
