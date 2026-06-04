/**
 * crid_rx_types.h — 接收端公共类型定义与配置常量
 *
 * ESP32 Remote ID Scanner
 * Standards: ASTM F3411-22a / ASD-STAN prEN 4709-002 / GB 42590-2023
 */

#ifndef CRID_RX_TYPES_H
#define CRID_RX_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "opendroneid.h"

/* ================================================================
 * 程序版本号 (开发阶段以编译时间为主)
 * ================================================================ */
#define CRID_VERSION_MAJOR     0
#define CRID_VERSION_MINOR     1
#define CRID_VERSION_PATCH     0
#define CRID_VERSION_STRING    "0.1.0-dev"
#define CRID_BUILD_DATE        __DATE__
#define CRID_BUILD_TIME        __TIME__

/* ================================================================
 * 协议标准 OUI 定义
 *
 * Wi-Fi Beacon 统一使用 FA:0B:BC（ASTM F3411-22a / ASD-STAN / GB 42590-2023）
 * 不同协议通过 Message Pack 首字节区分：ASTM=0xF2，国标=0xF1
 *
 * OUI 来源：
 *   RID_TRANSPORT_WIFI_BEACON_OUI  = 0xFA0BBC  (Wi-Fi Beacon)
 *   RID_TRANSPORT_WIFI_NAN_OUI     = 0x506F9A  (Wi-Fi NAN)
 *   RID_TRANSPORT_BLUETOOTH_OUI    = 0xFFFA    (Bluetooth Legacy/LR)
 *   厂商扩展：DJI 专用 OUI
 */

// ---- Wi-Fi Beacon OUI (0xFA0BBC) ----
#define OUI_BEACON_0  0xFA
#define OUI_BEACON_1  0x0B
#define OUI_BEACON_2  0xBC

// ---- Wi-Fi NAN OUI (0x506F9A) ----
#define OUI_NAN_0  0x50
#define OUI_NAN_1  0x6F
#define OUI_NAN_2  0x9A

// ---- Bluetooth OUI (0xFFFA) ----
// BLE Legacy Advertising / Long Range
#define OUI_BLE_0  0xFF
#define OUI_BLE_1  0xFF
#define OUI_BLE_2  0xFA

// ---- 厂商扩展 OUI ----
// DJI 厂商
#define OUI_DJI_60_0  0x60
#define OUI_DJI_60_1  0x60
#define OUI_DJI_60_2  0x1F
#define OUI_DJI_48_0  0x48
#define OUI_DJI_48_1  0x1C
#define OUI_DJI_48_2  0xB9
#define OUI_DJI_34_0  0x34
#define OUI_DJI_34_1  0xD2
#define OUI_DJI_34_2  0x62

// 判断是否为任一 Remote ID OUI
// 注：Wi-Fi Beacon 标准 OUI 统一为 FA:0B:BC（ASTM / ASD-STAN / GB 42590 均使用此 OUI）
#define IS_RID_OUI(o0, o1, o2) \
    (((o0) == OUI_BEACON_0   && (o1) == OUI_BEACON_1   && (o2) == OUI_BEACON_2)   || \
     ((o0) == OUI_NAN_0      && (o1) == OUI_NAN_1      && (o2) == OUI_NAN_2)      || \
     ((o0) == OUI_BLE_0      && (o1) == OUI_BLE_1      && (o2) == OUI_BLE_2)      || \
     ((o0) == OUI_DJI_60_0   && (o1) == OUI_DJI_60_1   && (o2) == OUI_DJI_60_2)   || \
     ((o0) == OUI_DJI_48_0   && (o1) == OUI_DJI_48_1   && (o2) == OUI_DJI_48_2)   || \
     ((o0) == OUI_DJI_34_0   && (o1) == OUI_DJI_34_1   && (o2) == OUI_DJI_34_2))

// 根据 OUI 获取传输类型
#define GET_RID_TRANSPORT(o0, o1, o2) \
    (((o0) == OUI_BEACON_0 && (o1) == OUI_BEACON_1 && (o2) == OUI_BEACON_2) ? RID_TRANSPORT_WIFI_BEACON : \
     ((o0) == OUI_NAN_0    && (o1) == OUI_NAN_1    && (o2) == OUI_NAN_2)    ? RID_TRANSPORT_WIFI_NAN    : \
     ((o0) == OUI_BLE_0    && (o1) == OUI_BLE_1    && (o2) == OUI_BLE_2)    ? RID_TRANSPORT_BLUETOOTH_LEGACY : \
     RID_TRANSPORT_WIFI_BEACON)  // 默认按 Beacon 处理

/* ================================================================
 * 传输方式定义 (基于 ASTM F3411-22a Section 5.4)
 * ================================================================ */

typedef enum {
    RID_TRANSPORT_BLUETOOTH_LEGACY     = 0,
    RID_TRANSPORT_BLUETOOTH_LONG_RANGE = 1,
    RID_TRANSPORT_WIFI_NAN             = 2,
    RID_TRANSPORT_WIFI_BEACON          = 3,
} rid_transport_t;

// 传输方式最大 payload 大小
#define RID_PAYLOAD_BLE_LEGACY_MAX     25
#define RID_PAYLOAD_BLE_LONG_RANGE_MAX 255
#define RID_PAYLOAD_WIFI_NAN_MAX       255
#define RID_PAYLOAD_WIFI_BEACON_MAX    250

// Wi-Fi Beacon Vendor Type (ASD-STAN)
#define RID_WIFI_BEACON_VENDOR_TYPE    0x0D

/* ================================================================
 * 协议类型枚举 (参照 ORIP types.h)
 * ================================================================ */

typedef enum {
    RID_PROTOCOL_UNKNOWN    = 0,
    RID_PROTOCOL_ASTM_F3411 = 1,
    RID_PROTOCOL_ASD_STAN   = 2,
    RID_PROTOCOL_GB42590    = 3,   // 中国 GB 42590-2023
} rid_protocol_t;

/* ================================================================
 * 分层数据结构 (参照 ORIP types.h)
 *
 * 设计思想：
 *   - 与底层 opendroneid 库的 ODID_UAS_Data 解耦
 *   - 每个结构体有独立的 valid 标志
 *   - 便于未来扩展 GB 42590 专有字段
 *   - 显示层直接读取这些结构体，无需理解 ODID_* 枚举
 * ================================================================ */

// 位置/矢量数据
typedef struct {
    bool     valid;
    double   latitude;           // 纬度 (-90° ~ 90°)
    double   longitude;          // 经度 (-180° ~ 180°)
    float    altitude_baro;      // 气压高度 (m)
    float    altitude_geo;       // 大地高度 (m, WGS84)
    float    height;             // 相对高度 (m)
    uint8_t  height_ref;         // 0=Takeoff, 1=Ground (ODID_Height_reference_t)
    float    speed_horizontal;   // 水平速度 (m/s)
    float    speed_vertical;     // 垂直速度 (m/s)
    float    direction;          // 航向 (0°~360°)
    uint8_t  status;             // UAV 状态 (ODID_status_t)
    uint8_t  h_accuracy;         // 水平精度 (ODID_Horizontal_accuracy_t)
    uint8_t  v_accuracy;         // 垂直精度 (ODID_Vertical_accuracy_t)
    uint8_t  baro_accuracy;      // 气压精度 (ODID_Vertical_accuracy_t)
    uint8_t  speed_accuracy;     // 速度精度 (ODID_Speed_accuracy_t)
    uint8_t  ts_accuracy;        // 时间戳精度 (ODID_Timestamp_accuracy_t)
    float    timestamp;          // 相对于整小时的秒数 (0.1s 单位)
} rid_location_t;

// 系统信息（操作员/飞行员数据）
typedef struct {
    bool     valid;
    uint8_t  operator_location_type;  // 操作员位置类型 (ODID_operator_location_type_t)
    double   operator_latitude;       // 操作员纬度
    double   operator_longitude;      // 操作员经度
    float    operator_altitude_geo;   // 操作员大地高度 (m)
    uint16_t area_count;              // 区域内飞行器数量
    uint16_t area_radius;             // 区域半径 (m)
    float    area_ceiling;            // 运行区域上限 (m)
    float    area_floor;              // 运行区域下限 (m)
    uint8_t  classification_type;     // 分类类型 (ODID_classification_type_t)
    uint8_t  category_eu;             // EU 类别 (ODID_category_EU_t)
    uint8_t  class_eu;                // EU 等级 (ODID_class_EU_t)
    uint32_t timestamp;               // Unix 时间戳
} rid_system_info_t;

// Self-ID 消息
typedef struct {
    bool     valid;
    uint8_t  description_type;        // 描述类型 (ODID_desctype_t)
    char     description[24];         // 自由格式文本 (最多 23 字符 + null)
} rid_self_id_t;

// 操作员 ID 消息
typedef struct {
    bool     valid;
    uint8_t  id_type;                 // ID 类型 (ODID_operatorIdType_t)
    char     id[21];                  // 操作员 ID (最多 20 字符 + null)
} rid_operator_id_t;

// Basic ID 消息
typedef struct {
    bool     valid;
    uint8_t  id_type;                 // ID 类型 (ODID_idtype_t)
    uint8_t  ua_type;                 // UA 类型 (ODID_uatype_t)
    char     uas_id[21];              // UAS ID (最多 20 字符 + null)
} rid_basic_id_t;

/* ================================================================
 * 扫描配置常量
 * ================================================================ */

#define MAX_TRACKED_UAVS        100         // 最多同时追踪的无人机数量
#define SNIFFER_QUEUE_SIZE      32          // sniffer 消息队列深度
#define PARSER_TASK_STACK       8192        // 解析任务栈大小
#define MONITOR_TASK_STACK      4096        // 监控任务栈大小
#define PARSER_TASK_PRIO        4           // 解析任务优先级
#define MONITOR_TASK_PRIO       3           // 监控任务优先级
#define CH_HOLD_TASK_STACK      3072        // 信道保持任务栈大小
#define CH_HOLD_TASK_PRIO       2           // 信道保持任务优先级

// 锁定监听信道
#define FIXED_CHANNEL           6

// 无人机超时时间 (毫秒)
#define UAV_TIMEOUT_MS          300000      // 5 分钟

/* ================================================================
 * 802.11 MAC 头结构 (packed)
 * ================================================================ */

typedef struct __attribute__((__packed__)) {
    uint16_t frame_ctrl;
    uint16_t duration_id;
    uint8_t  addr1[6];
    uint8_t  addr2[6];
    uint8_t  addr3[6];
    uint16_t seq_ctrl;
} wifi_mac_hdr_t;

/* ================================================================
 * Sniffer 消息队列元素
 * ================================================================ */

// 消息类型枚举
typedef enum {
    MSG_TYPE_RID = 0,           // RID Vendor IE
    MSG_TYPE_NON_RID_VENDOR,    // 非 RID Vendor IE (ID=221, 其他 OUI)
    MSG_TYPE_BEACON_NO_VENDOR,  // 普通 Beacon（不含 Vendor IE 或诊断采样）
} sniffer_msg_type_t;

typedef struct {
    uint8_t  src_mac[6];        // 源 MAC 地址
    int8_t   rssi;              // 信号强度
    uint8_t  channel;           // 接收信道
    uint32_t timestamp_ms;      // 接收时间戳
    uint8_t  oui_type;          // OUI 类型字节
    uint16_t data_len;          // 有效数据长度
    uint8_t  data[256];         // Vendor Specific IE 数据 (最大255)
    bool     is_rid;            // 是否为 RID 消息
    uint8_t  oui[3];            // 完整 OUI 字节 (用于非 RID 诊断)
    sniffer_msg_type_t msg_type; // 消息类型
    uint8_t  ssid_len;          // SSID 长度 (0-32)
    uint8_t  ssid[32];          // SSID (用于 Beacon 诊断)
    bool     has_vendor_ie;     // 是否包含 Vendor IE (ID=221)
} sniffer_msg_t;

/* ================================================================
 * 单个无人机追踪状态
 * ================================================================ */

typedef struct {
    uint8_t  mac[6];                    // 源 MAC 地址
    bool     active;                    // 是否活跃
    uint32_t last_seen_ms;              // 最后收到信号的时间
    int8_t   last_rssi;                 // 最近 RSSI
    uint8_t  last_channel;              // 最近所在信道
    uint8_t  oui[3];                    // Remote ID OUI 字节
    uint8_t  oui_type;                  // OUI Type 字节
    uint8_t  transport;                 // 传输方式 (rid_transport_t)
    uint8_t  protocol;                  // 协议类型 (rid_protocol_t)

    // 使用 opendroneid 库的标准数据结构 (底层解码目标)
    ODID_UAS_Data uas_data;             // 完整无人机数据
    ODID_MessagePack_data last_pack;    // 最近接收的消息包

    // 分层数据视图 (从 uas_data 提取，供显示层直接使用)
    rid_basic_id_t    basic_id;         // Basic ID
    rid_location_t    location;         // 位置/速度
    rid_system_info_t system;           // 系统/操作员信息
    rid_self_id_t     self_id;          // Self-ID 描述
    rid_operator_id_t operator_id;      // 操作员 ID

    // 统计
    uint32_t msg_count;                 // 累计消息数
    uint32_t first_seen_ms;             // 首次发现时间
} uav_track_t;

/* ================================================================
 * 全局统计
 * ================================================================ */

typedef struct {
    volatile uint32_t total_packets;
    volatile uint32_t mgmt_frames;
    volatile uint32_t rid_detections;
    volatile uint32_t queue_overflows;
    volatile uint32_t non_rid_vendor_ie;   // 非 RID OUI 的 Vendor IE 计数
    volatile uint32_t beacon_count;         // Beacon 帧总数
} sniffer_stats_t;

#endif // CRID_RX_TYPES_H
