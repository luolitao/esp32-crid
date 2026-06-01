#ifndef CRID_CONFIG_H
#define CRID_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- 报文类型 (符合 ASTM F3411 / ASD-STAN 4709-002) ---
#define MSG_TYPE_BASIC_ID    0x0  // 基本 ID 报文
#define MSG_TYPE_LOCATION    0x1  // 位置向量报文
#define MSG_TYPE_AUTH        0x2  // 认证报文
#define MSG_TYPE_SELF_DESC   0x3  // 运行描述报文
#define MSG_TYPE_SYSTEM      0x4  // 系统报文
#define MSG_TYPE_OPERATOR_ID 0x5  // 操作员 ID 报文
#define MSG_TYPE_PACKED      0xF  // 报文打包

// --- ID 类型 ---
#define ID_TYPE_NONE             0
#define ID_TYPE_SERIAL_NUMBER    1
#define ID_TYPE_CAA_REGISTRATION 2
#define ID_TYPE_UTM_ASSIGNED     3
#define ID_TYPE_SPECIFIC_SESSION 4

// --- UA 类型 ---
#define UA_TYPE_NONE             0
#define UA_TYPE_AEROPLANE        1
#define UA_TYPE_HELICOPTER       2
#define UA_TYPE_GYROPLANE        3
#define UA_TYPE_HYBRID_LIFT      4
#define UA_TYPE_ORNITHOPTER      5
#define UA_TYPE_GLIDER           6
#define UA_TYPE_KITE             7
#define UA_TYPE_FREE_BALLOON     8
#define UA_TYPE_CAPTIVE_BALLOON  9
#define UA_TYPE_AIRSHIP          10
#define UA_TYPE_FALL_AWAY        11
#define UA_TYPE_ROCKET           12
#define UA_TYPE_TETHERED         13
#define UA_TYPE_GROUND_OBSTACLE  14
#define UA_TYPE_OTHER            15

// --- 运行状态 (ASTM F3411) ---
#define STATUS_UNDECLARED         0
#define STATUS_GROUND             1
#define STATUS_AIRBORNE           2
#define STATUS_EMERGENCY          3
#define STATUS_REMOTE_ID_FAIL     4

// --- 高度参考类型 ---
#define HEIGHT_REF_OVER_TAKEOFF   0
#define HEIGHT_REF_OVER_GROUND    1

// --- 操作员位置类型 ---
#define OP_LOC_TYPE_TAKEOFF       0
#define OP_LOC_TYPE_LIVE_GNSS     1
#define OP_LOC_TYPE_FIXED         2

// --- 分类类型 ---
#define CLASSIFICATION_UNDECLARED 0
#define CLASSIFICATION_EU         1

// --- 描述类型 ---
#define DESC_TYPE_TEXT            0
#define DESC_TYPE_EMERGENCY       1
#define DESC_TYPE_EXTENDED_STATUS 2

// --- 报文常量 ---
#define CRID_MESSAGE_SIZE      25     // 每条报文 25 字节
#define CRID_UAS_ID_MAX_LEN    20     // UAS ID 最大长度
#define CRID_SSID_MAX_LEN      31     // SSID 最大长度

// --- 中国 C-RID 标准 OUI 和类型 ---
#define CRID_OUI_0 0xFA
#define CRID_OUI_1 0x0B
#define CRID_OUI_2 0xBC
#define CRID_VENDOR_TYPE 0x0D

// --- 默认配置 ---
#define DEFAULT_WIFI_CHANNEL       6
#define DEFAULT_BEACON_INTERVAL_MS 1000
#define DEFAULT_MAX_FRAME_SIZE     512

// --- 配置结构体 ---
typedef struct {
    char uas_id[CRID_UAS_ID_MAX_LEN + 1];  // UAS ID / 无人机唯一标识 (序列号，填入 Basic ID 报文)
    uint8_t id_type;                         // ID 类型 (0-4)
    uint8_t ua_type;                         // 无人机类型 (0-15)
    char drone_name[CRID_UAS_ID_MAX_LEN + 1]; // 无人机型号 (Self-ID 描述报文)
    float latitude;                          // 纬度
    float longitude;                         // 经度
    float altitude_msl;                      // 海拔高度 (m)
    float altitude_agl;                      // 相对地面高度 (m)
    float speed_horizontal;                  // 水平速度 (m/s)
    float speed_vertical;                    // 垂直速度 (m/s)
    float heading;                           // 航向 (度)
    uint8_t status;                          // 运行状态 (0-4)
    float operator_lat;                      // 操作员纬度
    float operator_lon;                      // 操作员经度
    float operator_alt;                      // 操作员高度 (m)
    char operator_id[CRID_UAS_ID_MAX_LEN + 1]; // 操作员 ID
    uint8_t operator_location_type;          // 操作员位置类型 (0=Takeoff, 1=Live GNSS, 2=Fixed)
    uint8_t classification_type;             // 分类类型 (0=Undeclared, 1=EU)
    uint8_t category_eu;                     // EU 类别 (0=Undeclared, 1=Open, 2=Specific, 3=Certified)
    uint8_t class_eu;                        // EU 等级 (0-7)
    uint8_t height_type;                     // 高度参考类型 (0=Takeoff, 1=Ground)
    uint8_t mac_address[6];                  // MAC 地址
    char ssid[CRID_SSID_MAX_LEN + 1];        // SSID
    uint8_t channel;                         // 通道
    uint8_t message_counter;                 // 消息计数器 (0-255, 循环)

    // 巡游参数
    float base_latitude;                     // 基准纬度
    float base_longitude;                    // 基准经度
    float base_altitude_msl;                 // 基准海拔高度 (m)
    float patrol_radius_lat;                 // 纬度方向巡游半径
    float patrol_radius_lon;                 // 经度方向巡游半径
    float patrol_speed;                      // 巡游速度参数
    float time_counter;                      // 时间计数器
} cn_crid_config_t;

/**
 * @brief 初始化中国 C-RID 配置为默认值
 * @param config 指向配置结构体的指针
 */
void crid_config_init_default(cn_crid_config_t *config);

/**
 * @brief 更新位置数据
 * @param config 指向配置结构体的指针
 */
void crid_config_update_position(cn_crid_config_t *config,
                                 float lat, float lon,
                                 float alt_msl, float alt_agl,
                                 float speed_h, float speed_v,
                                 float heading);

#ifdef __cplusplus
}
#endif

#endif // CRID_CONFIG_H
