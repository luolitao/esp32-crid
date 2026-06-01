#include "crid_config.h"
#include <string.h>
#include <stdio.h>
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"

static const char *TAG = "CN_C-RID_CFG";

void crid_config_init_default(cn_crid_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "config is NULL");
        return;
    }

    memset(config, 0, sizeof(cn_crid_config_t));

    // --- 从硬件获取 MAC 地址 ---
    esp_err_t mac_ret = esp_efuse_mac_get_default(config->mac_address);
    if (mac_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC address, using fallback");
        config->mac_address[0] = 0x24;
        config->mac_address[1] = 0x0A;
        config->mac_address[2] = 0xC4;
        config->mac_address[3] = 0x12;
        config->mac_address[4] = 0x34;
        config->mac_address[5] = 0x57;
    }

    // 提取 MAC 地址最后 4 位（即后 2 字节）作为后缀
    // 例如 MAC 24:0A:C4:12:34:56 -> 后缀 "3456"
    char mac_suffix[5];
    snprintf(mac_suffix, sizeof(mac_suffix), "%02X%02X",
             config->mac_address[4], config->mac_address[5]);

    // --- UAS ID / 无人机唯一标识: 前缀 "CRID-" + MAC 后 4 位 ---
    snprintf(config->uas_id, CRID_UAS_ID_MAX_LEN + 1, "ESP32-CRID-%s", mac_suffix);

    config->id_type = ID_TYPE_SERIAL_NUMBER;
    config->ua_type = UA_TYPE_HELICOPTER;

    // 越秀山坐标
    config->latitude = 23.14287f;
    config->longitude = 113.26026f;
    config->altitude_msl = 50.0f;
    config->altitude_agl = 50.0f;
    config->speed_horizontal = 1.0f;
    config->speed_vertical = 0.0f;
    config->heading = 45.0f;
    config->status = STATUS_AIRBORNE;

    config->operator_lat = 23.14f;
    config->operator_lon = 113.26f;
    config->operator_alt = 10.0f;

    // 飞手名字: 前缀 "OP-CAAC-" + MAC 后 4 位
    snprintf(config->operator_id, CRID_UAS_ID_MAX_LEN + 1, "ESP32-OP-%s", mac_suffix);

    // 无人机名字/型号 (Self-ID 描述): 填写为 ESP32S3
    strncpy(config->drone_name, "ESP32S3", CRID_UAS_ID_MAX_LEN);
    config->drone_name[CRID_UAS_ID_MAX_LEN] = '\0';

    config->operator_location_type = OP_LOC_TYPE_LIVE_GNSS; // Dynamic
    config->classification_type = CLASSIFICATION_UNDECLARED;
    config->category_eu = 0;
    config->class_eu = 0;
    config->height_type = HEIGHT_REF_OVER_TAKEOFF;

    // SSID 后缀也用 MAC 后 4 位
    snprintf(config->ssid, CRID_SSID_MAX_LEN + 1, "ESP32-CRID-%s", mac_suffix);

    config->channel = DEFAULT_WIFI_CHANNEL;
    config->message_counter = 0;

    // 巡游参数
    config->base_latitude = config->latitude;
    config->base_longitude = config->longitude;
    config->base_altitude_msl = config->altitude_msl;
    config->patrol_radius_lat = 0.00005f;  // 约 5.5 米
    config->patrol_radius_lon = 0.00004f;  // 约 4.4 米
    config->patrol_speed = 0.2f;
    config->time_counter = 0.0f;

    ESP_LOGI(TAG, "China C-RID configuration initialized");
    ESP_LOGI(TAG, "  MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             config->mac_address[0], config->mac_address[1], config->mac_address[2],
             config->mac_address[3], config->mac_address[4], config->mac_address[5]);
    ESP_LOGI(TAG, "  UAS ID: %s", config->uas_id);
    ESP_LOGI(TAG, "  Drone Model (Self-ID): %s", config->drone_name);
    ESP_LOGI(TAG, "  Operator ID: %s", config->operator_id);
    ESP_LOGI(TAG, "  ID Type: %d (Serial Number)", config->id_type);
    ESP_LOGI(TAG, "  UA Type: %d (Helicopter/Multirotor)", config->ua_type);
    ESP_LOGI(TAG, "  Position: %.6f, %.6f", config->latitude, config->longitude);
}

void crid_config_update_position(cn_crid_config_t *config,
                                  float lat, float lon,
                                  float alt_msl, float alt_agl,
                                  float speed_h, float speed_v,
                                  float heading) {
    if (config == NULL) return;

    config->latitude = lat;
    config->longitude = lon;
    config->altitude_msl = alt_msl;
    config->altitude_agl = alt_agl;
    config->speed_horizontal = speed_h;
    config->speed_vertical = speed_v;
    config->heading = heading;

    ESP_LOGI(TAG, "Position updated: %.6f, %.6f, Alt: %.2fm, Hdg: %.1f",
             config->latitude, config->longitude,
             config->altitude_msl, config->heading);
}
