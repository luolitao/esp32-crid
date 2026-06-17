/**
 * crid_web_api.c — HTTP JSON API 处理器
 *
 * 实现 /api/ 系列端点，返回系统、设备、OTA、追踪、网络、扫描状态。
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <esp_http_server.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include "esp_system.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "crid_ota_web.h"
#include "crid_ota_internal.h"
#include "crid_web_api.h"
#include "crid_rx_types.h"
#include "crid_tracker.h"
#include "crid_sniffer.h"
#include "crid_display.h"

/* ---------- helpers ---------- */

static const char *chip_name(esp_chip_model_t model)
{
    switch (model) {
        case CHIP_ESP32:   return "ESP32";
        case CHIP_ESP32S2: return "ESP32-S2";
        case CHIP_ESP32S3: return "ESP32-S3";
        case CHIP_ESP32C3: return "ESP32-C3";
        default:           return "Unknown";
    }
}

/* ---------- /api/system/info ---------- */

esp_err_t crid_web_api_system_info(httpd_req_t *req)
{
    char buf[512];

    esp_chip_info_t ci;
    esp_chip_info(&ci);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_heap  = esp_get_minimum_free_heap_size();

    snprintf(buf, sizeof(buf),
        "{"
        "\"chip_model\":\"%s\","
        "\"chip_revision\":%d,"
        "\"chip_cores\":%d,"
        "\"flash_size\":%lu,"
        "\"free_heap\":%lu,"
        "\"min_free_heap\":%lu,"
        "\"version\":\"%s\","
        "\"build_date\":\"%s\","
        "\"build_time\":\"%s\""
        "}",
        chip_name(ci.model), ci.revision, ci.cores,
        (unsigned long)flash_size,
        (unsigned long)free_heap, (unsigned long)min_heap,
        CRID_VERSION_STRING, CRID_BUILD_DATE, CRID_BUILD_TIME);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

/* ---------- /api/device/status ---------- */

esp_err_t crid_web_api_device_status(httpd_req_t *req)
{
    char buf[256];

    wifi_ap_record_t ap;
    char ssid[33] = "Not connected";
    int  rssi     = 0;

    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        strncpy(ssid, (char *)ap.ssid, sizeof(ssid) - 1);
        ssid[sizeof(ssid) - 1] = '\0';
        rssi = ap.rssi;
    }

    snprintf(buf, sizeof(buf),
        "{"
        "\"uptime_seconds\":%lu,"
        "\"wifi_ssid\":\"%s\","
        "\"wifi_rssi\":%d,"
        "\"heap_free\":%lu,"
        "\"heap_min_free\":%lu"
        "}",
        (unsigned long)(esp_log_timestamp() / 1000),
        ssid, rssi,
        (unsigned long)esp_get_free_heap_size(),
        (unsigned long)esp_get_minimum_free_heap_size());

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

/* ---------- /api/ota/progress ---------- */

esp_err_t crid_web_api_ota_progress(httpd_req_t *req)
{
    char buf[512];

    int total    = crid_ota_get_total_size();
    int received = crid_ota_get_received_size();
    int progress = (total > 0) ? (received * 100 / total) : 0;

    snprintf(buf, sizeof(buf),
        "{"
        "\"state\":\"%s\","
        "\"progress\":%d,"
        "\"received\":%d,"
        "\"total\":%d,"
        "\"error\":\"%s\""
        "}",
        crid_ota_state_name(),
        progress, received, total,
        crid_ota_get_error_string());

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

/* ---------- /api/system/status ---------- */

esp_err_t crid_web_api_system_status(httpd_req_t *req)
{
    char buf[512];

    esp_chip_info_t ci;
    esp_chip_info(&ci);

    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_heap  = esp_get_minimum_free_heap_size();

    snprintf(buf, sizeof(buf),
        "{"
        "\"system\":{"
            "\"chip_model\":\"%s\","
            "\"chip_revision\":%d,"
            "\"chip_cores\":%d,"
            "\"free_heap\":%lu,"
            "\"min_free_heap\":%lu,"
            "\"uptime_seconds\":%lu,"
            "\"version\":\"%s\","
            "\"build_date\":\"%s\","
            "\"build_time\":\"%s\""
        "},"
        "\"ota\":{"
            "\"state\":\"%s\","
            "\"progress\":%d"
        "}"
        "}",
        chip_name(ci.model), ci.revision, ci.cores,
        (unsigned long)free_heap, (unsigned long)min_heap,
        (unsigned long)(esp_log_timestamp() / 1000),
        CRID_VERSION_STRING, CRID_BUILD_DATE, CRID_BUILD_TIME,
        crid_ota_state_name(),
        crid_ota_get_progress());

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

/* ---------- /api/uav/tracking ---------- */

esp_err_t crid_web_api_uav_tracking(httpd_req_t *req)
{
    char buf[2048];
    int  off = 0;

    SemaphoreHandle_t mutex = crid_tracker_get_mutex();
    uav_track_t      *table = crid_tracker_get_table();
    int active_count        = crid_tracker_get_active_count();

    off += snprintf(buf + off, sizeof(buf) - off,
        "{\"tracking\":{"
        "\"max_uavs\":%d,"
        "\"active_uavs\":%d,"
        "\"uavs\":[",
        MAX_TRACKED_UAVS, active_count);

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool first = true;
        for (int i = 0; i < MAX_TRACKED_UAVS; i++) {
            if (!table[i].active) continue;

            if (!first) {
                off += snprintf(buf + off, sizeof(buf) - off, ",");
            }
            first = false;

            char mac_str[18];
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                     table[i].mac[0], table[i].mac[1], table[i].mac[2],
                     table[i].mac[3], table[i].mac[4], table[i].mac[5]);

            off += snprintf(buf + off, sizeof(buf) - off,
                "{"
                "\"mac\":\"%s\","
                "\"rssi\":%d,"
                "\"channel\":%u,"
                "\"transport\":\"%s\","
                "\"protocol\":\"%s\","
                "\"messages\":%lu,"
                "\"active\":true",
                mac_str,
                table[i].last_rssi,
                table[i].last_channel,
                crid_display_transport_name(table[i].transport),
                crid_display_protocol_name(table[i].protocol),
                (unsigned long)table[i].msg_count);

            if (table[i].basic_id.valid) {
                off += snprintf(buf + off, sizeof(buf) - off,
                    ",\"basic_id\":{"
                    "\"id_type\":\"%s\","
                    "\"ua_type\":\"%s\","
                    "\"uas_id\":\"%s\""
                    "}",
                    crid_display_ua_type_name(table[i].basic_id.id_type),
                    crid_display_ua_type_name(table[i].basic_id.ua_type),
                    table[i].basic_id.uas_id);
            }

            if (table[i].location.valid) {
                off += snprintf(buf + off, sizeof(buf) - off,
                    ",\"location\":{"
                    "\"latitude\":%.7f,"
                    "\"longitude\":%.7f,"
                    "\"altitude_baro\":%.1f,"
                    "\"speed_h\":%.2f,"
                    "\"status\":\"%s\""
                    "}",
                    table[i].location.latitude,
                    table[i].location.longitude,
                    table[i].location.altitude_baro,
                    table[i].location.speed_horizontal,
                    crid_display_status_name(table[i].location.status));
            }

            off += snprintf(buf + off, sizeof(buf) - off, "}");
        }
        xSemaphoreGive(mutex);
    }

    off += snprintf(buf + off, sizeof(buf) - off, "]}}");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

/* ---------- /api/network/status ---------- */

esp_err_t crid_web_api_network_status(httpd_req_t *req)
{
    char buf[512];

    wifi_ap_record_t ap;
    char ssid[33] = "Not connected";
    int  rssi     = 0;
    bool connected= false;

    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        strncpy(ssid, (char *)ap.ssid, sizeof(ssid) - 1);
        ssid[sizeof(ssid) - 1] = '\0';
        rssi = ap.rssi;
        connected = true;
    }

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),
                          &ip_info);

    char gw_str[16], nm_str[16];
    snprintf(gw_str,  sizeof(gw_str),  IPSTR, IP2STR(&ip_info.gw));
    snprintf(nm_str,  sizeof(nm_str),  IPSTR, IP2STR(&ip_info.netmask));

    time_t now;
    time(&now);

    snprintf(buf, sizeof(buf),
        "{"
        "\"network\":{"
            "\"ssid\":\"%s\","
            "\"rssi\":%d,"
            "\"connected\":%s,"
            "\"ip_address\":\"" IPSTR "\","
            "\"gateway\":\"%s\","
            "\"netmask\":\"%s\","
            "\"timestamp\":%lld"
        "}"
        "}",
        ssid, rssi, connected ? "true" : "false",
        IP2STR(&ip_info.ip),
        gw_str, nm_str,
        (long long)now);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

/* ---------- /api/scan/stats ---------- */

esp_err_t crid_web_api_scan_stats(httpd_req_t *req)
{
    char buf[256];

    sniffer_stats_t *stats = crid_sniffer_get_stats();
    int active_count       = crid_tracker_get_active_count();

    snprintf(buf, sizeof(buf),
        "{"
        "\"scan_stats\":{"
            "\"total_packets\":%lu,"
            "\"management_frames\":%lu,"
            "\"rid_detections\":%lu,"
            "\"queue_overflows\":%lu,"
            "\"non_rid_vendor_ie\":%lu,"
            "\"beacon_count\":%lu,"
            "\"active_uavs\":%d"
        "}"
        "}",
        (unsigned long)stats->total_packets,
        (unsigned long)stats->mgmt_frames,
        (unsigned long)stats->rid_detections,
        (unsigned long)stats->queue_overflows,
        (unsigned long)stats->non_rid_vendor_ie,
        (unsigned long)stats->beacon_count,
        active_count);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}
