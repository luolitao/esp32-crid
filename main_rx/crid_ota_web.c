#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <esp_chip_info.h>
#include <nvs_flash.h>
#include <esp_mac.h>
#include <esp_app_desc.h>
#include <esp_log.h>
#include <esp_wifi.h>   

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs.h"
#include "esp_netif.h"

#include <time.h>
#include "crid_ota_web.h"
#include "crid_rx_types.h"
#include "crid_tracker.h"
#include "crid_sniffer.h"
#include "crid_display.h"

static const char *TAG = "OTA_WEB";

/* HTTP Server handle */
static httpd_handle_t g_httpd_handle = NULL;

static ota_state_t g_ota_state = OTA_IDLE;
static int g_ota_update_size = 0;
static int g_ota_received_size = 0;
static char g_ota_error_msg[256] = {0};

/* OTA update handler */
static esp_err_t ota_handler(httpd_req_t *req)
{
    char *buf = NULL;
    ssize_t read_bytes = 0;
    esp_err_t err = ESP_OK;

    /* Check if request is multipart form data */
    char content_type[256];
    size_t content_type_len = sizeof(content_type);
    err = httpd_req_get_hdr_value_str(req, "Content-Type", content_type, content_type_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get Content-Type header");
        return httpd_resp_send_500(req);
    }

    /* Check if it's multipart form data */
    if (strstr(content_type, "multipart/form-data") == NULL) {
        ESP_LOGE(TAG, "Not multipart/form-data");
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Bad Request", HTTPD_RESP_USE_STRLEN);
    }

    /* Get the size of the firmware image */
    char content_length[32];
    size_t content_length_len = sizeof(content_length);
    err = httpd_req_get_hdr_value_str(req, "Content-Length", content_length, content_length_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get Content-Length header");
        return httpd_resp_send_500(req);
    }

    int fw_size = atoi(content_length);
    if (fw_size <= 0) {
        ESP_LOGE(TAG, "Invalid firmware size: %d", fw_size);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Bad Request", HTTPD_RESP_USE_STRLEN);
    }

    ESP_LOGI(TAG, "OTA update started, firmware size: %d bytes", fw_size);

    /* Start OTA update */
    const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    if (partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition found");
        return httpd_resp_send_500(req);
    }

    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             partition->subtype, partition->address);

    esp_ota_handle_t ota_handle;
    err = esp_ota_begin(partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        snprintf(g_ota_error_msg, sizeof(g_ota_error_msg), "OTA begin failed: %s", esp_err_to_name(err));
        return httpd_resp_send_500(req);
    }

    g_ota_state = OTA_STARTED;
    g_ota_update_size = fw_size;
    g_ota_received_size = 0;

    /* Read firmware data and write to flash */
    buf = malloc(1024);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        esp_ota_end(ota_handle);
        snprintf(g_ota_error_msg, sizeof(g_ota_error_msg), "Memory allocation failed");
        return httpd_resp_send_500(req);
    }

    while (g_ota_received_size < fw_size) {
        read_bytes = httpd_req_recv(req, buf, MIN(1024, fw_size - g_ota_received_size));
        if (read_bytes <= 0) {
            ESP_LOGE(TAG, "Error receiving data");
            free(buf);
            esp_ota_end(ota_handle);
            snprintf(g_ota_error_msg, sizeof(g_ota_error_msg), "Error receiving data");
            return httpd_resp_send_500(req);
        }

        err = esp_ota_write(ota_handle, buf, read_bytes);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_end(ota_handle);
            snprintf(g_ota_error_msg, sizeof(g_ota_error_msg), "OTA write failed: %s", esp_err_to_name(err));
            return httpd_resp_send_500(req);
        }

        g_ota_received_size += read_bytes;

        /* Progress update */
        ESP_LOGI(TAG, "OTA progress: %d/%d bytes", g_ota_received_size, fw_size);
    }

    free(buf);

    /* Finalize OTA update */
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        snprintf(g_ota_error_msg, sizeof(g_ota_error_msg), "OTA end failed: %s", esp_err_to_name(err));
        return httpd_resp_send_500(req);
    }

    /* Set new OTA partition as boot partition */
    err = esp_ota_set_boot_partition(partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        snprintf(g_ota_error_msg, sizeof(g_ota_error_msg), "Set boot partition failed: %s", esp_err_to_name(err));
        return httpd_resp_send_500(req);
    }

    g_ota_state = OTA_COMPLETED;

    /* Send success response */
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OTA update successful. Device will restart now.", HTTPD_RESP_USE_STRLEN);

    /* Restart after a short delay */
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();

    return ESP_OK;
}

/* Simple root page handler */
static esp_err_t root_handler(httpd_req_t *req)
{
    const char* html_content =
        "<html>"
        "<head>"
        "<title>ESP32 C-RID OTA Update</title>"
        "<meta charset='utf-8'>"
        "<style>"
        "body { font-family: Arial, sans-serif; margin: 40px; background-color: #f5f5f5; }"
        "h2 { color: #333; }"
        "form { margin: 20px 0; padding: 20px; background-color: white; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
        "input[type='file'] { margin: 10px 0; }"
        "input[type='submit'] { background-color: #4CAF50; color: white; padding: 10px 20px; border: none; cursor: pointer; border-radius: 3px; }"
        "input[type='submit']:hover { background-color: #45a049; }"
        ".status-box { margin: 20px 0; padding: 15px; background-color: white; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
        ".info-box { margin: 20px 0; padding: 15px; background-color: #e3f2fd; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
        ".progress-bar { width: 100%; background-color: #f0f0f0; border-radius: 5px; overflow: hidden; margin: 10px 0; }"
        ".progress-fill { height: 20px; background-color: #4CAF50; width: 0%; transition: width 0.3s; }"
        ".section { margin: 30px 0; }"
        ".section-title { color: #333; border-bottom: 2px solid #4CAF50; padding-bottom: 5px; }"
        "</style>"
        "</head>"
        "<body>"
        "<h2>ESP32 C-RID Firmware OTA Update</h2>"

        "<div class='info-box'>"
        "<h3 class='section-title'>Device Information</h3>"
        "<div id='deviceInfo'>Loading device information...</div>"
        "</div>"

        "<div class='info-box'>"
        "<h3 class='section-title'>System Information</h3>"
        "<div id='systemInfo'>Loading system information...</div>"
        "</div>"

        "<div class='section'>"
        "<h3 class='section-title'>Firmware Update</h3>"
        "<form id='otaForm' action=\"/ota\" method=\"post\" enctype=\"multipart/form-data\">"
        "<p>Firmware File (.bin): <input type=\"file\" name=\"firmware\" accept=\".bin\" required></p>"
        "<p><input type=\"submit\" value=\"Upload Firmware\"></p>"
        "</form>"
        "</div>"

        "<div class='status-box'>"
        "<h3 class='section-title'>Update Status</h3>"
        "<div id=\"progressText\">Waiting for update...</div>"
        "<div class=\"progress-bar\">"
        "<div id=\"progressBar\" class=\"progress-fill\"></div>"
        "</div>"
        "</div>"

        "<script>"
        "function updateSystemInfo() {"
        "  fetch('/api/system/info')"
        "    .then(response => response.json())"
        "    .then(data => {"
        "      document.getElementById('systemInfo').innerHTML = "
        "        '<p><b>Chip Model:</b> ' + data.chip_model + '</p>' +"
        "        '<p><b>Chip Revision:</b> ' + data.chip_revision + '</p>' +"
        "        '<p><b>Cores:</b> ' + data.chip_cores + '</p>' +"
        "        '<p><b>Flash Size:</b> ' + (data.flash_size / 1024 / 1024) + ' MB</p>' +"
        "        '<p><b>Free Heap:</b> ' + data.free_heap + ' bytes</p>' +"
        "        '<p><b>Min Free Heap:</b> ' + data.min_free_heap + ' bytes</p>' +"
        "        '<p><b>Version:</b> ' + data.version + '</p>' +"
        "        '<p><b>Build Date:</b> ' + data.build_date + '</p>' +"
        "        '<p><b>Build Time:</b> ' + data.build_time + '</p>';"
        "    })"
        "    .catch(error => {"
        "      console.error('Error fetching system info:', error);"
        "      document.getElementById('systemInfo').innerHTML = 'Error loading system information';"
        "    });"
        "}"

        "function updateDeviceInfo() {"
        "  fetch('/api/device/status')"
        "    .then(response => response.json())"
        "    .then(data => {"
        "      const uptimeHours = Math.floor(data.uptime_seconds / 3600);"
        "      const uptimeMinutes = Math.floor((data.uptime_seconds % 3600) / 60);"
        "      const uptimeSeconds = data.uptime_seconds % 60;"
        "      document.getElementById('deviceInfo').innerHTML = "
        "        '<p><b>Uptime:</b> ' + uptimeHours + 'h ' + uptimeMinutes + 'm ' + uptimeSeconds + 's</p>' +"
        "        '<p><b>WiFi SSID:</b> ' + data.wifi_ssid + '</p>' +"
        "        '<p><b>WiFi RSSI:</b> ' + data.wifi_rssi + ' dBm</p>' +"
        "        '<p><b>Heap Free:</b> ' + data.heap_free + ' bytes</p>' +"
        "        '<p><b>Min Heap Free:</b> ' + data.heap_min_free + ' bytes</p>';"
        "    })"
        "    .catch(error => {"
        "      console.error('Error fetching device info:', error);"
        "      document.getElementById('deviceInfo').innerHTML = 'Error loading device information';"
        "    });"
        "}"

        "function updateProgress() {"
        "  fetch('/api/ota/progress')"
        "    .then(response => response.json())"
        "    .then(data => {"
        "      document.getElementById('progressText').innerText = "
        "        'State: ' + data.state + ', Progress: ' + data.progress + '%';"
        "      document.getElementById('progressBar').style.width = data.progress + '%';"
        "      if (data.state !== 'completed' && data.state !== 'failed') {"
        "        setTimeout(updateProgress, 1000);"
        "      }"
        "    })"
        "    .catch(error => {"
        "      console.error('Error:', error);"
        "      setTimeout(updateProgress, 5000);"
        "    });"
        "}"

        "document.getElementById('otaForm').addEventListener('submit', function() {"
        "  setTimeout(updateProgress, 1000);"
        "});"

        "// Initial updates"
        "updateSystemInfo();"
        "updateDeviceInfo();"
        "updateProgress();"

        "// Periodic updates"
        "setInterval(updateSystemInfo, 10000);"
        "setInterval(updateDeviceInfo, 5000);"
        "</script>"

        "<hr>"
        "<p><small>Note: Make sure you upload a valid firmware file for this device.</small></p>"
        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_content, HTTPD_RESP_USE_STRLEN);
}

/* Initialize the web server for OTA updates */
esp_err_t crid_ota_web_init(void)
{
    /* Only initialize if we have sufficient heap memory */
    if (esp_get_free_heap_size() < 100000) {
        ESP_LOGW(TAG, "Insufficient heap memory for OTA server (%d bytes)", esp_get_free_heap_size());
        return ESP_FAIL;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 12;
    config.max_resp_headers = 8;
    config.backlog_conn = 3;
    config.lru_purge_enable = true;

    /* Start the httpd server */
    esp_err_t err = httpd_start(&g_httpd_handle, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
        return err;
    }

    /* Register URI handlers */
    httpd_uri_t root_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = root_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(g_httpd_handle, &root_uri);

    httpd_uri_t ota_uri = {
        .uri       = "/ota",
        .method    = HTTP_POST,
        .handler   = ota_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(g_httpd_handle, &ota_uri);

    // Register new API endpoints
    httpd_uri_t system_info_uri = {
        .uri       = "/api/system/info",
        .method    = HTTP_GET,
        .handler   = system_info_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(g_httpd_handle, &system_info_uri);

    httpd_uri_t device_status_uri = {
        .uri       = "/api/device/status",
        .method    = HTTP_GET,
        .handler   = device_status_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(g_httpd_handle, &device_status_uri);

    httpd_uri_t ota_progress_uri = {
        .uri       = "/api/ota/progress",
        .method    = HTTP_GET,
        .handler   = ota_progress_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(g_httpd_handle, &ota_progress_uri);

    // 新增的设备和系统状态API
    httpd_uri_t system_status_uri = {
        .uri       = "/api/system/status",
        .method    = HTTP_GET,
        .handler   = system_status_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(g_httpd_handle, &system_status_uri);

    httpd_uri_t uav_tracking_uri = {
        .uri       = "/api/uav/tracking",
        .method    = HTTP_GET,
        .handler   = uav_tracking_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(g_httpd_handle, &uav_tracking_uri);

    httpd_uri_t network_status_uri = {
        .uri       = "/api/network/status",
        .method    = HTTP_GET,
        .handler   = network_status_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(g_httpd_handle, &network_status_uri);

    httpd_uri_t scan_stats_uri = {
        .uri       = "/api/scan/stats",
        .method    = HTTP_GET,
        .handler   = scan_stats_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(g_httpd_handle, &scan_stats_uri);

    ESP_LOGI(TAG, "OTA web server started on port %d", config.server_port);
    return ESP_OK;
}

/* Deinitialize the web server */
void crid_ota_web_deinit(void)
{
    if (g_httpd_handle) {
        httpd_stop(g_httpd_handle);
        g_httpd_handle = NULL;
        ESP_LOGI(TAG, "OTA web server stopped");
    }
}

/* Check if OTA is currently in progress */
bool crid_ota_is_in_progress(void)
{
    return (g_ota_state == OTA_STARTED || g_ota_state == OTA_IN_PROGRESS);
}

/* Get OTA progress percentage */
int crid_ota_get_progress(void)
{
    if (g_ota_update_size <= 0) {
        return 0;
    }
    return (g_ota_received_size * 100) / g_ota_update_size;
}

/* Get current OTA state */
ota_state_t crid_ota_get_state(void)
{
    return g_ota_state;
}

/* Get last error message */
const char* crid_ota_get_last_error(void)
{
    if (strlen(g_ota_error_msg) > 0) {
        return g_ota_error_msg;
    }
    return "No error";
}

/* 设置预期的MD5校验值用于固件验证 */
void crid_ota_set_expected_md5(const uint8_t *md5)
{
    // 当前未实现，保留占位符
    (void)md5;
}

/* Get current system status */
const char* crid_ota_get_system_status(void)
{
    // 这个函数将在后续实现中添加
    return "{}";
}

/* Get current UAV tracking status */
const char* crid_ota_get_uav_tracking_status(void)
{
    // 这个函数将在后续实现中添加
    return "{}";
}

/* Get current network status */
const char* crid_ota_get_network_status(void)
{
    // 这个函数将在后续实现中添加
    return "{}";
}

/* Get current scan statistics */
const char* crid_ota_get_scan_stats(void)
{
    // 这个函数将在后续实现中添加
    return "{}";
}

/* System info handler */
static esp_err_t system_info_handler(httpd_req_t *req)
{
    char json_response[1024];

    // Get chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    // Get flash size
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);

    // Get free heap size
    uint32_t free_heap = esp_get_free_heap_size();

    // Get minimum free heap size
    uint32_t min_free_heap = esp_get_minimum_free_heap_size();

    // Format JSON response
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"chip_model\": \"%s\","
        "\"chip_revision\": %d,"
        "\"chip_cores\": %d,"
        "\"flash_size\": %lu,"
        "\"free_heap\": %lu,"
        "\"min_free_heap\": %lu,"
        "\"version\": \"%s\","
        "\"build_date\": \"%s\","
        "\"build_time\": \"%s\""
        "}",
        chip_info.model == CHIP_ESP32 ? "ESP32" :
        chip_info.model == CHIP_ESP32S2 ? "ESP32-S2" :
        chip_info.model == CHIP_ESP32S3 ? "ESP32-S3" :
        chip_info.model == CHIP_ESP32C3 ? "ESP32-C3" : "Unknown",
        chip_info.revision,
        chip_info.cores,
        flash_size,
        free_heap,
        min_free_heap,
        CRID_VERSION_STRING,
        CRID_BUILD_DATE,
        CRID_BUILD_TIME
    );

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

/* Device status handler */
static esp_err_t device_status_handler(httpd_req_t *req)
{
    char json_response[512];

    // Get WiFi information
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    char ssid[33] = "Not connected";
    int rssi = 0;

    if (err == ESP_OK) {
        strncpy(ssid, (char*)ap_info.ssid, sizeof(ssid) - 1);
        ssid[sizeof(ssid) - 1] = '\0';
        rssi = ap_info.rssi;
    }

    // Get uptime in seconds
    uint32_t uptime_seconds = esp_log_timestamp() / 1000;

    // Format JSON response
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"uptime_seconds\": %lu,"
        "\"wifi_ssid\": \"%s\","
        "\"wifi_rssi\": %d,"
        "\"heap_free\": %lu,"
        "\"heap_min_free\": %lu"
        "}",
        uptime_seconds,
        ssid,
        rssi,
        esp_get_free_heap_size(),
        esp_get_minimum_free_heap_size()
    );

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

/* OTA progress handler */
static esp_err_t ota_progress_handler(httpd_req_t *req)
{
    char json_response[256];
    int progress = 0;

    if (g_ota_update_size > 0) {
        progress = (g_ota_received_size * 100) / g_ota_update_size;
    }

    snprintf(json_response, sizeof(json_response),
        "{"
        "\"state\": \"%s\","
        "\"progress\": %d,"
        "\"received\": %d,"
        "\"total\": %d,"
        "\"error\": \"%s\""
        "}",
        (g_ota_state == OTA_IDLE) ? "idle" :
        (g_ota_state == OTA_STARTED) ? "started" :
        (g_ota_state == OTA_IN_PROGRESS) ? "in_progress" :
        (g_ota_state == OTA_COMPLETED) ? "completed" : "failed",
        progress,
        g_ota_received_size,
        g_ota_update_size,
        g_ota_error_msg
    );

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

/* 新增的API处理函数 */
/* 系统状态处理器 */
static esp_err_t system_status_handler(httpd_req_t *req)
{
    char json_response[1024];

    // 获取芯片信息
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    // 获取内存信息
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_free_heap = esp_get_minimum_free_heap_size();
    uint32_t total_heap = esp_get_free_heap_size() + esp_get_minimum_free_heap_size();

    // 获取系统运行时间
    uint32_t uptime_seconds = esp_log_timestamp() / 1000;

    // 获取OTA状态
    ota_state_t ota_state = crid_ota_get_state();
    int ota_progress = crid_ota_get_progress();

    // 构造JSON响应
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"system\":{"
            "\"chip_model\": \"%s\","
            "\"chip_revision\": %d,"
            "\"chip_cores\": %d,"
            "\"free_heap\": %lu,"
            "\"min_free_heap\": %lu,"
            "\"total_heap\": %lu,"
            "\"uptime_seconds\": %lu,"
            "\"version\": \"%s\","
            "\"build_date\": \"%s\","
            "\"build_time\": \"%s\""
        "},"
        "\"ota\":{"
            "\"state\": \"%s\","
            "\"progress\": %d"
        "}"
        "}",
        chip_info.model == CHIP_ESP32 ? "ESP32" :
        chip_info.model == CHIP_ESP32S2 ? "ESP32-S2" :
        chip_info.model == CHIP_ESP32S3 ? "ESP32-S3" :
        chip_info.model == CHIP_ESP32C3 ? "ESP32-C3" : "Unknown",
        chip_info.revision,
        chip_info.cores,
        free_heap,
        min_free_heap,
        total_heap,
        uptime_seconds,
        CRID_VERSION_STRING,
        CRID_BUILD_DATE,
        CRID_BUILD_TIME,
        (ota_state == OTA_IDLE) ? "idle" :
        (ota_state == OTA_STARTED) ? "started" :
        (ota_state == OTA_IN_PROGRESS) ? "in_progress" :
        (ota_state == OTA_COMPLETED) ? "completed" : "failed",
        ota_progress
    );

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

/* 无人机追踪状态处理器 */
static esp_err_t uav_tracking_handler(httpd_req_t *req)
{
    char json_response[2048];
    int offset = 0;

    // 获取追踪器数据
    SemaphoreHandle_t mutex = crid_tracker_get_mutex();
    uav_track_t *table = crid_tracker_get_table();
    int active_count = crid_tracker_get_active_count();

    offset += snprintf(json_response + offset, sizeof(json_response) - offset,
        "{"
        "\"tracking\":{"
            "\"max_uavs\": %d,"
            "\"active_uavs\": %d,"
            "\"uavs\": [",
        MAX_TRACKED_UAVS, active_count);

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool first = true;
        for (int i = 0; i < MAX_TRACKED_UAVS; i++) {
            if (!table[i].active) continue;

            if (!first) {
                offset += snprintf(json_response + offset, sizeof(json_response) - offset, ",");
            }
            first = false;

            // 将MAC地址转换为字符串
            char mac_str[18];
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                     table[i].mac[0], table[i].mac[1], table[i].mac[2],
                     table[i].mac[3], table[i].mac[4], table[i].mac[5]);

            // 构造每个无人机的状态
            offset += snprintf(json_response + offset, sizeof(json_response) - offset,
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

            // 添加基本ID信息（如果有效）
            if (table[i].basic_id.valid) {
                offset += snprintf(json_response + offset, sizeof(json_response) - offset,
                    ",\"basic_id\":{"
                    "\"id_type\":\"%s\","
                    "\"ua_type\":\"%s\","
                    "\"uas_id\":\"%s\""
                    "}",
                    crid_display_ua_type_name(table[i].basic_id.id_type),
                    crid_display_ua_type_name(table[i].basic_id.ua_type),
                    table[i].basic_id.uas_id);
            }

            // 添加位置信息（如果有效）
            if (table[i].location.valid) {
                offset += snprintf(json_response + offset, sizeof(json_response) - offset,
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
                    get_status_name(table[i].location.status));
            }

            offset += snprintf(json_response + offset, sizeof(json_response) - offset,
                "}");
        }

        xSemaphoreGive(mutex);
    }

    offset += snprintf(json_response + offset, sizeof(json_response) - offset,
        "]"
        "}"
        "}");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

/* 网络状态处理器 */
static esp_err_t network_status_handler(httpd_req_t *req)
{
    char json_response[1024];

    // 获取WiFi信息
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    char ssid[33] = "Not connected";
    int rssi = 0;
    bool connected = false;

    if (err == ESP_OK) {
        strncpy(ssid, (char*)ap_info.ssid, sizeof(ssid) - 1);
        ssid[sizeof(ssid) - 1] = '\0';
        rssi = ap_info.rssi;
        connected = true;
    }

    // 获取IP地址信息
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);

    // 获取网关和子网掩码
    char gateway_str[16];
    char netmask_str[16];
    snprintf(gateway_str, sizeof(gateway_str), IPSTR, IP2STR(&ip_info.gw));
    snprintf(netmask_str, sizeof(netmask_str), IPSTR, IP2STR(&ip_info.netmask));

    // 获取SNTP时间
    time_t now;
    time(&now);

    // 构造JSON响应
    snprintf(json_response, sizeof(json_response),
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
        ssid,
        rssi,
        connected ? "true" : "false",
        IP2STR(&ip_info.ip),
        gateway_str,
        netmask_str,
        now
    );

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

/* 扫描统计处理器 */
static esp_err_t scan_stats_handler(httpd_req_t *req)
{
    char json_response[1024];

    // 获取扫描统计信息
    sniffer_stats_t *stats = crid_sniffer_get_stats();

    // 获取追踪器信息
    int active_count = crid_tracker_get_active_count();

    // 构造JSON响应
    snprintf(json_response, sizeof(json_response),
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
        active_count
    );

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}