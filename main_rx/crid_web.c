/**
 * crid_web.c — HTTP 服务器初始化与路由注册
 *
 * 负责启动 HTTP 服务器、注册所有 URI 处理器、提供根页面。
 */

#include <esp_http_server.h>
#include <esp_log.h>
#include "esp_system.h"

#include "crid_ota_web.h"
#include "crid_ota_internal.h"
#include "crid_web_api.h"

static const char *TAG = "C-RID Web";

/* HTTP Server handle */
static httpd_handle_t g_httpd_handle = NULL;

/* ---------- 根页面 ---------- */

static const char ROOT_HTML[] =
    "<html>"
    "<head>"
    "<title>ESP32 C-RID OTA Update</title>"
    "<meta charset='utf-8'>"
    "<style>"
    "body { font-family: Arial, sans-serif; margin: 40px; background-color: #f5f5f5; }"
    "h2 { color: #333; }"
    "form { margin: 20px 0; padding: 20px; background-color: white; "
    "border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
    "input[type='file'] { margin: 10px 0; }"
    "input[type='submit'] { background-color: #4CAF50; color: white; "
    "padding: 10px 20px; border: none; cursor: pointer; border-radius: 3px; }"
    "input[type='submit']:hover { background-color: #45a049; }"
    ".status-box, .info-box { margin: 20px 0; padding: 15px; "
    "border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
    ".info-box { background-color: #e3f2fd; }"
    ".status-box { background-color: white; }"
    ".progress-bar { width: 100%; background-color: #f0f0f0; "
    "border-radius: 5px; overflow: hidden; margin: 10px 0; }"
    ".progress-fill { height: 20px; background-color: #4CAF50; "
    "width: 0%; transition: width 0.3s; }"
    ".section { margin: 30px 0; }"
    ".section-title { color: #333; border-bottom: 2px solid #4CAF50; "
    "padding-bottom: 5px; }"
    "</style>"
    "</head>"
    "<body>"
    "<h2>ESP32 C-RID Firmware OTA Update</h2>"
    "<div class='info-box'>"
    "<h3 class='section-title'>Device Information</h3>"
    "<div id='deviceInfo'>Loading...</div>"
    "</div>"
    "<div class='info-box'>"
    "<h3 class='section-title'>System Information</h3>"
    "<div id='systemInfo'>Loading...</div>"
    "</div>"
    "<div class='section'>"
    "<h3 class='section-title'>Firmware Update</h3>"
    "<form id='otaForm' action=\"/ota\" method=\"post\" "
    "enctype=\"multipart/form-data\">"
    "<p>Firmware File (.bin): <input type=\"file\" name=\"firmware\" "
    "accept=\".bin\" required></p>"
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
    "function fetchJSON(url,cb){fetch(url).then(r=>r.json()).then(cb)"
    ".catch(e=>console.error('fetch error:',e));}"
    "function updateSystemInfo(){fetchJSON('/api/system/info',d=>{"
    "document.getElementById('systemInfo').innerHTML="
    "'<p><b>Chip:</b> '+d.chip_model+' (rev '+d.chip_revision+')</p>'+"
    "'<p><b>Cores:</b> '+d.chip_cores+'</p>'+"
    "'<p><b>Flash:</b> '+(d.flash_size/1024/1024)+' MB</p>'+"
    "'<p><b>Heap:</b> '+d.free_heap+' / '+d.min_free_heap+' bytes</p>'+"
    "'<p><b>Version:</b> '+d.version+' ('+d.build_date+' '+d.build_time+')</p>';"
    "});}"
    "function updateDeviceInfo(){fetchJSON('/api/device/status',d=>{"
    "var h=Math.floor(d.uptime_seconds/3600);"
    "var m=Math.floor((d.uptime_seconds%3600)/60);"
    "var s=d.uptime_seconds%60;"
    "document.getElementById('deviceInfo').innerHTML="
    "'<p><b>Uptime:</b> '+h+'h '+m+'m '+s+'s</p>'+"
    "'<p><b>WiFi:</b> '+d.wifi_ssid+' ('+d.wifi_rssi+' dBm)</p>'+"
    "'<p><b>Heap:</b> '+d.heap_free+' / '+d.heap_min_free+' bytes</p>';"
    "});}"
    "function updateProgress(){fetchJSON('/api/ota/progress',d=>{"
    "document.getElementById('progressText').innerText="
    "'State: '+d.state+', Progress: '+d.progress+'%';"
    "document.getElementById('progressBar').style.width=d.progress+'%';"
    "if(d.state!=='completed'&&d.state!=='failed')"
    "  setTimeout(updateProgress,1000);"
    "});}"
    "document.getElementById('otaForm').addEventListener('submit',()=>{"
    "setTimeout(updateProgress,1000);});"
    "updateSystemInfo();updateDeviceInfo();updateProgress();"
    "setInterval(updateSystemInfo,10000);"
    "setInterval(updateDeviceInfo,5000);"
    "</script>"
    "<hr><p><small>Upload a valid firmware file for this device.</small></p>"
    "</body>"
    "</html>";

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, ROOT_HTML, HTTPD_RESP_USE_STRLEN);
}

/* ---------- 注册单个路由的辅助函数 ---------- */

static void register_get(httpd_handle_t h, const char *uri,
                          esp_err_t (*handler)(httpd_req_t *))
{
    httpd_uri_t u = {
        .uri     = uri,
        .method  = HTTP_GET,
        .handler = handler,
        .user_ctx= NULL,
    };
    httpd_register_uri_handler(h, &u);
}

/* ---------- public API ---------- */

esp_err_t crid_ota_web_init(void)
{
    if (esp_get_free_heap_size() < 100000) {
        ESP_LOGW(TAG, "Insufficient heap for web server (%d bytes)",
                 esp_get_free_heap_size());
        return ESP_FAIL;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port     = 80;
    config.ctrl_port       = 32768;
    config.max_uri_handlers= 12;
    config.max_resp_headers= 8;
    config.backlog_conn    = 3;
    config.lru_purge_enable= true;

    esp_err_t err = httpd_start(&g_httpd_handle, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s",
                 esp_err_to_name(err));
        return err;
    }

    /* 静态页面 */
    register_get(g_httpd_handle, "/", root_handler);

    /* OTA 上传 (POST) */
    httpd_uri_t ota_uri = {
        .uri     = "/ota",
        .method  = HTTP_POST,
        .handler = crid_ota_upload_handler,
        .user_ctx= NULL,
    };
    httpd_register_uri_handler(g_httpd_handle, &ota_uri);

    /* JSON API */
    register_get(g_httpd_handle, "/api/system/info",
                 crid_web_api_system_info);
    register_get(g_httpd_handle, "/api/device/status",
                 crid_web_api_device_status);
    register_get(g_httpd_handle, "/api/ota/progress",
                 crid_web_api_ota_progress);
    register_get(g_httpd_handle, "/api/system/status",
                 crid_web_api_system_status);
    register_get(g_httpd_handle, "/api/uav/tracking",
                 crid_web_api_uav_tracking);
    register_get(g_httpd_handle, "/api/network/status",
                 crid_web_api_network_status);
    register_get(g_httpd_handle, "/api/scan/stats",
                 crid_web_api_scan_stats);

    ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
    return ESP_OK;
}

void crid_ota_web_deinit(void)
{
    if (g_httpd_handle) {
        httpd_stop(g_httpd_handle);
        g_httpd_handle = NULL;
        ESP_LOGI(TAG, "Web server stopped");
    }
}
