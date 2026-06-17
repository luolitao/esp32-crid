/**
 * crid_ota.c — OTA 固件更新核心逻辑
 *
 * 处理固件上传、写入 Flash 和重启。不依赖 HTTP 路由注册。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_partition.h"

#include "crid_ota_web.h"
#include "crid_ota_internal.h"

static const char *TAG = "C-RID OTA";

/* OTA runtime state */
static ota_state_t g_ota_state      = OTA_IDLE;
static int         g_ota_total_size = 0;
static int         g_ota_recv_size  = 0;
static char        g_ota_error_msg[256] = {0};

/* --------------- public getters --------------- */

bool crid_ota_is_active(void)
{
    return g_ota_state == OTA_STARTED || g_ota_state == OTA_IN_PROGRESS;
}

int crid_ota_get_total_size(void)   { return g_ota_total_size; }
int crid_ota_get_received_size(void){ return g_ota_recv_size; }

const char *crid_ota_get_error_string(void)
{
    return g_ota_error_msg[0] ? g_ota_error_msg : "No error";
}

const char *crid_ota_state_name(void)
{
    switch (g_ota_state) {
        case OTA_IDLE:       return "idle";
        case OTA_STARTED:    return "started";
        case OTA_IN_PROGRESS:return "in_progress";
        case OTA_COMPLETED:  return "completed";
        case OTA_FAILED:     return "failed";
    }
    return "unknown";
}

int crid_ota_get_progress(void)
{
    if (g_ota_total_size <= 0) return 0;
    return (g_ota_recv_size * 100) / g_ota_total_size;
}

ota_state_t crid_ota_get_state(void)
{
    return g_ota_state;
}

bool crid_ota_is_in_progress(void)
{
    return crid_ota_is_active();
}

const char *crid_ota_get_last_error(void)
{
    return crid_ota_get_error_string();
}

void crid_ota_set_expected_md5(const uint8_t *md5)
{
    (void)md5;  /* reserved for future MD5 verification */
}

/* --------------- OTA upload handler --------------- */

esp_err_t crid_ota_upload_handler(httpd_req_t *req)
{
    char *buf = NULL;
    ssize_t read_bytes = 0;
    esp_err_t err = ESP_OK;

    char content_type[256];
    err = httpd_req_get_hdr_value_str(req, "Content-Type",
                                      content_type, sizeof(content_type));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get Content-Type header");
        return httpd_resp_send_500(req);
    }

    if (strstr(content_type, "multipart/form-data") == NULL) {
        ESP_LOGE(TAG, "Not multipart/form-data");
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Bad Request", HTTPD_RESP_USE_STRLEN);
    }

    char content_length[32];
    err = httpd_req_get_hdr_value_str(req, "Content-Length",
                                      content_length, sizeof(content_length));
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
        snprintf(g_ota_error_msg, sizeof(g_ota_error_msg),
                 "OTA begin failed: %s", esp_err_to_name(err));
        return httpd_resp_send_500(req);
    }

    g_ota_state     = OTA_STARTED;
    g_ota_total_size = fw_size;
    g_ota_recv_size  = 0;

    buf = malloc(1024);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        esp_ota_end(ota_handle);
        snprintf(g_ota_error_msg, sizeof(g_ota_error_msg),
                 "Memory allocation failed");
        return httpd_resp_send_500(req);
    }

    while (g_ota_recv_size < fw_size) {
        read_bytes = httpd_req_recv(req, buf,
                                    MIN(1024, fw_size - g_ota_recv_size));
        if (read_bytes <= 0) {
            ESP_LOGE(TAG, "Error receiving data");
            free(buf);
            esp_ota_end(ota_handle);
            snprintf(g_ota_error_msg, sizeof(g_ota_error_msg),
                     "Error receiving data");
            return httpd_resp_send_500(req);
        }

        err = esp_ota_write(ota_handle, buf, read_bytes);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_end(ota_handle);
            snprintf(g_ota_error_msg, sizeof(g_ota_error_msg),
                     "OTA write failed: %s", esp_err_to_name(err));
            return httpd_resp_send_500(req);
        }

        g_ota_recv_size += read_bytes;
        ESP_LOGI(TAG, "OTA progress: %d/%d bytes", g_ota_recv_size, fw_size);
    }

    free(buf);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        snprintf(g_ota_error_msg, sizeof(g_ota_error_msg),
                 "OTA end failed: %s", esp_err_to_name(err));
        return httpd_resp_send_500(req);
    }

    err = esp_ota_set_boot_partition(partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s",
                 esp_err_to_name(err));
        snprintf(g_ota_error_msg, sizeof(g_ota_error_msg),
                 "Set boot partition failed: %s", esp_err_to_name(err));
        return httpd_resp_send_500(req);
    }

    g_ota_state = OTA_COMPLETED;

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req,
                    "OTA update successful. Device will restart now.",
                    HTTPD_RESP_USE_STRLEN);

    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();

    return ESP_OK;
}
