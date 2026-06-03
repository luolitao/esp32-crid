/**
 * crid_scan_main.c — Remote ID Scanner 主入口
 *
 * ESP32 Remote ID Scanner
 * Standards: ASTM F3411-22 / ASD-STAN prEN 4709-002
 *
 * 架构：
 *   - crid_sniffer:   Wi-Fi 混杂模式抓包，ISR 安全回调
 *   - crid_parser:    opendroneid 库解码
 *   - crid_tracker:   无人机追踪表（线程安全）
 *   - crid_display:   信息展示（摘要/详情/状态行）
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "opendroneid.h"

#include "crid_rx_types.h"
#include "crid_sniffer.h"
#include "crid_parser.h"
#include "crid_tracker.h"
#include "crid_display.h"

static const char *TAG = "RID_MAIN";

/* ================================================================
 * 解析任务 (从队列取数据，使用 opendroneid 库解析)
 * ================================================================ */

static void parser_task(void *pvParameter) {
    ESP_LOGI(TAG, "Parser task started");

    QueueHandle_t queue = crid_sniffer_get_queue();
    SemaphoreHandle_t mutex = crid_tracker_get_mutex();
    sniffer_msg_t msg;
    uint32_t last_cleanup_ms = 0;

    while (1) {
        if (xQueueReceive(queue, &msg, pdMS_TO_TICKS(1000)) != pdTRUE) {
            continue;
        }

        // 普通 Beacon（无 Vendor IE）：直接跳过
        if (msg.msg_type == MSG_TYPE_BEACON_NO_VENDOR) {
            continue;
        }

        // 非 RID Vendor IE：直接跳过
        if (msg.msg_type == MSG_TYPE_NON_RID_VENDOR) {
            continue;
        }

        // 以下仅处理 MSG_TYPE_RID

        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        uav_track_t *uav = crid_tracker_find_or_create(msg.src_mac);
        if (uav == NULL) {
            // 追踪表已满，尝试清理超时条目腾出空间
            uint32_t now = esp_log_timestamp();
            if (now - last_cleanup_ms >= 10000) {  // 每 10 秒最多清理一次
                crid_tracker_cleanup(UAV_TIMEOUT_MS);
                last_cleanup_ms = now;
                // 重试一次
                uav = crid_tracker_find_or_create(msg.src_mac);
            }
            if (uav == NULL) {
                ESP_LOGW(TAG, "Tracker full! Cannot track new UAV");
                xSemaphoreGive(mutex);
                continue;
            }
        }

        bool was_new = (uav->msg_count == 0);
        uav->last_rssi = msg.rssi;
        uav->last_channel = msg.channel;

        // 记录 OUI 和传输/协议类型
        uav->oui[0] = msg.oui[0];
        uav->oui[1] = msg.oui[1];
        uav->oui[2] = msg.oui[2];
        uav->oui_type = msg.oui_type;
        uav->transport = (uint8_t)GET_RID_TRANSPORT(msg.oui[0], msg.oui[1], msg.oui[2]);
        uav->protocol = RID_PROTOCOL_ASTM_F3411;

        // 解码
        crid_parser_decode(uav, msg.data, msg.data_len);

        // 提取分层数据（供显示层使用）
        crid_parser_extract_layered(uav);

        xSemaphoreGive(mutex);

        // 仅新发现 UAV 时打印 1 行精简信息，重复更新不显示
        if (was_new && uav->basic_id.valid) {
            crid_display_uav_summary(uav);
        }
    }
}

/* ================================================================
 * 监控任务 (定期输出状态)
 * ================================================================ */

static void monitor_task(void *pvParameter) {
    ESP_LOGI(TAG, "Monitor task started");

    uint32_t loop_count = 0;
    uint32_t last_packets = 0;
    uint32_t last_mgmt = 0;
    uint32_t last_rid = 0;
    uint32_t last_beacons = 0;
    uint32_t last_non_rid = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        loop_count++;

        sniffer_stats_t *stats = crid_sniffer_get_stats();
        SemaphoreHandle_t mutex = crid_tracker_get_mutex();

        uint32_t total_pkts = stats->total_packets;
        uint32_t mgmt_pkts = stats->mgmt_frames;
        uint32_t rid_pkts = stats->rid_detections;
        uint32_t overflows = stats->queue_overflows;
        uint32_t beacons = stats->beacon_count;
        uint32_t non_rid = stats->non_rid_vendor_ie;

        ESP_LOGI(TAG, "=== Status (%lu min) ===", (unsigned long)loop_count);
        ESP_LOGI(TAG, "  Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
        ESP_LOGI(TAG, "  Total packets: %lu (%.1f pkt/s)",
                 (unsigned long)total_pkts,
                 (total_pkts - last_packets) / 60.0f);
        ESP_LOGI(TAG, "  Mgmt frames:   %lu (%.1f frm/s)",
                 (unsigned long)mgmt_pkts,
                 (mgmt_pkts - last_mgmt) / 60.0f);
        ESP_LOGI(TAG, "  Beacons:        %lu (%.1f /s)",
                 (unsigned long)beacons,
                 (beacons - last_beacons) / 60.0f);
        ESP_LOGI(TAG, "  RID detected:   %lu (%.1f /s)",
                 (unsigned long)rid_pkts,
                 (rid_pkts - last_rid) / 60.0f);
        ESP_LOGI(TAG, "  Non-RID Vendor: %lu (%.1f /s)",
                 (unsigned long)non_rid,
                 (non_rid - last_non_rid) / 60.0f);
        ESP_LOGI(TAG, "  Queue overflows: %lu", (unsigned long)overflows);

        last_packets = total_pkts;
        last_mgmt = mgmt_pkts;
        last_rid = rid_pkts;
        last_beacons = beacons;
        last_non_rid = non_rid;

        // 打印活跃无人机列表并清理超时
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int active = crid_tracker_get_active_count();
            ESP_LOGI(TAG, "  Active UAVs: %d", active);

            uav_track_t *table = crid_tracker_get_table();
            for (int i = 0; i < MAX_TRACKED_UAVS; i++) {
                if (!table[i].active) continue;
                crid_display_uav_status(&table[i]);
            }

            // 清理超时条目
            crid_tracker_cleanup(UAV_TIMEOUT_MS);

            xSemaphoreGive(mutex);
        }

        ESP_LOGI(TAG, "========================");
    }
}

/* ================================================================
 * 主函数
 * ================================================================ */

void app_main(void) {
    printf("\n\n");
    printf("================================================\n");
    printf("  ESP32 Remote ID Scanner\n");
    printf("  Version: %s (built %s %s)\n", CRID_VERSION_STRING, CRID_BUILD_DATE, CRID_BUILD_TIME);
    printf("  Standards: ASTM F3411-22 / ASD-STAN prEN 4709-002\n");
    printf("================================================\n");
    fflush(stdout);

    ESP_LOGI(TAG, "Starting Remote ID Scanner v%s", CRID_VERSION_STRING);
    ESP_LOGI(TAG, "Build: %s %s", CRID_BUILD_DATE, CRID_BUILD_TIME);
    ESP_LOGI(TAG, "ESP-IDF: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Protocol library: opendroneid v%d", ODID_PROTOCOL_VERSION);

    // 1. 初始化追踪器
    crid_tracker_init();

    // 2. 初始化 NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return;
    }

    // 3. 初始化网络接口和事件循环
    ESP_LOGI(TAG, "Initializing netif...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGI(TAG, "Creating event loop...");
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 4. 初始化 Wi-Fi sniffer
    ret = crid_sniffer_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sniffer init failed!");
        return;
    }

    ESP_LOGI(TAG, "Wi-Fi monitor mode enabled");
    ESP_LOGI(TAG, "Promiscuous mode: ON");
    ESP_LOGI(TAG, "Locked to channel %d", FIXED_CHANNEL);

    // 5. 创建任务
    BaseType_t task_created;

    task_created = xTaskCreate(parser_task, "parser",
                               PARSER_TASK_STACK, NULL, PARSER_TASK_PRIO, NULL);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create parser task!");
        return;
    }
    ESP_LOGI(TAG, "Parser task created (stack: %d)", PARSER_TASK_STACK);

    task_created = xTaskCreate(monitor_task, "monitor",
                               MONITOR_TASK_STACK, NULL, MONITOR_TASK_PRIO, NULL);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task!");
        return;
    }
    ESP_LOGI(TAG, "Monitor task created (stack: %d)", MONITOR_TASK_STACK);

    crid_sniffer_start_channel_hold();
    ESP_LOGI(TAG, "Channel hold task created");

    // 6. 启动完成
    printf("\n");
    printf("================================================\n");
    printf("  Remote ID Scanner Ready!\n");
    printf("  Version: %s (built %s %s)\n", CRID_VERSION_STRING, CRID_BUILD_DATE, CRID_BUILD_TIME);
    printf("  Locked to channel %d\n", FIXED_CHANNEL);
    printf("  Tracking up to %d UAVs\n", MAX_TRACKED_UAVS);
    printf("  Protocols: ASTM F3411, ASD-STAN prEN 4709-002\n");
    printf("  Free heap: %lu bytes\n",
           (unsigned long)esp_get_free_heap_size());
    printf("================================================\n\n");
    fflush(stdout);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Remote ID Scanner v%s is running!", CRID_VERSION_STRING);
    ESP_LOGI(TAG, "Tracking up to %d UAVs", MAX_TRACKED_UAVS);
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");
}
