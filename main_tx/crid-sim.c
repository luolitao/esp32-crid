/**
 * @file crid-sim.c
 * @brief ESP32 中国民用无人机远程识别 (C-RID) 模拟发射器 - 主入口
 *
 * 符合 GB42590-2023 和《民用微轻小型无人驾驶航空器运行识别最低性能要求（试行）》
 */

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "crid_config.h"
#include "crid_messages.h"
#include "crid_wifi.h"
#include "crid_patrol.h"

static const char *TAG = "ESP32_CRID_STD";

// --- 全局 Beacon 帧缓冲区（仅在发送任务中使用） ---
#define BEACON_FRAME_BUF_SIZE 512
static uint8_t g_beacon_frame[BEACON_FRAME_BUF_SIZE];
static uint16_t g_beacon_frame_len = 0;

// --- 全局配置 ---
static cn_crid_config_t g_config;

// --- 位置更新间隔 ---
#define PATROL_INTERVAL_SEC 10

/**
 * @brief C-RID Beacon 发送任务
 *
 * 每秒发送一次 Beacon 帧，每 PATROL_INTERVAL_SEC 秒更新一次巡游位置。
 */
static void crid_send_beacon_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting China C-RID beacon transmission...");

    TickType_t xLastWake1s = xTaskGetTickCount();
    TickType_t xLastWake10s = xTaskGetTickCount();

    const TickType_t xInterval1s  = pdMS_TO_TICKS(DEFAULT_BEACON_INTERVAL_MS);
    const TickType_t xInterval10s = pdMS_TO_TICKS(PATROL_INTERVAL_SEC * 1000);

    for (;;) {
        // --- 1 秒定时：发送 Beacon ---
        vTaskDelayUntil(&xLastWake1s, xInterval1s);

        esp_err_t ret = crid_wifi_send_raw_frame(g_beacon_frame, g_beacon_frame_len);
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "Beacon sent (counter=%u)", g_config.message_counter);
        }

        // --- 10 秒定时：更新巡游位置 ---
        TickType_t xNow = xTaskGetTickCount();
        if ((xNow - xLastWake10s) >= xInterval10s) {
            ESP_LOGI(TAG, "=== Patrol position update ===");

            crid_patrol_step(&g_config);

            // 重建帧
            if (!crid_build_beacon_frame(&g_config,
                                         g_beacon_frame, BEACON_FRAME_BUF_SIZE,
                                         &g_beacon_frame_len)) {
                ESP_LOGE(TAG, "Failed to rebuild beacon frame!");
            }

            xLastWake10s = xNow;
        }
    }
}

/**
 * @brief 应用主入口
 */
void app_main(void) {
    ESP_LOGI(TAG, "=== ESP32 China C-RID Transmitter ===");
    ESP_LOGI(TAG, "Standard: GB42590-2023");

    // 1. 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 初始化 TCP/IP 网络接口
    ESP_ERROR_CHECK(esp_netif_init());

    // 3. 创建默认事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 4. 初始化 C-RID 配置
    crid_config_init_default(&g_config);

    // 5. 初始化 Wi-Fi
    ret = crid_wifi_init(g_config.channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi init failed: %s", esp_err_to_name(ret));
        return;
    }

    // 6. 构建初始 Beacon 帧
    if (!crid_build_beacon_frame(&g_config,
                                  g_beacon_frame, BEACON_FRAME_BUF_SIZE,
                                  &g_beacon_frame_len)) {
        ESP_LOGE(TAG, "Failed to build initial beacon frame!");
        return;
    }

    // 7. 启动发送任务
    BaseType_t task_ret = xTaskCreate(crid_send_beacon_task, "cn_crid_tx",
                                      4096, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TX task!");
        return;
    }

    ESP_LOGI(TAG, "Transmitter started successfully!");
    ESP_LOGI(TAG, "  UAS ID (Model): %s", g_config.uas_id);
    ESP_LOGI(TAG, "  Drone Name: %s", g_config.drone_name);
    ESP_LOGI(TAG, "  Operator ID: %s", g_config.operator_id);
    ESP_LOGI(TAG, "  MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             g_config.mac_address[0], g_config.mac_address[1], g_config.mac_address[2],
             g_config.mac_address[3], g_config.mac_address[4], g_config.mac_address[5]);
    ESP_LOGI(TAG, "  Position: %.6f, %.6f", g_config.latitude, g_config.longitude);
    ESP_LOGI(TAG, "  Channel: %u, Interval: %ums", g_config.channel, DEFAULT_BEACON_INTERVAL_MS);
    ESP_LOGI(TAG, "  OUI: FA:0B:BC, Vendor Type: 0x0D (GB42590-2023)");
}
