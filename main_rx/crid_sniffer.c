/**
 * crid_sniffer.c — Wi-Fi Sniffer 模块
 *
 * 负责：
 *   - Wi-Fi 初始化（NULL 模式 + Promiscuous）
 *   - sniffer 回调（ISR 安全，仅拷贝数据到队列）
 *   - 信道锁定任务
 *   - 全局统计
 */

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi_types.h"
#include "crid_sniffer.h"
#include "crid_json.h"

/* ---- 模块内部状态 ---- */

static QueueHandle_t g_sniffer_queue = NULL;
static sniffer_stats_t g_stats;

/* ---- ISR 回调 ---- */

static void wifi_sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    g_stats.total_packets++;

    // 仅处理管理帧
    if (type != WIFI_PKT_MGMT) return;
    g_stats.mgmt_frames++;

    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;

    // 帧长度检查
    if (pkt->rx_ctrl.sig_len < 24) return;

    wifi_mac_hdr_t *hdr = (wifi_mac_hdr_t *)pkt->payload;

    // 仅处理 Beacon (subtype 8) 和 Probe Response (subtype 5)
    uint8_t type_field = (hdr->frame_ctrl & 0x0C) >> 2;
    uint8_t subtype = (hdr->frame_ctrl & 0xF0) >> 4;
    if (type_field != 0 || (subtype != 8 && subtype != 5)) return;

    if (subtype == 8) g_stats.beacon_count++;

    // 解析信息元素 (跳过 24 字节 MAC 头)
    uint16_t frame_len = pkt->rx_ctrl.sig_len;
    if (frame_len <= 24) return;

    uint8_t *ie_start = pkt->payload + 24;
    uint16_t ie_len = frame_len - 24;
    uint16_t offset = 0;
    bool has_vendor_ie = false;
    uint8_t ssid[32] = {0};
    uint8_t ssid_len = 0;

    while (offset + 2 <= ie_len) {
        uint8_t ie_id = ie_start[offset];
        uint8_t ie_length = ie_start[offset + 1];

        if (offset + 2 + ie_length > ie_len) break;

        // SSID (ID=0)
        if (ie_id == 0 && ie_length <= 32) {
            ssid_len = ie_length;
            if (ssid_len > 0) {
                memcpy(ssid, &ie_start[offset + 2], ssid_len);
            }
        }

        // Vendor Specific IE (ID=221)
        if (ie_id == 221 && ie_length >= 4) {
            has_vendor_ie = true;
            uint8_t oui0 = ie_start[offset + 2];
            uint8_t oui1 = ie_start[offset + 3];
            uint8_t oui2 = ie_start[offset + 4];
            uint8_t oui_type = ie_start[offset + 5];

            bool is_rid = IS_RID_OUI(oui0, oui1, oui2);

            if (is_rid) {
                g_stats.rid_detections++;

                sniffer_msg_t msg;
                memset(&msg, 0, sizeof(msg));
                memcpy(msg.src_mac, hdr->addr2, 6);
                msg.rssi = pkt->rx_ctrl.rssi;
                msg.channel = pkt->rx_ctrl.channel;
                msg.timestamp_ms = esp_log_timestamp();
                msg.oui_type = oui_type;
                msg.is_rid = true;
                msg.msg_type = MSG_TYPE_RID;
                msg.oui[0] = oui0;
                msg.oui[1] = oui1;
                msg.oui[2] = oui2;
                msg.has_vendor_ie = true;
                memcpy(msg.ssid, ssid, ssid_len);
                msg.ssid_len = ssid_len;

                // 提取 Vendor Specific 数据
                // 格式: OUI(3) + OUI_Type(1) + ODID_service_info
                //   ODID_service_info: [message_counter(1)] [ODID_MessagePack_encoded(...)]
                // 跳过 ID + Len + OUI(3) + OUI_Type(1) = 6 字节
                // parser 使用 odid_message_process_pack() 解析
                uint16_t data_offset = offset + 6;
                uint16_t vendor_data_len = ie_length - 4;  // OUI(3) + Type(1)
                msg.data_len = (vendor_data_len < sizeof(msg.data)) ? vendor_data_len : sizeof(msg.data);
                memcpy(msg.data, &ie_start[data_offset], msg.data_len);

                // 非阻塞发送到解析队列
                if (xQueueSend(g_sniffer_queue, &msg, 0) != pdTRUE) {
                    g_stats.queue_overflows++;
                }
            } else {
                g_stats.non_rid_vendor_ie++;

                // 采样：每 64 个非 RID Vendor IE 入队 1 个用于诊断
                if ((g_stats.non_rid_vendor_ie & 0x3F) == 0) {
                    sniffer_msg_t msg;
                    memset(&msg, 0, sizeof(msg));
                    memcpy(msg.src_mac, hdr->addr2, 6);
                    msg.rssi = pkt->rx_ctrl.rssi;
                    msg.channel = pkt->rx_ctrl.channel;
                    msg.timestamp_ms = esp_log_timestamp();
                    msg.oui_type = oui_type;
                    msg.is_rid = false;
                    msg.msg_type = MSG_TYPE_NON_RID_VENDOR;
                    msg.oui[0] = oui0;
                    msg.oui[1] = oui1;
                    msg.oui[2] = oui2;
                    msg.has_vendor_ie = true;
                    memcpy(msg.ssid, ssid, ssid_len);
                    msg.ssid_len = ssid_len;

                    // 提取 Vendor IE 数据（跳过 OUI + Type，保留全部）
                    uint16_t data_offset = offset + 6;
                    uint16_t vendor_data_len = ie_length - 4;
                    msg.data_len = (vendor_data_len < sizeof(msg.data)) ? vendor_data_len : sizeof(msg.data);
                    memcpy(msg.data, &ie_start[data_offset], msg.data_len);

                    if (xQueueSend(g_sniffer_queue, &msg, 0) != pdTRUE) {
                        g_stats.queue_overflows++;
                    }
                }
            }
        }

        offset += 2 + ie_length;
    }

    // 普通 Beacon 采样：每 128 个无 Vendor IE 的 Beacon 入队 1 个用于诊断
    // 用于检查树莓派信号是否以普通 Beacon 形式到达
    static uint32_t s_beacon_no_vendor_count = 0;
    if (subtype == 8 && !has_vendor_ie) {
        s_beacon_no_vendor_count++;
        if ((s_beacon_no_vendor_count & 0x7F) == 0) {
            sniffer_msg_t msg;
            memset(&msg, 0, sizeof(msg));
            memcpy(msg.src_mac, hdr->addr2, 6);
            msg.rssi = pkt->rx_ctrl.rssi;
            msg.channel = pkt->rx_ctrl.channel;
            msg.timestamp_ms = esp_log_timestamp();
            msg.is_rid = false;
            msg.msg_type = MSG_TYPE_BEACON_NO_VENDOR;
            msg.has_vendor_ie = false;
            memcpy(msg.ssid, ssid, ssid_len);
            msg.ssid_len = ssid_len;

            if (xQueueSend(g_sniffer_queue, &msg, 0) != pdTRUE) {
                g_stats.queue_overflows++;
            }
        }
    }
}

/* ---- 信道保持任务 ---- */

static void channel_hold_task(void *pvParameter) {
    char msg[64];
    snprintf(msg, sizeof(msg), "Channel hold started, locked to channel %d", FIXED_CHANNEL);
    json_debug("RID_SNIFF", msg);

    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        esp_err_t ret = esp_wifi_set_channel(FIXED_CHANNEL, WIFI_SECOND_CHAN_NONE);
        if (ret != ESP_OK) {
            snprintf(msg, sizeof(msg), "Failed to set channel %d: %s", FIXED_CHANNEL, esp_err_to_name(ret));
            json_warning("RID_SNIFF", msg);
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

/* ---- 公开接口 ---- */

QueueHandle_t crid_sniffer_get_queue(void) {
    return g_sniffer_queue;
}

sniffer_stats_t *crid_sniffer_get_stats(void) {
    return &g_stats;
}

esp_err_t crid_sniffer_init(void) {
    memset(&g_stats, 0, sizeof(g_stats));
    char err[64];

    // 1. 创建消息队列
    g_sniffer_queue = xQueueCreate(SNIFFER_QUEUE_SIZE, sizeof(sniffer_msg_t));
    if (g_sniffer_queue == NULL) {
        json_error("RID_SNIFF", "Failed to create sniffer queue!");
        return ESP_ERR_NO_MEM;
    }

    // 2. 初始化 Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Wi-Fi init failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    // 3. 设置 NULL 模式
    ret = esp_wifi_set_mode(WIFI_MODE_NULL);
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Wi-Fi mode failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Wi-Fi start failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    // 4. 设置混杂模式
    esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_cb);

    // 设置 filter：接收所有帧（不过滤）
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL
    };
    esp_wifi_set_promiscuous_filter(&filter);

    ret = esp_wifi_set_promiscuous(true);
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Promiscuous mode failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    // 5. 锁定监听信道
    ret = esp_wifi_set_channel(FIXED_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Set channel failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    snprintf(err, sizeof(err), "Wi-Fi monitor mode enabled, promiscuous ON, channel %d", FIXED_CHANNEL);
    json_debug("RID_SNIFF", err);

    return ESP_OK;
}

void crid_sniffer_start_channel_hold(void) {
    xTaskCreate(channel_hold_task, "ch_hold",
                CH_HOLD_TASK_STACK, NULL, CH_HOLD_TASK_PRIO, NULL);
}
