#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "cJSON.h" // 需要添加 cJSON 组件
#include "opendroneid.h"

static const char *TAG = "RemoteID_Scanner";

// --- Configuration ---
#define MAX_UAVS 8
#define ID_SIZE (ODID_ID_SIZE + 10)
#define SCAN_TIME_MS 10000
#define CHANNEL_POLL_INTERVAL_MS 300
#define CHANNEL_SWITCH_DELAY_MS 50

// Wi-Fi 信道列表（2.4GHz）
static const uint8_t wifi_channels[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static const int channel_count = sizeof(wifi_channels) / sizeof(wifi_channels[0]);
static int current_channel_index = 0;

// --- NAN Specific Defines ---
#define NAN_OUI_BYTE_1 0x50
#define NAN_OUI_BYTE_2 0x6F
#define NAN_OUI_BYTE_3 0x9A
#define NAN_OUI_TYPE 0x13
#define NAN_VENDOR_SPECIFIC_CATEGORY 0x7f
#define NAN_PUBLIC_ACTION_CATEGORY 0x0d

// --- Data Structure ---
struct id_data {
    int flag;
    uint8_t mac[6];
    uint32_t last_seen_ms;
    char op_id[ID_SIZE];
    char uav_id[ID_SIZE];
    double lat_d, long_d;
    float altitude_msl, height_agl, speed, heading;
    int rssi;
    uint8_t last_channel;
};

static struct id_data uavs[MAX_UAVS];

// --- Function Prototypes ---
static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static void packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);
static struct id_data *find_uav_entry(uint8_t *mac);
static void parse_odid_wifi_frame(wifi_promiscuous_pkt_t *pkt, const uint8_t *source_mac);
static void parse_odid_nan_frame(wifi_promiscuous_pkt_t *pkt, const uint8_t *source_mac);
static void print_mac(const uint8_t *mac);
static void send_odid_data_json(const char* type, const uint8_t* mac, const char* uav_id, 
                               double lat, double lon, float alt_msl, float alt_agl, 
                               float speed, float heading, int rssi, uint8_t channel);

// --- Initialize WiFi in Promiscuous Mode ---
static void wifi_sniffer_init(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_promiscuous_rx_cb(&packet_handler);
    esp_wifi_set_promiscuous(true);

    wifi_sniffer_set_channel(wifi_channels[0]);
    ESP_LOGI(TAG, "Wi-Fi sniffer initialized on channel %d (Multi-channel enabled)", wifi_channels[0]);
}

// --- Set Wi-Fi Channel ---
static void wifi_sniffer_set_channel(uint8_t channel) {
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    vTaskDelay(pdMS_TO_TICKS(CHANNEL_SWITCH_DELAY_MS));
}

// --- Find or create an entry for a UAV based on MAC address ---
static struct id_data *find_uav_entry(uint8_t *mac) {
    struct id_data *empty_slot = NULL;

    for (int i = 0; i < MAX_UAVS; i++) {
        if (memcmp(uavs[i].mac, mac, 6) == 0) {
            uavs[i].last_seen_ms = esp_log_timestamp();
            return &uavs[i];
        }
        if (!uavs[i].flag && !empty_slot) {
            empty_slot = &uavs[i];
        }
    }

    if (empty_slot) {
        memset(empty_slot, 0, sizeof(struct id_data));
        memcpy(empty_slot->mac, mac, 6);
        empty_slot->flag = 1;
        empty_slot->last_seen_ms = esp_log_timestamp();
        ESP_LOGI(TAG, "New UAV detected (MAC: ");
        print_mac(mac);
        ESP_LOGI(TAG, ")");
        return empty_slot;
    }
    return NULL;
}

// --- Send ODID data as JSON via USB CDC ---
static void send_odid_data_json(const char* type, const uint8_t* mac, const char* uav_id, 
                               double lat, double lon, float alt_msl, float alt_agl, 
                               float speed, float heading, int rssi, uint8_t channel) {
    cJSON *json = cJSON_CreateObject();
    
    cJSON_AddStringToObject(json, "type", type);
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON_AddStringToObject(json, "mac", mac_str);
    cJSON_AddStringToObject(json, "uav_id", uav_id);
    cJSON_AddNumberToObject(json, "channel", channel);
    cJSON_AddNumberToObject(json, "rssi", rssi);
    cJSON_AddNumberToObject(json, "latitude", lat);
    cJSON_AddNumberToObject(json, "longitude", lon);
    cJSON_AddNumberToObject(json, "altitude_msl", alt_msl);
    cJSON_AddNumberToObject(json, "altitude_agl", alt_agl);
    cJSON_AddNumberToObject(json, "speed", speed);
    cJSON_AddNumberToObject(json, "heading", heading);

    char *json_str = cJSON_PrintUnformatted(json);
    printf("%s\n", json_str); // USB CDC 输出
    free(json_str);
    cJSON_Delete(json);
}

// --- Parse OpenDroneID data from Wi-Fi Management frame ---
static void parse_odid_wifi_frame(wifi_promiscuous_pkt_t *pkt, const uint8_t *source_mac) {
    const uint8_t *frame = pkt->payload;
    uint16_t frame_len = pkt->rx_ctrl.sig_len;
    if (frame_len < 24) return;

    const uint8_t *ie_ptr = frame + 24;
    uint16_t remaining_len = frame_len - 24;

    while (remaining_len >= 2) {
        uint8_t ie_id = ie_ptr[0];
        uint8_t ie_len = ie_ptr[1];
        if (ie_id == 221 && ie_len >= 5) {
            if (ie_ptr[2] == 0xFF && ie_ptr[3] == 0xFF && ie_ptr[4] == 0x5F) {
                const uint8_t *odid_data = ie_ptr + 2 + 3;
                uint8_t odid_len = ie_len - 3;
                if (odid_len > 0) {
                    uint8_t raw_msg_type = odid_data[0] & 0x0F;

                    if (raw_msg_type == ODID_MESSAGETYPE_BASIC_ID) {
                        ODID_BasicID_data basic_dec;
                        int decode_result = decodeBasicIDMessage(&basic_dec, (ODID_BasicID_encoded*)odid_data);
                        if (decode_result == 0) {
                            struct id_data *uav = find_uav_entry(source_mac);
                            if (uav) {
                                uav->rssi = pkt->rx_ctrl.rssi;
                                uav->last_channel = pkt->rx_ctrl.channel;

                                if (basic_dec.IDType >= ODID_IDTYPE_NONE && basic_dec.IDType <= ODID_IDTYPE_SPECIFIC_SESSION_ID) {
                                    if (basic_dec.IDType == ODID_IDTYPE_SERIAL_NUMBER) {
                                        snprintf(uav->uav_id, ID_SIZE, "S:%.*s", ODID_ID_SIZE, basic_dec.UASID);
                                    } else if (basic_dec.IDType == ODID_IDTYPE_CAA_REGISTRATION_ID) {
                                        snprintf(uav->uav_id, ID_SIZE, "R:%.*s", ODID_ID_SIZE, basic_dec.UASID);
                                    } else {
                                        snprintf(uav->uav_id, ID_SIZE, "T%u:%.*s", basic_dec.IDType, ODID_ID_SIZE, basic_dec.UASID);
                                    }
                                    
                                    // 通过 USB CDC 发送 JSON
                                    send_odid_data_json("basic_id", source_mac, uav->uav_id, 
                                                       uav->lat_d, uav->long_d, uav->altitude_msl, 
                                                       uav->height_agl, uav->speed, uav->heading, 
                                                       uav->rssi, uav->last_channel);
                                }
                            }
                        }
                    } else if (raw_msg_type == ODID_MESSAGETYPE_LOCATION) {
                        ODID_Location_encoded *loc_enc = (ODID_Location_encoded*)odid_data;
                        ODID_Location_data loc_dec;
                        int ret = decodeLocationMessage(&loc_dec, loc_enc);
                        if (ret == 0) {
                            struct id_data *uav = find_uav_entry(source_mac);
                            if (uav) {
                                uav->lat_d = loc_dec.Latitude;
                                uav->long_d = loc_dec.Longitude;
                                uav->altitude_msl = loc_dec.AltitudeGeo;
                                uav->height_agl = loc_dec.Height;
                                uav->speed = loc_dec.SpeedHorizontal;
                                uav->heading = loc_dec.Direction;
                                uav->rssi = pkt->rx_ctrl.rssi;
                                uav->last_channel = pkt->rx_ctrl.channel;
                                
                                // 通过 USB CDC 发送 JSON
                                send_odid_data_json("location", source_mac, uav->uav_id, 
                                                   uav->lat_d, uav->long_d, uav->altitude_msl, 
                                                   uav->height_agl, uav->speed, uav->heading, 
                                                   uav->rssi, uav->last_channel);
                            }
                        }
                    }
                    break;
                }
            }
        }
        ie_ptr += 2 + ie_len;
        if (remaining_len < 2 + ie_len) break;
        remaining_len -= (2 + ie_len);
    }
}

// --- Parse OpenDroneID data from NAN frame ---
static void parse_odid_nan_frame(wifi_promiscuous_pkt_t *pkt, const uint8_t *source_mac) {
    const uint8_t *frame = pkt->payload;
    uint16_t frame_len = pkt->rx_ctrl.sig_len;
    uint8_t subtype = (frame[0] >> 4) & 0x0F;
    uint8_t category = frame[4];

    if (category == NAN_PUBLIC_ACTION_CATEGORY) {
        if (frame_len < 10) return;
        if (frame[5] == NAN_OUI_BYTE_1 && frame[6] == NAN_OUI_BYTE_2 && frame[7] == NAN_OUI_BYTE_3 && frame[8] == NAN_OUI_TYPE) {
            const uint8_t *nan_payload = frame + 9;
            uint16_t nan_payload_len = frame_len - 9;

            const uint8_t *ie_ptr = nan_payload;
            uint16_t remaining_len = nan_payload_len;

            while (remaining_len >= 2) {
                uint8_t ie_id = ie_ptr[0];
                uint8_t ie_len = ie_ptr[1];
                if (ie_id == 221 && ie_len >= 5) {
                    if (ie_ptr[2] == 0xFF && ie_ptr[3] == 0xFF && ie_ptr[4] == 0x5F) {
                        const uint8_t *odid_data = ie_ptr + 2 + 3;
                        uint8_t odid_len = ie_len - 3;
                        if (odid_len > 0) {
                            uint8_t raw_msg_type = odid_data[0] & 0x0F;

                            if (raw_msg_type == ODID_MESSAGETYPE_BASIC_ID) {
                                ODID_BasicID_data basic_dec;
                                int decode_result = decodeBasicIDMessage(&basic_dec, (ODID_BasicID_encoded*)odid_data);
                                if (decode_result == 0) {
                                    struct id_data *uav = find_uav_entry(source_mac);
                                    if (uav) {
                                        uav->rssi = pkt->rx_ctrl.rssi;
                                        uav->last_channel = pkt->rx_ctrl.channel;

                                        if (basic_dec.IDType >= ODID_IDTYPE_NONE && basic_dec.IDType <= ODID_IDTYPE_SPECIFIC_SESSION_ID) {
                                            if (basic_dec.IDType == ODID_IDTYPE_SERIAL_NUMBER) {
                                                snprintf(uav->uav_id, ID_SIZE, "S:%.*s", ODID_ID_SIZE, basic_dec.UASID);
                                            } else if (basic_dec.IDType == ODID_IDTYPE_CAA_REGISTRATION_ID) {
                                                snprintf(uav->uav_id, ID_SIZE, "R:%.*s", ODID_ID_SIZE, basic_dec.UASID);
                                            } else {
                                                snprintf(uav->uav_id, ID_SIZE, "T%u:%.*s", basic_dec.IDType, ODID_ID_SIZE, basic_dec.UASID);
                                            }
                                            
                                            // 通过 USB CDC 发送 JSON
                                            send_odid_data_json("basic_id", source_mac, uav->uav_id, 
                                                               uav->lat_d, uav->long_d, uav->altitude_msl, 
                                                               uav->height_agl, uav->speed, uav->heading, 
                                                               uav->rssi, uav->last_channel);
                                        }
                                    }
                                }
                            } else if (raw_msg_type == ODID_MESSAGETYPE_LOCATION) {
                                ODID_Location_encoded *loc_enc = (ODID_Location_encoded*)odid_data;
                                ODID_Location_data loc_dec;
                                int ret = decodeLocationMessage(&loc_dec, loc_enc);
                                if (ret == 0) {
                                    struct id_data *uav = find_uav_entry(source_mac);
                                    if (uav) {
                                        uav->lat_d = loc_dec.Latitude;
                                        uav->long_d = loc_dec.Longitude;
                                        uav->altitude_msl = loc_dec.AltitudeGeo;
                                        uav->height_agl = loc_dec.Height;
                                        uav->speed = loc_dec.SpeedHorizontal;
                                        uav->heading = loc_dec.Direction;
                                        uav->rssi = pkt->rx_ctrl.rssi;
                                        uav->last_channel = pkt->rx_ctrl.channel;
                                        
                                        // 通过 USB CDC 发送 JSON
                                        send_odid_data_json("location", source_mac, uav->uav_id, 
                                                           uav->lat_d, uav->long_d, uav->altitude_msl, 
                                                           uav->height_agl, uav->speed, uav->heading, 
                                                           uav->rssi, uav->last_channel);
                                    }
                                }
                            }
                            break;
                        }
                    }
                }
                ie_ptr += 2 + ie_len;
                if (remaining_len < 2 + ie_len) break;
                remaining_len -= (2 + ie_len);
            }
        }
    }
}

// --- Helper function to print MAC address ---
static void print_mac(const uint8_t *mac) {
    printf("%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// --- Promiscuous Mode Callback ---
static void packet_handler(void *buff, wifi_promiscuous_pkt_type_t type) {
    if (type == WIFI_PKT_MGMT) {
        wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buff;
        const uint8_t *source_mac = pkt->payload + 10;
        uint8_t subtype = (pkt->payload[0] >> 4) & 0x0F;
        uint8_t category = 0xFF;

        if (subtype == 8 || subtype == 5) { // Beacon or Probe Response
            parse_odid_wifi_frame(pkt, source_mac);
        } else if (subtype == 13) { // Action Frame
            category = pkt->payload[4];
            if (category == NAN_PUBLIC_ACTION_CATEGORY) {
                parse_odid_nan_frame(pkt, source_mac);
            }
        }
    }
}

// --- Channel Polling Task ---
void channel_poll_task(void *pvParameters) {
    while (1) {
        int channel = wifi_channels[current_channel_index];
        wifi_sniffer_set_channel(channel);
        ESP_LOGI(TAG, "Switched to channel %d", channel);

        vTaskDelay(pdMS_TO_TICKS(CHANNEL_POLL_INTERVAL_MS));
        current_channel_index = (current_channel_index + 1) % channel_count;
    }
}

// --- Print Summary Task (also via USB CDC) ---
void print_summary_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SCAN_TIME_MS);

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 发送摘要 JSON
        cJSON *summary = cJSON_CreateObject();
        cJSON_AddStringToObject(summary, "event", "summary");
        cJSON_AddNumberToObject(summary, "timestamp", esp_log_timestamp());
        
        cJSON *active_uavs = cJSON_CreateArray();
        int active_count = 0;
        
        for (int i = 0; i < MAX_UAVS; i++) {
            if (uavs[i].flag && (esp_log_timestamp() - uavs[i].last_seen_ms < SCAN_TIME_MS)) {
                active_count++;
                cJSON *uav = cJSON_CreateObject();
                char mac_str[18];
                snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", 
                         uavs[i].mac[0], uavs[i].mac[1], uavs[i].mac[2], uavs[i].mac[3], uavs[i].mac[4], uavs[i].mac[5]);
                cJSON_AddStringToObject(uav, "mac", mac_str);
                cJSON_AddStringToObject(uav, "uav_id", uavs[i].uav_id);
                cJSON_AddNumberToObject(uav, "channel", uavs[i].last_channel);
                cJSON_AddNumberToObject(uav, "rssi", uavs[i].rssi);
                cJSON_AddNumberToObject(uav, "last_seen_ms", uavs[i].last_seen_ms);
                
                if (uavs[i].lat_d != 0.0 || uavs[i].long_d != 0.0) {
                    cJSON_AddNumberToObject(uav, "latitude", uavs[i].lat_d);
                    cJSON_AddNumberToObject(uav, "longitude", uavs[i].long_d);
                    cJSON_AddNumberToObject(uav, "altitude_msl", uavs[i].altitude_msl);
                    cJSON_AddNumberToObject(uav, "altitude_agl", uavs[i].height_agl);
                    cJSON_AddNumberToObject(uav, "speed", uavs[i].speed);
                    cJSON_AddNumberToObject(uav, "heading", uavs[i].heading);
                }
                cJSON_AddItemToArray(active_uavs, uav);
            }
        }
        
        cJSON_AddNumberToObject(summary, "active_count", active_count);
        cJSON_AddItemToObject(summary, "uavs", active_uavs);

        char *json_str = cJSON_PrintUnformatted(summary);
        printf("%s\n", json_str);
        free(json_str);
        cJSON_Delete(summary);
    }
}

// --- Main Application Entry Point ---
void app_main(void) {
    ESP_LOGI(TAG, "Starting Multi-channel RemoteID Scanner (USB CDC Output)");

    // 初始化 USB CDC（自动重定向 printf）
    // 确保在 sdkconfig 中启用了 USB CDC

    wifi_sniffer_init();
    xTaskCreate(channel_poll_task, "channel_poll", 4096, NULL, 5, NULL);
    xTaskCreate(print_summary_task, "print_summary", 4096, NULL, 5, NULL);
}