#include "crid_wifi.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CN_C-RID_WIFI";

esp_err_t crid_wifi_init(uint8_t channel) {
    esp_err_t ret;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    ret = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_promiscuous(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_promiscuous failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Wi-Fi initialized: channel=%u, promiscuous mode", channel);
    return ESP_OK;
}

esp_err_t crid_wifi_send_raw_frame(const uint8_t *frame, uint16_t len) {
    if (frame == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 四级 fallback: STA/AP × (no-seq / with-seq)
    static const struct {
        wifi_interface_t iface;
        bool enable_seq;
        const char *desc;
    } fallbacks[] = {
        {WIFI_IF_STA, false, "STA no seq"},
        {WIFI_IF_STA, true,  "STA with seq"},
        {WIFI_IF_AP,  false, "AP no seq"},
        {WIFI_IF_AP,  true,  "AP with seq"},
    };

    esp_err_t ret = ESP_FAIL;
    for (size_t i = 0; i < sizeof(fallbacks) / sizeof(fallbacks[0]); i++) {
        ret = esp_wifi_80211_tx(fallbacks[i].iface, frame, len, fallbacks[i].enable_seq);
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "TX OK (%s)", fallbacks[i].desc);
            return ESP_OK;
        }
        ESP_LOGD(TAG, "TX attempt %s: %s", fallbacks[i].desc, esp_err_to_name(ret));
    }

    ESP_LOGE(TAG, "All TX methods failed");
    return ret;
}
