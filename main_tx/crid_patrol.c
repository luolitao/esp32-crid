#include "crid_patrol.h"
#include <math.h>
#include "esp_log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "CN_C-RID_PATROL";

void crid_patrol_step(cn_crid_config_t *config) {
    if (config == NULL) return;

    config->time_counter += 1.0f;

    float angle = config->time_counter * config->patrol_speed;

    // 圆形巡游路径
    float new_lat = config->base_latitude +
                    config->patrol_radius_lat * cosf(angle);
    float new_lon = config->base_longitude +
                    config->patrol_radius_lon * sinf(angle);

    // 高度周期性缓慢变化：在 45m ~ 55m 之间波动，周期约 62.8 秒
    // 使用基准高度计算偏移，避免累积误差
    float alt_offset = 5.0f * sinf(config->time_counter * 0.1f);
    float new_alt_msl = config->base_altitude_msl + alt_offset;
    float new_alt_agl = new_alt_msl - 5.0f;

    // 速度变化
    float new_speed_h = 1.5f + 0.5f * sinf(config->time_counter * 0.1f);
    // 垂直速度 = 高度对时间的导数：d/dt[5*sin(0.1*t)] = 0.5*cos(0.1*t)
    // 幅度 ±0.5 m/s，符合低速飞行特征
    float new_speed_v = 0.5f * cosf(config->time_counter * 0.1f);

    // 航向（基于运动切线方向，正北为0°，顺时针增加）
    // 位置: lat = base + r_lat*cos(angle), lon = base + r_lon*sin(angle)
    // d(lat)/dt = -r_lat*sin(angle)*omega, d(lon)/dt = r_lon*cos(angle)*omega
    // 航向 = atan2(dlon, dlat) （注意：atan2(x, y) 给出从y轴顺时针的角度）
    float dlat = -config->patrol_radius_lat * sinf(angle) * config->patrol_speed;
    float dlon = config->patrol_radius_lon * cosf(angle) * config->patrol_speed;
    float new_heading = atan2f(dlon, dlat) * 180.0f / M_PI;
    if (new_heading < 0.0f) new_heading += 360.0f;

    crid_config_update_position(config, new_lat, new_lon,
                                new_alt_msl, new_alt_agl,
                                new_speed_h, new_speed_v,
                                new_heading);

    ESP_LOGI(TAG, "Patrol step: pos=(%.6f,%.6f), alt=%.1fm, hdg=%.1f°, spd=%.1fm/s",
             new_lat, new_lon, new_alt_msl, new_heading, new_speed_h);
}
