# ESP32 C-RID OTA Web API Documentation

## 概述

本文档描述了ESP32 C-RID项目中OTA Web服务提供的RESTful API接口。这些接口可用于获取设备和系统状态信息，以及监控OTA更新进度。

## API端点

### 1. 系统信息接口

#### 获取系统信息
```
GET /api/system/info
```

**描述**: 获取设备的硬件和软件系统信息

**响应格式**: JSON

**响应示例**:
```json
{
  "chip_model": "ESP32-S3",
  "chip_revision": 3,
  "chip_cores": 2,
  "flash_size": 8388608,
  "free_heap": 234567,
  "min_free_heap": 123456,
  "version": "0.1.0-dev",
  "build_date": "Jun 17 2026",
  "build_time": "15:30:45"
}
```

**字段说明**:
- `chip_model`: 芯片型号
- `chip_revision`: 芯片修订版本
- `chip_cores`: CPU核心数
- `flash_size`: Flash大小（字节）
- `free_heap`: 当前可用堆内存（字节）
- `min_free_heap`: 最小可用堆内存（字节）
- `version`: 固件版本
- `build_date`: 构建日期
- `build_time`: 构建时间

### 2. 设备状态接口

#### 获取设备状态
```
GET /api/device/status
```

**描述**: 获取设备的运行状态和网络信息

**响应格式**: JSON

**响应示例**:
```json
{
  "uptime_seconds": 3665,
  "wifi_ssid": "MyNetwork",
  "wifi_rssi": -45,
  "heap_free": 234567,
  "heap_min_free": 123456
}
```

**字段说明**:
- `uptime_seconds`: 设备运行时间（秒）
- `wifi_ssid`: 当前连接的WiFi网络名称
- `wifi_rssi`: WiFi信号强度（dBm）
- `heap_free`: 当前可用堆内存（字节）
- `heap_min_free`: 最小可用堆内存（字节）

### 3. OTA进度接口

#### 获取OTA更新进度
```
GET /api/ota/progress
```

**描述**: 获取当前OTA更新的进度信息

**响应格式**: JSON

**响应示例**:
```json
{
  "state": "in_progress",
  "progress": 45,
  "received": 234567,
  "total": 524288,
  "error": "No error"
}
```

**字段说明**:
- `state`: OTA状态（idle, started, in_progress, completed, failed）
- `progress`: 更新进度百分比（0-100）
- `received`: 已接收字节数
- `total`: 总字节数
- `error`: 错误信息（如果有）

## 使用示例

### JavaScript (浏览器环境)
```javascript
// 获取系统信息
fetch('/api/system/info')
  .then(response => response.json())
  .then(data => {
    console.log('Chip Model:', data.chip_model);
    console.log('Free Heap:', data.free_heap);
  });

// 获取设备状态
fetch('/api/device/status')
  .then(response => response.json())
  .then(data => {
    console.log('Uptime:', data.uptime_seconds);
    console.log('WiFi RSSI:', data.wifi_rssi);
  });

// 监控OTA进度
function monitorOTAProgress() {
  fetch('/api/ota/progress')
    .then(response => response.json())
    .then(data => {
      console.log('OTA Progress:', data.progress + '%');
      if (data.state === 'in_progress') {
        setTimeout(monitorOTAProgress, 1000);
      }
    });
}
```

### Python (使用requests库)
```python
import requests

# 获取系统信息
response = requests.get('http://esp32-device-ip/api/system/info')
system_info = response.json()
print(f"Chip Model: {system_info['chip_model']}")

# 获取设备状态
response = requests.get('http://esp32-device-ip/api/device/status')
device_status = response.json()
print(f"Uptime: {device_status['uptime_seconds']} seconds")

# 获取OTA进度
response = requests.get('http://esp32-device-ip/api/ota/progress')
ota_progress = response.json()
print(f"OTA Progress: {ota_progress['progress']}%")
```

## 错误处理

所有API端点在发生错误时都会返回相应的HTTP状态码：
- `200 OK`: 请求成功
- `404 Not Found`: 请求的端点不存在
- `500 Internal Server Error`: 服务器内部错误

响应体中的JSON对象可能包含错误信息字段。