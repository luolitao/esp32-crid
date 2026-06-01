# ESP32 C-RID (China Remote ID)

基于 ESP-IDF 6.0.1 的中国 GB42590-2023 无人机 RemoteID 收发系统。主要部署在 ESP32-S3，兼容 ESP32-C3、ESP32-C5 等模组。

## 功能

- **接收器 (Scanner)**：混杂模式抓包，解析 ASTM F3411 / ASD-STAN prEN 4709-002 / GB42590 三种标准的 RemoteID 信号
- **发射器 (Simulator)**：模拟中国 C-RID 无人机广播 Beacon 帧（越秀山坐标巡游路径）

## 构建

```bash
# 接收器（默认）
idf.py set-target esp32s3 -DMAIN_DIR=main_rx
idf.py build

# 发射器
idf.py set-target esp32s3 -DMAIN_DIR=main_tx
idf.py build
```

## 项目结构

```
├── main_rx/                    # 接收器（活跃）
│   ├── CMakeLists.txt
│   └── esp32_crid_scan.c       # 核心：sniffer 回调 → 队列 → opendroneid 解析
├── main_tx/                    # 发射器（活跃）
│   ├── crid-sim.c              # 主入口
│   ├── crid_wifi.c/h           # 原始帧发送
│   ├── crid_messages.c/h       # C-RID 消息编码
│   ├── crid_config.c/h         # 配置管理
│   └── crid_patrol.c/h         # 巡游路径模拟
├── components/opendroneid/     # OpenDroneID 官方库
├── archive/                    # 历史版本和独立工具
├── CMakeLists.txt              # 根构建文件（通过 MAIN_DIR 切换目标）
├── sdkconfig                   # SDK 配置
├── sdkconfig.defaults          # 默认配置覆盖
└── dependencies.lock           # 依赖锁定
```

## 协议参数

| 参数 | 值 |
|------|-----|
| OUI | `FA:0B:BC`（中国 GB42590-2023） |
| Vendor Type | `0x0D` |
| Wi-Fi 信道 | 6（2.4GHz） |
| 广播频率 | 1 Hz |
| IE ID | 221（Vendor Specific） |
