# ESP32 C-RID (Remote ID Scanner & Simulator)

基于 ESP-IDF 6.0.1 的无人机 Remote ID 收发系统，遵循 ASTM F3411-22 / ASD-STAN prEN 4709-002 标准。主要部署在 ESP32-S3，兼容 ESP32-C3 等模组。

## 功能

- **接收器 (Scanner)**：Wi-Fi 混杂模式抓包，解析 ASTM F3411-22 / ASD-STAN prEN 4709-002 标准的 Remote ID 信号。新发现无人机仅打印 1 行精简信息，每分钟输出状态摘要。
- **发射器 (Simulator)**：模拟无人机广播 Beacon 帧（巡游路径），Vendor IE 包含 Message Counter + Packed 消息。

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
├── main_rx/                      # 接收器
│   ├── crid_scan_main.c          # 主入口：启动 sniffer / parser / monitor 任务
│   ├── crid_sniffer.c/h          # Wi-Fi 混杂模式抓包，ISR 安全队列
│   ├── crid_parser.c/h           # opendroneid 库解码（Message Pack / 单消息）
│   ├── crid_tracker.c/h          # 无人机追踪表（线程安全，超时清理）
│   ├── crid_display.c/h          # 精简输出：新 UAV 1 行 + 每分钟状态摘要
│   └── crid_rx_types.h           # 接收端类型定义、OUI、配置常量
├── main_tx/                      # 发射器
│   ├── crid-sim.c                # 主入口
│   ├── crid_wifi.c/h             # 原始 Beacon 帧发送
│   ├── crid_messages.c/h         # 消息编码（Message Counter + Packed）
│   ├── crid_config.c/h           # 配置管理（坐标/速度/ID 等）
│   └── crid_patrol.c/h           # 巡游路径模拟
├── components/opendroneid/       # OpenDroneID 官方库（解码核心）
├── partition_table/              # 分区表
├── archive/                      # 历史版本和独立工具
├── CMakeLists.txt                # 根构建文件（通过 MAIN_DIR 切换目标）
├── sdkconfig.defaults            # 默认配置覆盖
└── dependencies.lock             # 依赖锁定
```

## 协议参数

| 参数 | 值 |
|------|-----|
| OUI | `FA:0B:BC` |
| Vendor Type | `0x0D` |
| Wi-Fi 信道 | 6（2.437 GHz） |
| 广播间隔 | 1 Hz |
| IE ID | 221（Vendor Specific） |
| 消息格式 | Message Pack（含 Message Counter） |

## 接收端输出

- **新 UAV 发现**：1 行精简日志 `[MAC] ID @ lat,lng alt=Xm spd=X.Xm/s rssi=-XXdBm`
- **重复更新**：不显示，仅内部追踪
- **每分钟摘要**：monitor_task 列出活跃 UAV 状态、抓包速率统计
