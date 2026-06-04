# ESP32 C-RID (Remote ID Scanner & Simulator)

基于 ESP-IDF 6.0.1 的无人机 Remote ID 收发系统，遵循 ASTM F3411-22 / ASD-STAN prEN 4709-002 标准。主要部署在 ESP32-S3，兼容 ESP32-C3 等模组。

## 功能

- **接收器 (Scanner)**：Wi-Fi 混杂模式抓包，解析 ASTM F3411-22 / ASD-STAN prEN 4709-002 / GB 42590-2023 标准的 Remote ID 信号。所有输出以 JSON 格式通过串口输出，每行一条完整的 JSON 对象。支持双端口输出：UAV 数据走 UART1（GPIO17），调试信息走 USB CDC。
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
│   ├── crid_json.c/h              # JSON 格式化输出（所有信息统一 JSON 格式）
│   └── crid_rx_types.h           # 接收端类型定义、OUI、配置常量
├── main_tx/                      # 发射器
│   ├── crid-sim.c                # 主入口
│   ├── crid_wifi.c/h             # 原始 Beacon 帧发送
│   ├── crid_messages.c/h         # 消息编码（Message Counter + Packed）
│   ├── crid_config.c/h           # 配置管理（坐标/速度/ID 等）
│   └── crid_patrol.c/h           # 巡游路径模拟
├── components/opendroneid/       # OpenDroneID 官方库（解码核心）
├── partition_table/              # 分区表
├── tools/                        # 上位机工具
│   └── json_monitor.py           # 串口 JSON 监视器（实时展示 + 退出摘要）
├── CMakeLists.txt                # 根构建文件（通过 MAIN_DIR 切换目标）
├── sdkconfig.defaults            # 默认配置覆盖
└── dependencies.lock             # 依赖锁定
```

## 协议参数

| 参数 | 值 |
|------|-----|
| OUI | `FA:0B:BC`（国标）/ `FF:FF:5F`（ASTM） |
| Vendor Type | `0x0D` |
| Wi-Fi 信道 | 6（2.437 GHz） |
| 广播间隔 | 1 Hz |
| IE ID | 221（Vendor Specific） |
| 消息格式 | Message Pack（含 Message Counter） |

## 接收端输出（JSON 格式，双端口）

所有输出以 JSON 格式发送，每行一条完整的 JSON 对象。

### 端口分配

| 端口 | 物理接口 | 内容 |
|------|---------|------|
| **数据端口** | UART1（GPIO17 TX, 115200 baud）| UAV 解析数据（`uav_discovery` / `uav_update` / `uav_status` / `uav_timeout` / `status`） |
| **调试端口** | USB CDC（stdout）| 启动信息、调试、告警、错误、解码诊断（`startup` / `debug` / `warning` / `error` / `decode_fail`） |

> 数据端口同时也会输出到 USB CDC，方便开发调试。上位机可以只连接 UART1 接收纯净的 UAV 数据流。

### 事件类型

| evt | 输出端口 | 说明 | 触发时机 |
|-----|---------|------|---------|
| `startup` | 调试 | 系统启动信息 | 启动时（横幅 + 详细参数） |
| `status` | 数据 | 定期状态统计 | 每 60 秒（抓包速率、活跃 UAV 数等） |
| `uav_discovery` | 数据 | 新 UAV 发现 | 首次收到某 MAC 的 RID 信号 |
| `uav_update` | 数据 | UAV 解析数据更新 | 每次解码成功后（完整字段） |
| `uav_status` | 数据 | UAV 活跃状态 | 每 60 秒（含 age_ms） |
| `uav_timeout` | 数据 | UAV 超时移除 | 5 分钟无信号后清理 |
| `uav_detail` | 数据 | UAV 完整详情 | 按需调用（所有 Basic ID、Auth 等） |
| `warning` | 调试 | 告警 | 追踪表满、信道设置失败等 |
| `error` | 调试 | 错误 | 初始化失败、任务创建失败等 |
| `debug` | 调试 | 调试信息 | 任务启动确认、Wi-Fi 模式确认等 |
| `decode_fail` | 调试 | 解码失败诊断 | 每 32 次失败输出一次（含原始字节） |

### UAV 数据字段

每次 `uav_update` 包含完整的无人机 Remote ID 数据：

```json
{
  "evt": "uav_update",
  "ts": 12345,
  "mac": "AA:BB:CC:DD:EE:FF",
  "rssi": -45,
  "channel": 6,
  "transport": "Wi-Fi Beacon",
  "protocol": "ASTM F3411",
  "msg_count": 10,
  "basic_id": {
    "id_type": "serial_number",
    "ua_type": "helicopter_or_multirotor",
    "uas_id": "SN12345678"
  },
  "location": {
    "status": "airborne",
    "latitude": 22.1234567,
    "longitude": 113.1234567,
    "alt_baro": 120.5,
    "alt_geo": 125.3,
    "height": 100.0,
    "height_ref": "over_takeoff",
    "direction": 45.0,
    "speed_h": 5.50,
    "speed_v": 0.00,
    "acc_h": "...",
    "acc_v": "...",
    "acc_baro": "...",
    "acc_speed": "...",
    "timestamp": 1234.5,
    "ts_acc": 1
  },
  "system": { "operator_loc_type": "takeoff", ... },
  "self_id": { "type": "text", "desc": "Drone #1" },
  "operator_id": { "type": 1, "id": "OP12345" },
  "auth": []
}
```

- 所有枚举值使用 snake_case 字符串（如 `"serial_number"`、`"airborne"`），便于下游解析
- 未获取到的字段为 `null`
- 字符串字段经过 JSON 转义，安全可解析

## 上位机监视工具

`tools/json_monitor.py` 从串口读取 ESP32 的 JSON 输出并实时展示。

```bash
# 安装依赖
pip install pyserial

# 自动检测串口，默认仅显示 UAV 数据
python3 tools/json_monitor.py

# 指定串口和模式
python3 tools/json_monitor.py -p /dev/tty.usbserial-A5069RR4 -m data
python3 tools/json_monitor.py -m debug      # 仅调试信息（启动/告警/解码失败）
python3 tools/json_monitor.py -m static     # 仅静态消息（basic_id/self_id/operator_id/system/auth，自动去重）
python3 tools/json_monitor.py -m all        # 所有事件

# 退出时不显示摘要
python3 tools/json_monitor.py --no-summary
```

### 显示模式

| 模式 | 说明 |
|------|------|
| `data`（默认） | UAV 解析数据：discovery / update / status / timeout |
| `debug` | 调试信息：startup / warning / error / debug / decode_fail |
| `static` | 仅无人机静态消息（basic_id、self_id、operator_id、system、auth），每个 MAC 每种字段只显示一次 |
| `all` | 全部事件 |

### 退出摘要

按 `Ctrl+C` 退出时自动打印会话摘要，包括运行时长、发现/活跃 UAV 数量、解码失败计数、全局状态快照、每个 UAV 的详细信息和活跃状态。可用 `--no-summary` 跳过。

## 协议支持

| 标准 | OUI | 格式 |
|------|-----|------|
| ASTM F3411-22a | `FF:FF:5F` | Message Pack（`0xF2`），Message Counter + Packed 消息 |
| GB 42590-2023 | `FA:0B:BC` | 国标格式（`0xF1`），Message Counter + 3 字节管理信息 + 消息体 |

> 解析器自动识别 ASTM（`0xF2`）和国标（`0xF1`）格式，无需手动切换。
