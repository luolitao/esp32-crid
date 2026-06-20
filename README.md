

# ESP32 C-RID (Remote ID Scanner & Simulator)

基于 ESP-IDF 6.0.1 开发的开源无人机远程识别（Remote ID, 简称 RID）工具系统。本项目集成了 RID 信号的扫描接收（Scanner）**与**模拟发射（Simulator）双重功能，完美兼容国际与国内主流行业标准，并提供结构化的 JSON 串口数据输出，极易整合至边缘计算网关、无人机地面站或低空安全监测系统。

主要部署在 **ESP32-S3** 芯片，同时向下兼容 **ESP32-C3** 等模组。

---

## 🚀 核心特性

* **双模一体化**：通过编译指令轻松切换“接收器（Scanner）”与“发射器（Simulator）”模式。
* **多标准全面兼容**：
* 国际标准：**ASTM F3411-22a** / **ASD-STAN prEN 4709-002**
* 中国国标：**GB 42590-2023** / **GB 46750-2025**


* **智能协议解析**：解析器通过检测消息头部特征（ASTM 为 `0xF1`，GB 42590 为 `0xF1`，GB 46750 为 `0xFF`）实现自动无缝识别。
* **工业级双端口设计**：
* **数据端口 (UART1 / GPIO17)**：纯净输出解析后的无人机数据（单行 JSON），方便下游设备（如树莓派、PC）直接解析。
* **调试端口 (USB CDC)**：输出系统启动、异常告警及详尽的解码失败诊断数据。


* **高可靠追踪引擎**：内置线程安全的无人机追踪表（Tracker），支持新机发现、持续活跃度监视，以及 5 分钟无信号自动超时清理。
* **完善的配套工具**：内置 Python 上位机，支持数据去重、分类过滤以及会话结束时的全局状态快照统计。

---

## 🛠️ 协议核心参数

项目遵循各大标准的无线电底层规范，默认核心参数如下：

| 参数 | 设定值 / 说明 |
| --- | --- |
| **OUI（组织唯一标识符）** | `FA:0B:BC` （ASTM / 中国国标统一使用） |
| **Vendor Type** | `0x0D` |
| **Wi-Fi 信道** | Channel 6（2.437 GHz） |
| **广播间隔** | 1 Hz （每秒广播 1 次） |
| **IE ID** | 221（Vendor Specific） |
| **消息格式** | Message Pack（含 Message Counter）、GB 46750-2025 格式 |

---

## 📂 项目结构

```text
├── main_rx/                      # 接收器（Scanner）组件
│   ├── crid_scan_main.c          # 主入口：初始化并启动 sniffer / parser / monitor 任务
│   ├── crid_sniffer.c/h          # Wi-Fi 混杂模式（Sniffer）抓包，含 ISR 安全队列
│   ├── crid_parser.c/h           # 基于 opendroneid 官方库的解码核心（支持 ASTM / 国标）
│   ├── crid_tracker.c/h          # 无人机追踪表（线程安全、动态超时清理）
│   ├── crid_display.c/h          # 终端精简输出控制
│   ├── crid_json.c/h              # JSON 格式化输出模块
│   └── crid_rx_types.h           # 接收端通用类型定义与配置常量
├── main_tx/                      # 发射器（Simulator）组件
│   ├── crid-sim.c                # 主入口
│   ├── crid_wifi.c/h             # 原始 Beacon 帧构筑与无线发送
│   ├── crid_messages.c/h         # 消息编码（Message Counter + Packed）
│   ├── crid_config.c/h           # 轨迹/速度/ID 等仿真参数配置管理
│   └── crid_patrol.c/h           # 巡游路径模拟算法
├── components/opendroneid/       # OpenDroneID 官方解码核心库
├── tools/                        # 上位机工具
│   └── json_monitor.py           # 串口 JSON 监视器（支持实时数据流展示与退出摘要）
├── CMakeLists.txt                # 顶层构建文件（通过 MAIN_DIR 参数切换模式）
└── sdkconfig.defaults            # 默认宏定义与配置覆盖

```

---

## 📦 编译与烧录

确保本地已配置好 **ESP-IDF v6.0.1** 环境。通过在构建时传入 `-DMAIN_DIR` 参数来切换功能模块。

### 1. 编译为接收器 (Scanner)

```bash
idf.py set-target esp32s3 -DMAIN_DIR=main_rx
idf.py build
idf.py flash monitor

```

### 2. 编译为发射器 (Simulator)

```bash
idf.py set-target esp32s3 -DMAIN_DIR=main_tx
idf.py build
idf.py flash monitor

```

---

## 📊 数据输出与事件系统

系统产生的所有事件都会转换为标准的单行 JSON 对象进行分流输出。

### 端口映射表

* **数据端口（UART1 / GPIO17 TX @ 115200 baud）**：仅输出纯净的无人机业务数据。
* **调试端口（USB CDC / stdout）**：输出日志、错误、告警及诊断信息。*(注：数据端口内容默认也会镜像输出至 USB CDC 方便调试。)*

### 事件（`evt`）触发说明

| 事件类型 (`evt`) | 输出端口 | 触发时机与说明 |
| --- | --- | --- |
| `startup` | 调试端口 | 系统启动时，输出版本横幅与初始化核心参数。 |
| `status` | 数据端口 | 每 60 秒定期输出，报告当前抓包速率、活跃 UAV 计数等系统状态。 |
| `uav_discovery` | 数据端口 | 首次捕获到某全新 MAC 地址的无人机 RID 信号。 |
| `uav_update` | 数据端口 | 无人机信号解码成功后，高频输出其包含位置、速度、状态等完整字段的数据。 |
| `uav_status` | 数据端口 | 每 60 秒定期输出，包含各无人机当前的活跃时长（`age_ms`）。 |
| `uav_timeout` | 数据端口 | 某无人机连续 5 分钟未更新信号，将其从追踪表清除并发出该通知。 |
| `warning` / `error` | 调试端口 | 追踪表满、硬件初始化失败、任务创建异常等系统级告警。 |
| `decode_fail` | 调试端口 | 诊断专用。每累计失败 32 次输出一次，包含原始错误字节流供分析。 |

### `uav_update` 数据字段示例

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
    "timestamp": 1234.5
  },
  "self_id": {
    "type": "text",
    "desc": "Drone #1"
  },
  "operator_id": {
    "type": 1,
    "id": "OP12345"
  }
}

```

> 💡 **提示**：未获取到的字段在 JSON 中将统一呈现为 `null`；所有枚举数据均采用规范的 `snake_case` 字符串呈现，对下游解析极度友好。

---

## 🖥️ 上位机监视器 (`json_monitor.py`)

位于 `tools/` 目录下的 Python 脚本可以实时捕获串口 JSON 流并分类呈现。

### 快速开始

```bash
# 安装依赖
pip install pyserial

# 自动检测串口，默认仅显示 UAV 核心动态数据
python3 tools/json_monitor.py

# 指定物理串口并以“静态消息去重模式”运行（适合记录空中无人机注册身份）
python3 tools/json_monitor.py -p /dev/tty.usbserial-A5069RR4 -m static

```

### 监视模式 (`-m` / `--mode`)

* `data`（默认）：仅关注无人机动态数据（`discovery`, `update`, `status`, `timeout`）。
* `debug`：仅监视系统运行日志、告警及解码失败诊断。
* `static`：**静态身份提取模式**。过滤位置等高频动态字段，每个 MAC 地址的静态信息（Basic ID, Self ID, Operator ID 等）仅在其首次出现或改变时输出一次，界面极度清爽。
* `all`：不加过滤器，打印硬件串口抛出的所有 JSON 原始事件。

> 💡 **会话摘要**：当你在终端按下 `Ctrl+C` 退出脚本时，监视器会自动输出一份详尽的图形化**会话摘要统计**（可通过 `--no-summary` 参数关闭）。摘要包含：系统运行时长、累计捕获设备总数、解码丢包率、以及每个活跃 MAC 地址最后留在空中的状态快照。

---

## 🌐 Web OTA 更新功能

本项目支持通过Web界面进行固件升级，无需串口连接。OTA功能具有以下特性：

### 核心功能
- **Web界面升级**：通过浏览器上传固件文件完成升级
- **进度跟踪**：实时显示升级进度
- **固件校验**：支持MD5校验确保固件完整性
- **安全认证**：基础认证保护OTA接口
- **错误处理**：完善的错误提示和恢复机制

### 使用方法

1. 启动设备后，它会自动创建一个WiFi热点
2. 连接到该热点
3. 在浏览器中访问 `http://192.168.4.1` 
4. 输入用户名(admin)和密码(esp32ota)进行认证
5. 选择要上传的固件文件(.bin格式)
6. 点击上传按钮开始升级

### 技术细节

- **默认端口**：80
- **认证信息**：用户名 `admin`，密码 `esp32ota`
- **支持格式**：.bin固件文件
- **安全特性**：MD5校验、基础认证、固件验证

### 固件校验

为了确保固件安全性，可以启用MD5校验功能：

1. 使用工具脚本生成固件MD5：
   ```bash
   python3 tools/ota_util.py firmware.bin
   ```

2. 在代码中设置期望的MD5值：
   ```c
   uint8_t expected_md5[16] = { /* MD5 bytes */ };
   crid_ota_set_expected_md5(expected_md5);
   ```

该功能使设备维护更加便捷，特别适用于远程部署的场景。

---
## 📋 GB 46750-2025 协议支持

GB 46750-2025《民用无人驾驶航空器系统安全要求》是中国国家标准，专门针对民用无人机系统的远程识别功能制定了技术规范。本项目完全兼容该标准的数据格式解析。

### 协议特点

GB 46750-2025 协议格式如下：
- 格式：`[MessageCounter(1)] [0xFF data_type(1)] [版本号(1)] [数据内容长度(1)] [数据标识(3)] [数据内容(变长)]`
- 版本号：高3位 = 主版本(0x1)，低5位 = 子版本号
- 数据标识：固定3字节，用于标识数据内容类型
- 数据内容：变长数据，根据标识字节解析不同类型的无人机信息

### 支持的数据类型

| 标识字节 | 数据类型 | 说明 |
|---------|----------|------|
| 0x07 | 唯一标识符 | 20字节的无人机唯一ID |
| 0x06 | 实名登记号 | 8字节的实名登记号码 |
| 0x05 | 运营分类 | 无人机运营分类信息 |
| 0x04 | 无人机类型 | 无人机机型分类 |
| 0x03 | RCS定位类型 | 无线电通信系统定位类型 |
| 0x02 | RCS位置 | 8字节经纬度坐标 (经度|纬度) |
| 0x0F | 无人机位置 | 8字节经纬度坐标 (经度|纬度) |
| 0x0E | 航向角 | 无人机当前航向角度 |
| 其他 | 更多数据项 | 支持扩展的无人机运行参数 |

### 重要修正

在GB 46750-2025协议中，经纬度顺序为(经度|纬度)，而非传统(纬度|经度)顺序，项目代码中已正确处理此顺序差异。

---
