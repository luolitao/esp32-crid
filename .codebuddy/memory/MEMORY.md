# ESP32 RemoteID Scanner 项目记忆

## 项目概述
基于 ESP-IDF 6.0.1，主要部署在 ESP32-S3，兼容 ESP32-C3、ESP32-C5 等模组，用于接收和解析无人机 RemoteID 广播信号。

## 项目来源
本项目由两个独立项目合并而来：
1. **RemoteID_Scanner** (`/Users/luolitao/remoteid/esp32/RemoteID_Scanner/`)：无人机 RemoteID 信号接收/扫描器
2. **crid-sim** (`/Users/luolitao/remoteid/esp32/crid-sim/`)：中国 C-RID (GB42590-2023) 模拟发射器

## 项目结构
- 通过 `MAIN_DIR` CMake 变量选择构建目标：
  - `main_rx`：接收解析程序 (C-RID Scanner)，源于 RemoteID_Scanner
  - `main_tx`：模拟发送程序 (C-RID Transmitter)，源于 crid-sim
- `components/opendroneid/`：OpenDroneID 官方库组件
- `archive/`：历史版本和独立工具归档（不参与编译）
  - `RemoteID_Scanner.txt`：AI 对话历史
  - `ridscanner-usb.c` + `.py`：USB CDC JSON 输出版本
  - `china_crid_receiver_fixed.py`：Python Scapy 版 C-RID 探测器

## 支持的协议标准
- ASTM F3411-22 (OpenDroneID)，OUI: FF:FF:5F
- ASD-STAN prEN 4709-002
- 中国 GB 42590-2023 (C-RID)，OUI: FA:0B:BC，Vendor Type: 0x0D

## 核心架构 (main_rx/ — 模块化)

| 模块 | 文件 | 职责 |
|------|------|------|
| 公共类型 | `crid_rx_types.h` | OUI 定义、配置常量、结构体 |
| Sniffer | `crid_sniffer.c/h` | Wi-Fi 初始化、ISR 回调、信道锁定 |
| 追踪器 | `crid_tracker.c/h` | 无人机追踪表（线程安全，互斥锁） |
| 解析器 | `crid_parser.c/h` | opendroneid 库解码 |
| 显示 | `crid_display.c/h` | 枚举→名称映射、摘要/详情打印 |
| 主入口 | `crid_scan_main.c` | app_main()，组装所有模块 |

### 运行时任务
- `parser_task`：从队列取数据，使用 opendroneid 库解析
- `monitor_task`：每 30 秒输出统计和追踪摘要
- `channel_hold_task`：锁定信道 6，防止漂移
- 使用 FreeRTOS 互斥锁保护无人机追踪表（最多 16 架）

## TX 模拟器 (main_tx)
- `crid-sim.c`：主入口，创建 Beacon 发送任务
- `crid_wifi.c/h`：Wi-Fi 原始帧发送
- `crid_messages.c/h`：C-RID 消息编码
- `crid_config.c/h`：配置管理
- `crid_patrol.c/h`：模拟巡游路径（越秀山附近圆形运动）

## 关键技术细节
- 中国 C-RID 使用 Wi-Fi 信道 6（2.4GHz），1Hz 广播频率
- Vendor Specific IE (ID=221) 中携带 OUI + OUI Type + OpenDroneID 数据
- 支持 Message Pack 格式和单消息格式
- ESP-IDF 6.0.1，主要目标芯片 ESP32-S3，兼容 ESP32-C3/C5

## 历史关键问题与解决
1. **MACSTR/MAC2STR 宏未定义**：ESP-IDF v5.5 中这些宏不稳定，改用自定义 `print_mac()` 函数
2. **opendroneid 库 API 变化**：不同版本的字段名不同（如 `HorAcc` vs `HorizAccuracy`），需查阅头文件确认
3. **`esp_wifi_80211_tx` 的 `en_sys_seq` 参数**：AP 模式下必须设为 `true`
4. **NAN 帧监听**：通过 Public Action Frame (Category 0x0d) + Wi-Fi Alliance OUI (50:6F:9A) + NAN Type (0x13) 识别
5. **字符串截断警告**：ID_SIZE 从 `ODID_ID_SIZE+1` 增加到 `ODID_ID_SIZE+10`
6. **Wi-Fi NULL 模式**：Scanner 使用 WIFI_MODE_NULL + Promiscuous Mode，而非 AP 或 STA

## 编译方式
```bash
# Scanner
idf.py set-target esp32s3 -DMAIN_DIR=main_rx
idf.py build

# Transmitter
idf.py set-target esp32s3 -DMAIN_DIR=main_tx
idf.py build
```
