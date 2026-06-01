# Archive 归档目录

本目录存放项目历史版本和独立工具，**不参与 ESP-IDF 编译**。

## 文件说明

### `RemoteID_Scanner.txt`
完整的 AI 对话历史记录（181 KB），记录了从零开始构建 RemoteID Scanner 的过程，包括：
- ESP-IDF v5.5 适配问题排查
- Wi-Fi Promiscuous 模式 + NAN 帧监听
- opendroneid 库 API 使用
- MACSTR/MAC2STR 宏未定义等问题的解决方案

### `ridscanner-usb.c` + `ridscanner-usb.py`
- **`ridscanner-usb.c`**：USB CDC JSON 输出版本的 RemoteID 接收器（多信道轮询 + NAN），需要额外 cJSON 组件
- **`ridscanner-usb.py`**：配套 Python 脚本，从 USB 串口读取 JSON 输出并显示
- 当前活跃的 `main_rx/esp32_crid_scan.c` 是更精简的单信道版本，如需 USB CDC 功能可参考此实现

### `china_crid_receiver_fixed.py`
Python 版中国 C-RID (GB42590-2023) 探测器，使用 Scapy 在 PC 上嗅探 Wi-Fi 信号：
- 独立于 ESP32 项目运行
- 解析 OUI `FA:0B:BC` + Vendor Type `0x0D` 的 C-RID 信号
- 可用于快速验证或交叉检验 ESP32 接收器的结果
