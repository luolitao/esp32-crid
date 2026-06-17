# Tools 目录说明

本目录包含用于ESP32 C-RID项目的各种辅助工具。

## 工具列表

### 1. json_monitor.py
串口JSON监视器，用于实时捕获和分析设备输出的JSON数据。

### 2. ota_util.py
OTA固件工具，用于生成固件的MD5校验值和相关C代码。

使用方法：
```bash
# 生成固件MD5校验值
python3 ota_util.py firmware.bin

# 生成包含校验值的C代码文件
python3 ota_util.py firmware.bin -c firmware_md5.c
```

### 3. ridscanner-usb.py
USB接口的RID扫描器工具。

### 4. china_crid_receiver_fixed.py
中国标准RID接收器修复版本。

## 使用说明

确保已安装必要的Python依赖：
```bash
pip install -r pyproject.toml
```

或者单独安装：
```bash
pip install pyserial
```