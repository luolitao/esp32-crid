# ESP32 C-RID OTA 功能详细说明

## 概述

ESP32 C-RID 项目提供了增强版的OTA（Over-The-Air）固件更新功能，允许用户通过Web界面远程更新设备固件，而无需物理连接串口。该功能包含了安全认证、进度跟踪、固件校验等多项增强特性。

## 功能特性

### 1. Web界面升级
- 提供友好的Web界面用于固件上传
- 实时显示升级进度和状态
- 支持常见的.bin固件文件格式

### 2. 安全认证
- 基础认证保护OTA接口
- 默认用户名：`admin`
- 默认密码：`esp32ota`

### 3. 进度跟踪
- 实时显示固件上传进度
- 提供JSON API接口获取进度信息
- 清晰的状态指示（空闲、开始、进行中、完成、失败）

### 4. 固件校验
- 支持MD5校验确保固件完整性
- 自动验证固件签名
- 防止损坏或恶意固件更新

### 5. 错误处理
- 完善的错误提示机制
- 自动回滚功能
- 详细的错误日志记录

## 使用方法

### 1. 启动OTA服务
OTA服务会在设备启动时自动初始化：

```c
// 在主程序中初始化OTA服务
esp_err_t err = crid_ota_web_init();
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize OTA web server: %s", esp_err_to_name(err));
    return;
}
```

### 2. 访问Web界面
1. 连接到设备创建的WiFi热点
2. 在浏览器中访问 `http://192.168.4.1`
3. 输入认证信息（默认：admin/esp32ota）
4. 选择固件文件并点击上传

### 3. 监控升级状态
可以通过API接口监控升级状态：

```c
// 检查OTA是否正在进行
if (crid_ota_is_in_progress()) {
    int progress = crid_ota_get_progress();
    printf("OTA进度: %d%%\n", progress);
}

// 获取当前状态
ota_state_t state = crid_ota_get_state();
switch (state) {
    case OTA_IDLE:
        printf("OTA空闲\n");
        break;
    case OTA_IN_PROGRESS:
        printf("OTA进行中\n");
        break;
    // ... 其他状态
}
```

## API参考

### 函数列表

#### `esp_err_t crid_ota_web_init(void)`
初始化OTA Web服务

**返回值**：
- `ESP_OK`：成功
- 其他：失败

#### `void crid_ota_web_deinit(void)`
反初始化OTA Web服务

#### `bool crid_ota_is_in_progress(void)`
检查OTA是否正在进行

**返回值**：
- `true`：正在进行
- `false`：未进行

#### `int crid_ota_get_progress(void)`
获取OTA更新进度百分比

**返回值**：
- 进度百分比 (0-100)

#### `ota_state_t crid_ota_get_state(void)`
获取当前OTA状态

**返回值**：
- OTA状态枚举值

#### `const char* crid_ota_get_last_error(void)`
获取最后一次错误信息

**返回值**：
- 错误信息字符串

#### `void crid_ota_set_expected_md5(const uint8_t *md5)`
设置预期的MD5校验值用于固件验证

**参数**：
- `md5`：16字节的MD5哈希值，如果为NULL则禁用校验

## 固件校验

为了提高安全性，可以启用固件校验功能：

### 1. 生成MD5校验值
使用提供的工具脚本生成固件的MD5校验值：

```bash
python3 tools/ota_util.py firmware.bin
```

### 2. 在代码中设置校验值
```c
// 示例MD5值（实际使用时应替换为真实值）
static const uint8_t firmware_md5[16] = {
    0xd4, 0x1d, 0x8c, 0xd9, 0x8f, 0x00, 0xb2, 0x04,
    0xe9, 0x80, 0x09, 0x98, 0xec, 0xf8, 0x42, 0x7e
};

// 启用MD5校验
crid_ota_set_expected_md5(firmware_md5);
```

## 安全考虑

### 1. 认证信息
建议在生产环境中修改默认的认证信息：

```c
// 在crid_ota_web.c中修改
#define OTA_USERNAME "your_username"
#define OTA_PASSWORD "your_secure_password"
```

### 2. 网络安全
- 建议在受信任的网络环境中使用OTA功能
- 可以结合HTTPS或其他加密方案进一步增强安全性

### 3. 固件验证
- 强烈建议启用固件校验功能
- 定期更新固件以修复安全漏洞

## 故障排除

### 常见问题

1. **无法访问Web界面**
   - 确认设备已正确启动WiFi热点
   - 检查网络连接是否正常
   - 确认设备IP地址（通常是192.168.4.1）

2. **认证失败**
   - 确认输入的用户名和密码正确
   - 检查是否有大小写错误

3. **上传失败**
   - 确认固件文件格式正确（.bin）
   - 检查设备存储空间是否充足
   - 查看错误日志获取详细信息

### 日志查看
通过串口监视器查看设备日志可以获得更多调试信息：

```bash
idf.py monitor
```

## 示例代码

请参考 `examples/ota_usage_example.c` 文件获取完整的使用示例。