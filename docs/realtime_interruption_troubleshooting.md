# 实时打断功能故障排查指南

## 问题现象

从日志看到：
```
I (21230) Application: Entering Speaking state, listening_mode=0, aec_mode=0, audio_processor_running=1
I (21230) Application: Not in realtime mode, stopping audio processor
```

这表明：
- `aec_mode=0` 表示 AEC 未启用 (`kAecOff`)
- `listening_mode=0` 表示不是实时模式 (`kListeningModeAutoStop`)

## 根本原因

`CONFIG_USE_DEVICE_AEC` 配置没有在编译时生效，导致：
1. AEC 模式被设置为 `kAecOff`
2. 监听模式被设置为 `kListeningModeAutoStop` 而不是 `kListeningModeRealtime`
3. 在 Speaking 状态下，audio_processor 被停止，无法进行 VAD 检测

## 解决方案

### 步骤 1: 检查编译配置

运行以下命令检查配置是否生效：

```bash
idf.py menuconfig
```

导航到：
- `Component config` → `Audio` → `Enable Device-Side AEC`

确保该选项被**启用**（显示 `[*]`）。

### 步骤 2: 清理并重新编译

```bash
idf.py fullclean
idf.py build
```

### 步骤 3: 验证配置

重新编译后，查看启动日志，应该看到：

```
I (xxxx) Application: CONFIG_USE_DEVICE_AEC is enabled, AEC mode set to kAecOnDeviceSide
I (xxxx) Application: Enabling device AEC in audio processor
```

如果看到：
```
W (xxxx) Application: No AEC config enabled, AEC mode set to kAecOff - realtime interruption will not work!
```

说明配置仍未生效，需要检查：
1. `main/boards/movecall-moji-esp32s3/config.json` 中是否有 `CONFIG_USE_DEVICE_AEC=y`
2. `main/Kconfig.projbuild` 中 `BOARD_TYPE_MOVECALL_MOJI_ESP32S3` 是否在 `USE_DEVICE_AEC` 的依赖列表中

### 步骤 4: 验证运行时状态

在 TTS 播放期间，查看日志应该显示：

```
I (xxxx) Application: Entering Speaking state, listening_mode=2, aec_mode=1, audio_processor_running=1
I (xxxx) Application: Realtime mode enabled, keeping audio processor running for VAD interruption
```

其中：
- `listening_mode=2` 表示 `kListeningModeRealtime` ✅
- `aec_mode=1` 表示 `kAecOnDeviceSide` ✅
- `audio_processor_running=1` 表示 audio_processor 正在运行 ✅

### 步骤 5: 测试打断功能

在 TTS 播放期间说话，应该看到：

```
D (xxxx) Application: VAD detected speech in Speaking state, listening_mode=2, aec_mode=1
I (xxxx) Application: VAD detected speech during TTS, aborting speaking
I (xxxx) Application: Abort speaking
```

## 配置文件检查清单

### ✅ config.json
```json
{
    "target": "esp32s3",
    "builds": [
        {
            "name": "movecall-moji-esp32s3",
            "sdkconfig_append": [
                "CONFIG_USE_DEVICE_AEC=y"
            ]
        }
    ]
}
```

### ✅ config.h
```c
#define AUDIO_INPUT_REFERENCE    true  // 必须为 true
```

### ✅ Kconfig.projbuild
```kconfig
config USE_DEVICE_AEC
    depends on USE_AUDIO_PROCESSOR && (BOARD_TYPE_ESP_BOX_3 || ... || BOARD_TYPE_MOVECALL_MOJI_ESP32S3)
```

## 常见问题

### Q: 为什么 config.json 中已经添加了配置，但还是不生效？

A: `config.json` 中的 `sdkconfig_append` 只在首次配置时生效。如果之前已经运行过 `idf.py menuconfig`，需要：
1. 运行 `idf.py fullclean`
2. 重新运行 `idf.py menuconfig` 并手动启用选项
3. 或者删除 `build` 目录和 `sdkconfig` 文件后重新编译

### Q: 如何确认 CONFIG_USE_DEVICE_AEC 是否在编译时被定义？

A: 查看编译后的 `build/config/sdkconfig.h` 文件，搜索 `CONFIG_USE_DEVICE_AEC`，应该看到：
```c
#define CONFIG_USE_DEVICE_AEC 1
```

### Q: 硬件支持 AEC，但配置未启用，能否强制启用？

A: 可以，但需要修改代码。在 `main/application.cc` 的构造函数中，可以临时强制设置：
```cpp
// 临时强制启用（仅用于调试）
aec_mode_ = kAecOnDeviceSide;
```
**注意**：这只是临时方案，正确的做法是确保编译配置生效。

## 调试日志

添加了以下调试日志帮助诊断：

1. **启动时**：
   - `CONFIG_USE_DEVICE_AEC is enabled` - 配置已启用
   - `No AEC config enabled` - 配置未启用（警告）

2. **初始化时**：
   - `Enabling device AEC in audio processor` - 正在启用 AEC
   - `Device AEC not enabled (aec_mode_=0)` - AEC 未启用

3. **进入 Speaking 状态时**：
   - `Entering Speaking state, listening_mode=X, aec_mode=Y, audio_processor_running=Z`
   - `Realtime mode enabled` - 实时模式已启用
   - `Not in realtime mode` - 非实时模式

4. **VAD 检测时**：
   - `VAD detected speech in Speaking state` - VAD 检测到语音
   - `VAD detected speech but not in realtime mode` - 非实时模式，无法打断

















