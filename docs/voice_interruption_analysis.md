# ESP-BOX 语音对话实时打断实现分析

## 一、ESP-BOX 实时打断实现机制

### 1.1 核心机制

ESP-BOX 的实时打断主要通过以下机制实现：

#### 1.1.1 实时监听模式（kListeningModeRealtime）

```cpp
// main/application.cc:1076-1084
case kDeviceStateSpeaking:
    display->SetStatus(Lang::Strings::SPEAKING);
    
    if (listening_mode_ != kListeningModeRealtime) {
        audio_processor_->Stop();  // 非实时模式：停止音频处理
        // Only AFE wake word can be detected in speaking mode
        #if CONFIG_USE_AFE_WAKE_WORD
            wake_word_->StartDetection();
        #else
            wake_word_->StopDetection();
        #endif
    }
    // 实时模式：audio_processor_ 继续运行，VAD持续检测
    ResetDecoder();
    break;
```

**关键点**：
- 当 `listening_mode_ == kListeningModeRealtime` 时，在 `Speaking` 状态下 `audio_processor_` **不会停止**
- 这意味着 VAD（语音活动检测）会持续工作，可以检测用户语音

#### 1.1.2 唤醒词检测打断（主要方式）

```cpp
// main/application.cc:724-725
} else if (device_state_ == kDeviceStateSpeaking) {
    AbortSpeaking(kAbortReasonWakeWordDetected);
}
```

**ESP-BOX原始实现**：在 `Speaking` 状态下，主要通过**唤醒词检测**来触发打断。

#### 1.1.3 VAD 状态检测（原始实现不支持）

```cpp
// main/audio_processing/afe_audio_processor.cc (ESP-BOX原始实现)
#ifdef CONFIG_USE_DEVICE_AEC
    afe_config->aec_init = true;
    afe_config->vad_init = false;  // ⚠️ AEC模式下VAD被禁用
#endif

void AfeAudioProcessor::EnableDeviceAec(bool enable) {
    if (enable) {
        afe_iface_->disable_vad(afe_data_);  // ⚠️ 禁用VAD
        afe_iface_->enable_aec(afe_data_);
    }
}
```

**ESP-BOX原始限制**：
- ⚠️ **AEC模式下VAD被禁用**：启用AEC时，VAD初始化被设置为`false`，且`EnableDeviceAec()`会主动禁用VAD
- ⚠️ **VAD回调不处理Speaking状态**：原始实现中，VAD回调只在`kDeviceStateListening`状态下更新`voice_detected_`标志
- ✅ **结论**：ESP-BOX原始实现**不支持VAD实时打断**，只能通过唤醒词打断

### 1.2 实时模式启用条件

```cpp
// main/application.cc:340, 723
SetListeningMode(aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime);
```

**关键要求**：实时模式需要 **AEC（回声消除）支持**，以避免设备自己的 TTS 声音被误识别为用户语音。

### 1.3 打断流程

```
用户说话
    ↓
VAD检测到语音（或唤醒词检测）
    ↓
AbortSpeaking() 被调用
    ↓
设置 aborted_ = true
    ↓
发送 abort 消息到服务器
    ↓
清空音频解码队列
    ↓
切换到 Listening 状态
```

## 二、MOJI 板子实现实时打断的可行性分析

### 2.1 硬件支持情况

#### ✅ 已具备的条件

1. **音频编解码器**：MOJI 板子使用 `BoxAudioCodec`，与 ESP-BOX 相同
   ```cpp
   // movecall_moji_esp32s3.cc:1080-1093
   static BoxAudioCodec audio_codec(...)
   ```

2. **音频处理器**：可以使用相同的 `AfeAudioProcessor` 或 `NoAudioProcessor`

3. **VAD 支持**：AFE 音频处理器内置 VAD 功能

#### ⚠️ 潜在限制

1. **AEC 配置**：
   ```cpp
   // config.h:12
   #define AUDIO_INPUT_REFERENCE    false
   ```
   - `AUDIO_INPUT_REFERENCE` 设置为 `false`，可能影响 AEC 功能
   - 需要确认是否启用了设备端或服务器端 AEC

2. **配置检查**：需要确认编译配置中是否启用了：
   - `CONFIG_USE_DEVICE_AEC`（设备端 AEC）
   - `CONFIG_USE_SERVER_AEC`（服务器端 AEC）
   - `CONFIG_USE_AFE_WAKE_WORD`（AFE 唤醒词）

### 2.2 实现方案

#### 方案一：基于唤醒词的打断（推荐，简单可靠）

**优点**：
- 实现简单，不需要修改 VAD 回调逻辑
- 不依赖 AEC，误触发风险低
- 与当前代码兼容性好

**实现步骤**：
1. 确保在 `Speaking` 状态下启用唤醒词检测
2. 检测到唤醒词时调用 `AbortSpeaking()`

**代码修改**：
```cpp
// 在 SetDeviceState 的 kDeviceStateSpeaking 分支中
case kDeviceStateSpeaking:
    display->SetStatus(Lang::Strings::SPEAKING);
    
    // 启用唤醒词检测以支持打断
    #if CONFIG_USE_AFE_WAKE_WORD
        wake_word_->StartDetection();
    #endif
    
    ResetDecoder();
    break;
```

#### 方案二：基于 VAD 的实时打断（需要 AEC 支持）

**优点**：
- 更自然的交互体验
- 不需要说出唤醒词即可打断

**缺点**：
- 需要 AEC 支持，否则会误触发
- 实现复杂度较高

**实现步骤**：
1. 启用实时监听模式（需要 AEC）
2. 修改 VAD 回调，在 `Speaking` 状态下检测到语音时触发打断
3. 确保 AEC 正常工作，避免误触发

**代码修改**：
```cpp
// 1. 修改 VAD 回调处理
audio_processor_->OnVadStateChange([this](bool speaking) {
    if (device_state_ == kDeviceStateListening) {
        Schedule([this, speaking]() {
            voice_detected_ = speaking;
            auto led = Board::GetInstance().GetLed();
            led->OnStateChanged();
        });
    } else if (device_state_ == kDeviceStateSpeaking && speaking) {
        // 在Speaking状态下检测到语音，触发打断
        if (listening_mode_ == kListeningModeRealtime) {
            Schedule([this]() {
                AbortSpeaking(kAbortReasonNone);
            });
        }
    }
});

// 2. 在 SetDeviceState 中启用实时模式
case kDeviceStateSpeaking:
    if (listening_mode_ == kListeningModeRealtime) {
        // 保持 audio_processor_ 运行，VAD 持续检测
        // audio_processor_ 不停止
    } else {
        audio_processor_->Stop();
    }
    ResetDecoder();
    break;
```

### 2.3 推荐实现路径

**阶段一：快速实现（基于唤醒词）**
1. 在 `Speaking` 状态下启用唤醒词检测
2. 检测到唤醒词时调用 `AbortSpeaking()`
3. 测试验证打断功能

**阶段二：优化体验（基于 VAD，可选）**
1. 确认 AEC 配置和功能
2. 启用实时监听模式
3. 修改 VAD 回调支持 Speaking 状态下的打断
4. 充分测试避免误触发

## 三、关键代码位置

### 3.1 打断相关函数

- `Application::AbortSpeaking()` - `main/application.cc:990`
- `Protocol::SendAbortSpeaking()` - `main/protocols/protocol.cc:34`
- `Application::SetListeningMode()` - `main/application.cc:996`

### 3.2 状态管理

- `Application::SetDeviceState()` - `main/application.cc:1001`
- VAD 回调注册 - `main/application.cc:676`
- 唤醒词检测回调 - `main/application.cc:687`

### 3.3 音频处理

- `AfeAudioProcessor::OnVadStateChange()` - `main/audio/processors/afe_audio_processor.cc:117`
- VAD 状态检测 - `main/audio/processors/afe_audio_processor.cc:142-150`

## 四、实现状态

### ✅ 已完成的实现（方案二：基于VAD的实时打断）

#### 4.1 配置修改

1. **Kconfig 配置** (`main/Kconfig.projbuild`)
   - ✅ 添加 `BOARD_TYPE_MOVECALL_MOJI_ESP32S3` 到 `USE_DEVICE_AEC` 的依赖列表

2. **板子配置** (`main/boards/movecall-moji-esp32s3/config.json`)
   - ✅ 添加 `CONFIG_USE_DEVICE_AEC=y` 到 `sdkconfig_append`

3. **音频配置** (`main/boards/movecall-moji-esp32s3/config.h`)
   - ✅ 将 `AUDIO_INPUT_REFERENCE` 从 `false` 改为 `true`，启用AEC参考输入

#### 4.2 代码修改

1. **VAD回调增强** (`main/application.cc:676-693`)
   - ✅ 在 `Speaking` 状态下检测到语音时触发打断
   - ✅ 仅在 `kListeningModeRealtime` 模式下触发，避免误触发
   - ✅ 使用 `AbortSpeaking()` 发送打断信号到服务器

#### 4.3 工作流程

```
用户说话（在TTS播放期间）
    ↓
AEC消除TTS回声
    ↓
VAD检测到用户语音
    ↓
VAD回调触发（Speaking状态 + Realtime模式）
    ↓
AbortSpeaking() 被调用
    ↓
设置 aborted_ = true
    ↓
发送 abort 消息到服务器
    ↓
清空音频解码队列
    ↓
切换到 Listening 状态
```

### 4.4 关键特性

- ✅ **实时检测**：在 `Speaking` 状态下，`audio_processor_` 继续运行（实时模式）
- ✅ **AEC支持**：启用设备端AEC，避免TTS声音被误识别
- ✅ **VAD打断**：检测到用户语音时自动打断TTS播放
- ✅ **模式检查**：仅在 `kListeningModeRealtime` 模式下启用，确保安全

### 4.5 使用说明

1. **编译配置**：确保在menuconfig中启用了 `CONFIG_USE_DEVICE_AEC`
2. **自动启用**：当AEC模式开启时，系统会自动使用 `kListeningModeRealtime` 模式
3. **测试建议**：
   - 在TTS播放期间说话，验证是否能正确打断
   - 注意观察是否误触发（AEC应能有效消除TTS回声）
   - 测试不同音量下的打断效果

## 五、总结

### ESP-BOX 实时打断机制（原始实现）
- ✅ 使用 `kListeningModeRealtime` 模式
- ✅ 在 `Speaking` 状态下保持 `audio_processor_` 运行（用于AEC）
- ❌ **不支持VAD实时打断**：AEC模式下VAD被禁用
- ✅ **仅支持唤醒词打断**：在Speaking状态下启用唤醒词检测

### MOJI 板子实现状态
- ✅ **硬件支持**：具备实现实时打断的硬件条件
- ✅ **方案二（VAD打断）**：**已完成实现**
- ✅ **AEC配置**：已启用设备端AEC支持
- ✅ **VAD回调**：已增强支持Speaking状态下的打断

### 实现要点
1. ✅ **AEC配置**：启用设备端AEC，避免TTS回声干扰
2. ✅ **实时模式**：在Speaking状态下保持audio_processor运行
3. ✅ **VAD检测**：在Speaking状态下检测用户语音并触发打断
4. ✅ **安全检查**：仅在实时模式下启用，避免误触发


