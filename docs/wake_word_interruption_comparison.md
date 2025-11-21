# Moji项目与ESP-BOX-3唤醒词打断实现对比

## 一、ESP-BOX-3原始实现（根据文档）

### 1.1 Speaking状态处理

```cpp
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
- ✅ **非实时模式**：停止audio_processor，启动唤醒词检测
- ⚠️ **实时模式**：文档中只说明audio_processor继续运行，**未明确说明是否启动唤醒词检测**

### 1.2 唤醒词回调处理

```cpp
} else if (device_state_ == kDeviceStateSpeaking) {
    AbortSpeaking(kAbortReasonWakeWordDetected);
}
```

**关键点**：
- ✅ 在Speaking状态下检测到唤醒词时调用`AbortSpeaking()`

## 二、Moji项目当前实现

### 2.1 Speaking状态处理

```cpp
case kDeviceStateSpeaking:
    display->SetStatus(Lang::Strings::SPEAKING);
    
    if (listening_mode_ != kListeningModeRealtime) {
        audio_processor_->Stop();
        // Only AFE wake word can be detected in speaking mode (same as esp-box-3)
        #if CONFIG_USE_AFE_WAKE_WORD
            wake_word_->StartDetection();
        #else
            wake_word_->StopDetection();
        #endif
    } else {
        // In realtime mode, audio_processor_ continues running for AEC
        // Note: VAD interruption is not supported (same as esp-box-3)
        // Only wake word interruption is supported during Speaking state
        ESP_LOGI(TAG, "Realtime mode: keeping audio processor running for AEC");
        if (!audio_processor_->IsRunning()) {
            audio_processor_->Start();
        }
        // Only AFE wake word can be detected in speaking mode (same as esp-box-3)
        #if CONFIG_USE_AFE_WAKE_WORD
            wake_word_->StartDetection();  // ⚠️ 实时模式下也启动唤醒词检测
        #else
            wake_word_->StopDetection();
        #endif
    }
    ResetDecoder();
    break;
```

**关键点**：
- ✅ **非实时模式**：停止audio_processor，启动唤醒词检测（与esp-box-3一致）
- ⚠️ **实时模式**：保持audio_processor运行，**同时启动唤醒词检测**（与esp-box-3文档不一致）

### 2.2 唤醒词回调处理

```cpp
} else if (device_state_ == kDeviceStateSpeaking) {
    // Wake word interruption during Speaking state (same as esp-box-3)
    ESP_LOGI(TAG, "Wake word detected during Speaking state, aborting: %s", wake_word.c_str());
    AbortSpeaking(kAbortReasonWakeWordDetected);
}
```

**关键点**：
- ✅ 在Speaking状态下检测到唤醒词时调用`AbortSpeaking()`（与esp-box-3一致）

### 2.3 音频输入处理

```cpp
// In Speaking state, prioritize wake word detection for interruption (same as esp-box-3)
// Wake word detection must receive audio even when audio_processor is running
if (wake_word_->IsDetectionRunning()) { 
    std::vector<int16_t> data;
    int samples = wake_word_->GetFeedSize();
    if (samples > 0) {
        if (ReadAudio(data, 16000, samples)) {
            wake_word_->Feed(data);
            // Also feed to audio_processor if it's running (for AEC in realtime mode)
            if (audio_processor_->IsRunning()) {
                audio_processor_->Feed(data);
            }
            return;
        }
    }
}
```

**关键点**：
- ✅ 唤醒词检测优先接收音频
- ✅ 如果audio_processor在运行，也同时feed给它（用于AEC）

## 三、差异分析

### 3.1 主要差异

| 项目 | ESP-BOX-3（文档） | Moji项目 | 差异说明 |
|------|------------------|---------|---------|
| **非实时模式** | 停止audio_processor，启动唤醒词检测 | ✅ 一致 | 无差异 |
| **实时模式** | 文档未明确说明是否启动唤醒词检测 | ⚠️ 启动唤醒词检测 | **可能存在差异** |
| **唤醒词回调** | `AbortSpeaking(kAbortReasonWakeWordDetected)` | ✅ 一致 | 无差异 |
| **音频输入优先级** | 文档未说明 | ⚠️ 唤醒词检测优先 | **可能存在差异** |

### 3.2 差异原因分析

1. **实时模式下唤醒词检测**：
   - ESP-BOX-3文档中未明确说明实时模式下是否启动唤醒词检测
   - Moji项目在实时模式下也启动唤醒词检测，这是合理的，因为：
     - 实时模式下需要audio_processor运行（用于AEC）
     - 同时需要唤醒词检测来支持打断
     - 如果实时模式下不启动唤醒词检测，就无法通过唤醒词打断

2. **音频输入优先级**：
   - ESP-BOX-3文档中未说明音频输入的优先级
   - Moji项目让唤醒词检测优先接收音频，这是必要的，因为：
     - 如果audio_processor优先接收音频，唤醒词检测可能无法正常工作
     - 唤醒词检测需要持续接收音频才能检测到唤醒词

## 四、结论

### 4.1 一致性检查

- ✅ **非实时模式**：完全一致
- ✅ **唤醒词回调处理**：完全一致
- ⚠️ **实时模式**：可能存在差异（esp-box-3文档未明确说明）
- ⚠️ **音频输入优先级**：可能存在差异（esp-box-3文档未明确说明）

### 4.2 建议

1. **如果esp-box-3在实时模式下也启动唤醒词检测**：
   - ✅ Moji项目的实现与esp-box-3一致
   - ✅ 当前实现是正确的

2. **如果esp-box-3在实时模式下不启动唤醒词检测**：
   - ⚠️ Moji项目的实现与esp-box-3不一致
   - ⚠️ 需要移除实时模式下的唤醒词检测启动代码
   - ⚠️ 但这样会导致实时模式下无法通过唤醒词打断

### 4.3 推荐方案

**建议保持当前实现**，原因：
1. 实时模式下启动唤醒词检测是合理的，因为需要支持唤醒词打断
2. 如果esp-box-3在实时模式下不启动唤醒词检测，那么它可能不支持实时模式下的唤醒词打断
3. Moji项目的实现更加完善，支持了实时模式下的唤醒词打断

## 五、需要确认的问题

1. **ESP-BOX-3在实时模式下是否启动唤醒词检测？**
   - 需要查看esp-box-3的实际代码来确认
   - 或者通过测试esp-box-3来验证

2. **ESP-BOX-3的音频输入优先级是什么？**
   - 需要查看esp-box-3的实际代码来确认
   - 或者通过测试esp-box-3来验证

















