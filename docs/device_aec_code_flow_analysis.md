# DEVICE AEC 代码执行流程分析

本文档详细分析配置 `CONFIG_USE_DEVICE_AEC` 后，系统会执行哪些代码逻辑。

## 一、配置阶段（编译时）

### 1.1 Kconfig 配置
- **位置**: `main/Kconfig.projbuild:395-396`
- **配置项**: `CONFIG_USE_DEVICE_AEC`
- **说明**: 在 menuconfig 中启用 "Enable Device-Side AEC" 选项

### 1.2 Board 配置
- **位置**: `main/boards/*/config.json`
- **配置示例**: `"CONFIG_USE_DEVICE_AEC=y"`
- **说明**: 在特定 board 的配置文件中启用 DEVICE AEC

---

## 二、初始化阶段（运行时）

### 2.1 Application 构造函数初始化

**位置**: `main/application.cc:56-66`

```cpp
Application::Application() {
    // ...
#if CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;  // 设置为设备端AEC模式
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
#else
    aec_mode_ = kAecOff;
#endif
    // ...
}
```

**执行逻辑**:
- 如果 `CONFIG_USE_DEVICE_AEC` 已定义，将 `aec_mode_` 设置为 `kAecOnDeviceSide`
- 这决定了后续的 AEC 处理模式

### 2.2 AfeAudioProcessor 初始化

**位置**: `main/audio_processing/afe_audio_processor.cc:13-66`

#### 2.2.1 输入格式配置
```cpp
void AfeAudioProcessor::Initialize(AudioCodec* codec) {
    codec_ = codec;
    int ref_num = codec_->input_reference() ? 1 : 0;  // 检查是否有参考通道
    
    std::string input_format;
    for (int i = 0; i < codec_->input_channels() - ref_num; i++) {
        input_format.push_back('M');  // M = 麦克风通道
    }
    for (int i = 0; i < ref_num; i++) {
        input_format.push_back('R');  // R = 参考通道（用于AEC）
    }
```

**说明**: 
- 如果 codec 支持参考通道（`input_reference()` 返回 true），会添加 'R' 通道
- 参考通道用于 AEC 的回声消除

#### 2.2.2 AFE 配置初始化
```cpp
    afe_config_t* afe_config = afe_config_init(input_format.c_str(), NULL, AFE_TYPE_VC, AFE_MODE_HIGH_PERF);
    afe_config->aec_mode = AEC_MODE_VOIP_HIGH_PERF;
    afe_config->vad_mode = VAD_MODE_0;
    afe_config->vad_min_noise_ms = 100;
```

#### 2.2.3 AEC 初始化配置
```cpp
#ifdef CONFIG_USE_DEVICE_AEC
    afe_config->aec_init = true;      // 启用AEC初始化
    afe_config->vad_init = true;       // 同时启用VAD以支持唤醒词打断
#else
    afe_config->aec_init = false;
    afe_config->vad_init = true;
#endif
```

**执行逻辑**:
- 如果 `CONFIG_USE_DEVICE_AEC` 已定义，设置 `aec_init = true`
- 同时启用 VAD 以支持唤醒词打断功能

#### 2.2.4 AFE 句柄创建
```cpp
    afe_iface_ = esp_afe_handle_from_config(afe_config);
    afe_data_ = afe_iface_->create_from_config(afe_config);
```

**说明**: 
- 根据配置创建 AFE 接口和数据对象
- 此时 AEC 模块已初始化，但可能还未启用（取决于后续调用）

#### 2.2.5 音频处理任务创建
```cpp
    xTaskCreate([](void* arg) {
        auto this_ = (AfeAudioProcessor*)arg;
        this_->AudioProcessorTask();  // 启动音频处理任务
        vTaskDelete(NULL);
    }, "audio_communication", 4096, this, 3, NULL);
```

**说明**: 创建独立的 FreeRTOS 任务来处理音频数据

### 2.3 Application::Start() 中的配置

**位置**: `main/application.cc:410-433`

```cpp
void Application::Start() {
    // ...
    opus_encoder_ = std::make_unique<OpusEncoderWrapper>(16000, 1, OPUS_FRAME_DURATION_MS);
    opus_encoder_->SetComplexity(0);
    if (aec_mode_ != kAecOff) {
        ESP_LOGI(TAG, "AEC mode: %d, setting opus encoder complexity to 0", aec_mode_);
        opus_encoder_->SetComplexity(0);  // AEC模式下降低编码复杂度
    } else {
        // ...
        opus_encoder_->SetComplexity(5);  // 非AEC模式使用更高复杂度
    }
```

**执行逻辑**:
- 如果 AEC 模式不为 `kAecOff`，将 Opus 编码器复杂度设置为 0
- 这是因为 AEC 已经在设备端处理了回声，不需要高复杂度编码

---

## 三、启用阶段（运行时动态启用）

### 3.1 SetAecMode 调用

**位置**: `main/application.cc:1280-1305`

```cpp
void Application::SetAecMode(AecMode mode) {
    aec_mode_ = mode;
    Schedule([this]() {
        // ...
        switch (aec_mode_) {
        case kAecOff:
            audio_processor_->EnableDeviceAec(false);
            // ...
            break;
        case kAecOnServerSide:
            audio_processor_->EnableDeviceAec(false);
            // ...
            break;
        case kAecOnDeviceSide:
            audio_processor_->EnableDeviceAec(true);  // 启用设备端AEC
            // ...
            break;
        }
        
        // 如果AEC模式改变，关闭音频通道
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}
```

**执行逻辑**:
- 当 `aec_mode_` 设置为 `kAecOnDeviceSide` 时，调用 `EnableDeviceAec(true)`
- 如果音频通道已打开，会先关闭通道（需要重新建立连接以应用新配置）

### 3.2 EnableDeviceAec 实现

**位置**: `main/audio_processing/afe_audio_processor.cc:149-163`

```cpp
void AfeAudioProcessor::EnableDeviceAec(bool enable) {
    if (enable) {
#if CONFIG_USE_DEVICE_AEC
        afe_iface_->enable_aec(afe_data_);      // 启用AEC
        afe_iface_->enable_vad(afe_data_);      // 同时启用VAD以支持唤醒词打断
        ESP_LOGI(TAG, "Device AEC and VAD enabled for wake word interruption support");
#else
        ESP_LOGE(TAG, "Device AEC is not supported");
#endif
    } else {
        afe_iface_->disable_aec(afe_data_);     // 禁用AEC
        afe_iface_->enable_vad(afe_data_);      // 保持VAD启用
        ESP_LOGI(TAG, "Device AEC disabled, VAD enabled");
    }
}
```

**执行逻辑**:
- **启用时**: 
  - 调用 `afe_iface_->enable_aec()` 启用 AEC 功能
  - 调用 `afe_iface_->enable_vad()` 启用 VAD（用于唤醒词打断）
- **禁用时**: 
  - 禁用 AEC，但保持 VAD 启用

---

## 四、音频处理阶段（运行时持续执行）

### 4.1 音频输入循环

**位置**: `main/application.cc:823-941`

```cpp
void Application::AudioLoop() {
    auto codec = Board::GetInstance().GetAudioCodec();
    while (true) {
        OnAudioInput();      // 处理音频输入
        if (codec->output_enabled()) {
            OnAudioOutput(); // 处理音频输出
        }
    }
}
```

**说明**: 这是一个持续运行的循环，不断处理音频输入和输出

### 4.2 OnAudioInput 音频读取

**位置**: `main/application.cc:891-941`

```cpp
void Application::OnAudioInput() {
    // ...
    
    if (audio_processor_->IsRunning()) {
        std::vector<int16_t> data;
        int samples = audio_processor_->GetFeedSize();  // 获取需要的采样数
        if (samples > 0) {
            if (ReadAudio(data, 16000, samples)) {      // 读取音频数据
                audio_processor_->Feed(data);           // 送入AFE处理
                return;
            }
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(OPUS_FRAME_DURATION_MS / 2));
}
```

**执行逻辑**:
- 检查音频处理器是否运行中
- 获取需要的采样数（`GetFeedSize()`）
- 从 codec 读取音频数据（`ReadAudio()`）
- 将数据送入 AFE 处理（`Feed()`）

### 4.3 ReadAudio 音频数据读取

**位置**: `main/application.cc:943-988`

```cpp
bool Application::ReadAudio(std::vector<int16_t>& data, int sample_rate, int samples) {
    auto codec = Board::GetInstance().GetAudioCodec();
    // ...
    
    if (codec->input_channels() == 2) {
        // 双通道：麦克风 + 参考通道
        auto mic_channel = std::vector<int16_t>(data.size() / 2);
        auto reference_channel = std::vector<int16_t>(data.size() / 2);
        for (size_t i = 0, j = 0; i < mic_channel.size(); ++i, j += 2) {
            mic_channel[i] = data[j];           // 麦克风通道
            reference_channel[i] = data[j + 1]; // 参考通道（用于AEC）
        }
        // 重采样处理...
        // 合并为交错格式：mic[0], ref[0], mic[1], ref[1], ...
    }
    // ...
}
```

**执行逻辑**:
- 如果 codec 支持双通道输入（麦克风 + 参考通道）
- 分离麦克风通道和参考通道
- 分别重采样到 16kHz
- 合并为交错格式：`[mic0, ref0, mic1, ref1, ...]`
- **参考通道用于 AEC 的回声消除**

### 4.4 Feed 音频数据送入 AFE

**位置**: `main/audio_processing/afe_audio_processor.cc:82-87`

```cpp
void AfeAudioProcessor::Feed(const std::vector<int16_t>& data) {
    if (afe_data_ == nullptr) {
        return;
    }
    afe_iface_->feed(afe_data_, data.data());  // 将音频数据送入AFE
}
```

**执行逻辑**:
- 将音频数据（包含麦克风和参考通道）送入 AFE
- AFE 内部会使用参考通道进行回声消除

### 4.5 AudioProcessorTask 音频处理任务

**位置**: `main/audio_processing/afe_audio_processor.cc:112-147`

```cpp
void AfeAudioProcessor::AudioProcessorTask() {
    auto fetch_size = afe_iface_->get_fetch_chunksize(afe_data_);
    auto feed_size = afe_iface_->get_feed_chunksize(afe_data_);
    ESP_LOGI(TAG, "Audio communication task started, feed size: %d fetch size: %d",
        feed_size, fetch_size);

    while (true) {
        xEventGroupWaitBits(event_group_, PROCESSOR_RUNNING, pdFALSE, pdTRUE, portMAX_DELAY);

        // 从AFE获取处理后的音频数据（包含AEC处理结果）
        auto res = afe_iface_->fetch_with_delay(afe_data_, portMAX_DELAY);
        
        if ((xEventGroupGetBits(event_group_) & PROCESSOR_RUNNING) == 0) {
            continue;
        }
        if (res == nullptr || res->ret_value == ESP_FAIL) {
            // 错误处理...
            continue;
        }

        // VAD状态变化处理
        if (vad_state_change_callback_) {
            if (res->vad_state == VAD_SPEECH && !is_speaking_) {
                is_speaking_ = true;
                vad_state_change_callback_(true);
            } else if (res->vad_state == VAD_SILENCE && is_speaking_) {
                is_speaking_ = false;
                vad_state_change_callback_(false);
            }
        }

        // 输出处理后的音频数据（已进行AEC处理）
        if (output_callback_) {
            output_callback_(std::vector<int16_t>(res->data, res->data + res->data_size / sizeof(int16_t)));
        }
    }
}
```

**执行逻辑**:
- 等待 `PROCESSOR_RUNNING` 事件
- 调用 `fetch_with_delay()` 从 AFE 获取处理后的音频
- **此时音频已经过 AEC 处理，回声已被消除**
- 处理 VAD 状态变化（用于唤醒词打断）
- 通过 `output_callback_` 回调输出处理后的音频

### 4.6 音频输出回调处理

**位置**: `main/application.cc:627-666`

```cpp
    audio_processor_->Initialize(codec);
    audio_processor_->OnOutput([this](std::vector<int16_t>&& data) {
        // ...
        // 音乐检测（在后台运行，避免阻塞）
        bool frame_music = false;
        if (device_state_ == kDeviceStateListening || device_state_ == kDeviceStateIdle) {
            frame_music = IsMusicLikeFrame(data);
        }

        background_task_->Schedule([this, data = std::move(data), frame_music]() mutable {
            // 更新音乐状态
            if (device_state_ == kDeviceStateListening || device_state_ == kDeviceStateIdle) {
                UpdateMusicState(frame_music, OPUS_FRAME_DURATION_MS);
            }
            
            // 编码处理后的音频（已去除回声）
            opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                AudioStreamPacket packet;
                packet.payload = std::move(opus);
                // ...
                // 发送编码后的音频包
                protocol_->SendAudio(packet);
            });
        });
    });
```

**执行逻辑**:
- 接收经过 AEC 处理的音频数据
- 进行音乐检测（可选）
- 使用 Opus 编码器编码（复杂度为 0，因为 AEC 已处理回声）
- 发送编码后的音频包到服务器

---

## 五、状态管理

### 5.1 音频处理器启动

**位置**: `main/application.cc:1019-1035` 和 `1059-1071`

```cpp
case kDeviceStateIdle:
    // ...
    if (!audio_processor_->IsRunning()) {
        audio_processor_->Start();  // 启动音频处理器
        ESP_LOGI(TAG, "Audio processor restarted");
    }
    break;

case kDeviceStateListening:
    // ...
    if (!audio_processor_->IsRunning()) {
        protocol_->SendStartListening(listening_mode_);
        // ...
        audio_processor_->Start();  // 启动音频处理器
        wake_word_->StopDetection();
    }
    break;
```

**执行逻辑**:
- 在 `kDeviceStateIdle` 和 `kDeviceStateListening` 状态下启动音频处理器
- `Start()` 会设置 `PROCESSOR_RUNNING` 事件，使 `AudioProcessorTask` 开始处理

### 5.2 音频处理器停止

**位置**: `main/application.cc:1076-1084`

```cpp
case kDeviceStateSpeaking:
    // ...
    if (listening_mode_ != kListeningModeRealtime) {
        audio_processor_->Stop();  // 停止音频处理器
        // ...
    }
    break;
```

**执行逻辑**:
- 在 `kDeviceStateSpeaking` 状态下（非实时模式）停止音频处理器
- `Stop()` 会清除 `PROCESSOR_RUNNING` 事件，并重置缓冲区

---

## 六、关键执行流程总结

### 6.1 完整流程图

```
1. 编译时配置
   └─> CONFIG_USE_DEVICE_AEC=y (config.json/Kconfig)

2. 运行时初始化
   ├─> Application::Application()
   │   └─> aec_mode_ = kAecOnDeviceSide
   │
   ├─> AfeAudioProcessor::Initialize()
   │   ├─> 配置输入格式（包含参考通道 'R'）
   │   ├─> afe_config->aec_init = true
   │   ├─> afe_config->vad_init = true
   │   ├─> 创建 AFE 句柄和数据对象
   │   └─> 创建 AudioProcessorTask
   │
   └─> Application::Start()
       └─> opus_encoder_->SetComplexity(0)

3. 动态启用（可选）
   └─> Application::SetAecMode(kAecOnDeviceSide)
       └─> AfeAudioProcessor::EnableDeviceAec(true)
           ├─> afe_iface_->enable_aec(afe_data_)
           └─> afe_iface_->enable_vad(afe_data_)

4. 音频处理循环（持续运行）
   └─> Application::AudioLoop()
       └─> Application::OnAudioInput()
           └─> Application::ReadAudio()
               ├─> 读取麦克风通道
               ├─> 读取参考通道（用于AEC）
               └─> 合并为交错格式
           └─> AfeAudioProcessor::Feed()
               └─> afe_iface_->feed(afe_data_, data)
       └─> AfeAudioProcessor::AudioProcessorTask()
           └─> afe_iface_->fetch_with_delay(afe_data_)
               ├─> AFE内部AEC处理（消除回声）
               ├─> VAD处理（检测语音）
               └─> 返回处理后的音频
           └─> Application::OnOutput() 回调
               ├─> 音乐检测（可选）
               ├─> Opus编码（复杂度0）
               └─> 发送到服务器
```

### 6.2 关键点说明

1. **参考通道**: AEC 需要参考通道（播放的音频）来消除回声
   - 参考通道来自 codec 的 `input_reference()`
   - 数据格式为交错：`[mic0, ref0, mic1, ref1, ...]`

2. **AEC 处理位置**: 在 AFE（Audio Front End）内部进行
   - `feed()` 送入原始音频（含参考通道）
   - `fetch_with_delay()` 获取处理后的音频（已消除回声）

3. **VAD 同时启用**: 为了支持唤醒词打断功能
   - AEC 和 VAD 可以同时工作
   - VAD 检测语音状态，用于实时打断

4. **编码复杂度**: AEC 模式下降低 Opus 编码复杂度
   - 因为回声已在设备端消除，不需要高复杂度编码

5. **状态管理**: 音频处理器根据设备状态启动/停止
   - Idle/Listening: 启动处理器
   - Speaking（非实时模式）: 停止处理器

---

## 七、相关配置和依赖

### 7.1 硬件要求
- Codec 必须支持参考通道输入（`input_reference()` 返回 true）
- 需要双通道输入：麦克风通道 + 参考通道

### 7.2 编译配置
- `CONFIG_USE_DEVICE_AEC=y` 必须在编译时定义
- Board 必须在 Kconfig 的 `USE_DEVICE_AEC` 依赖列表中

### 7.3 运行时配置
- `aec_mode_` 必须设置为 `kAecOnDeviceSide`
- 需要调用 `EnableDeviceAec(true)` 启用 AEC 功能

---

## 八、调试和日志

### 8.1 关键日志点

1. **初始化日志**:
   ```
   I (xxx) Application: AEC mode: 1, setting opus encoder complexity to 0
   I (xxx) AfeAudioProcessor: Audio communication task started, feed size: xxx fetch size: xxx
   ```

2. **启用日志**:
   ```
   I (xxx) AfeAudioProcessor: Device AEC and VAD enabled for wake word interruption support
   ```

3. **禁用日志**:
   ```
   I (xxx) AfeAudioProcessor: Device AEC disabled, VAD enabled
   ```

### 8.2 错误日志

如果 `CONFIG_USE_DEVICE_AEC` 未定义但尝试启用：
```
E (xxx) AfeAudioProcessor: Device AEC is not supported
```

---

## 九、总结

配置 `CONFIG_USE_DEVICE_AEC` 后，系统会：

1. **编译时**: 定义配置宏，影响条件编译
2. **初始化时**: 
   - 设置 AEC 模式
   - 配置 AFE 初始化参数
   - 创建音频处理任务
3. **启用时**: 
   - 动态启用 AEC 和 VAD 功能
4. **运行时**: 
   - 持续读取麦克风和参考通道音频
   - AFE 内部进行回声消除处理
   - 输出处理后的音频进行编码和传输

整个流程确保了设备端的回声消除功能正常工作，同时支持唤醒词打断功能。













