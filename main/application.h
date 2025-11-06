#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <string>
#include <mutex>
#include <list>
#include <vector>
#include <condition_variable>
#include <memory>

#include <opus_encoder.h>
#include <opus_decoder.h>
#include <opus_resampler.h>

#include "protocol.h"
#include "ota.h"
#include "background_task.h"
#include "audio_processor.h"
#include "wake_word.h"
#include "audio_debugger.h"
#include "esp_doa.h"

#define SCHEDULE_EVENT (1 << 0)
#define SEND_AUDIO_EVENT (1 << 1)
#define CHECK_NEW_VERSION_DONE_EVENT (1 << 2)

enum AecMode {
    kAecOff,
    kAecOnDeviceSide,
    kAecOnServerSide,
};

enum DeviceState {
    kDeviceStateUnknown,
    kDeviceStateStarting,
    kDeviceStateWifiConfiguring,
    kDeviceStateIdle,
    kDeviceStateConnecting,
    kDeviceStateListening,
    kDeviceStateSpeaking,
    kDeviceStateUpgrading,
    kDeviceStateActivating,
    kDeviceStateAudioTesting,
    kDeviceStateFatalError
};

#define OPUS_FRAME_DURATION_MS 60
#define MAX_AUDIO_PACKETS_IN_QUEUE (2400 / OPUS_FRAME_DURATION_MS)
#define AUDIO_TESTING_MAX_DURATION_MS 10000

class Application {
public:
    static Application& GetInstance() {
        static Application instance;
        return instance;
    }
    // 删除拷贝构造函数和赋值运算符
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    void Start();
    DeviceState GetDeviceState() const { return device_state_; }
    bool IsVoiceDetected() const { return voice_detected_; }
    void Schedule(std::function<void()> callback);
    void SetDeviceState(DeviceState state);
    void Alert(const char* status, const char* message, const char* emotion = "", const std::string_view& sound = "");
    void DismissAlert();
    void AbortSpeaking(AbortReason reason);
    void ToggleChatState();
    void StartListening();
    void StopListening();
    void UpdateIotStates();
    void Reboot();
    void WakeWordInvoke(const std::string& wake_word);
    void PlaySound(const std::string_view& sound);
    void ClearAudioQueueAndDisableOutput();
    bool CanEnterSleepMode();
    void SendMcpMessage(const std::string& payload);
    void SetAecMode(AecMode mode);
    bool ReadAudio(std::vector<int16_t>& data, int sample_rate, int samples);
    AecMode GetAecMode() const { return aec_mode_; }
    BackgroundTask* GetBackgroundTask() const { return background_task_; }
    // DOA query helpers
    float GetLastDoaAngle() const { return last_doa_angle_deg_; }
    int GetLastDoaSide() const { return last_doa_side_; } // -1: left, 1: right, 0: unknown

private:
    Application();
    ~Application();

    std::unique_ptr<WakeWord> wake_word_;
    std::unique_ptr<AudioProcessor> audio_processor_;
    std::unique_ptr<AudioDebugger> audio_debugger_;
    std::mutex mutex_;
    std::list<std::function<void()>> main_tasks_;
    std::unique_ptr<Protocol> protocol_;
    EventGroupHandle_t event_group_ = nullptr;
    esp_timer_handle_t clock_timer_handle_ = nullptr;
    volatile DeviceState device_state_ = kDeviceStateUnknown;
    ListeningMode listening_mode_ = kListeningModeAutoStop;
    AecMode aec_mode_ = kAecOff;

    bool has_server_time_ = false;
    bool aborted_ = false;
    bool voice_detected_ = false;
    bool busy_decoding_audio_ = false;
    int clock_ticks_ = 0;
    TaskHandle_t check_new_version_task_handle_ = nullptr;

    // Audio encode / decode
    TaskHandle_t audio_loop_task_handle_ = nullptr;
    BackgroundTask* background_task_ = nullptr;
    std::chrono::steady_clock::time_point last_output_time_;
    std::list<AudioStreamPacket> audio_send_queue_;
    std::list<AudioStreamPacket> audio_decode_queue_;
    std::condition_variable audio_decode_cv_;
    std::list<AudioStreamPacket> audio_testing_queue_;

    // 新增：用于维护音频包的timestamp队列
    std::list<uint32_t> timestamp_queue_;
    std::mutex timestamp_mutex_;

    std::unique_ptr<OpusEncoderWrapper> opus_encoder_;
    std::unique_ptr<OpusDecoderWrapper> opus_decoder_;

    OpusResampler input_resampler_;
    OpusResampler reference_resampler_;
    OpusResampler output_resampler_;

    // DOA state
    doa_handle_t* doa_handle_ = nullptr;
    float last_doa_angle_deg_ = -1.0f;
    int last_doa_side_ = 0; // -1: left, 1: right, 0: unknown

    void MainEventLoop();
    void OnAudioInput();
    void OnAudioOutput();
    void ResetDecoder();
    void SetDecodeSampleRate(int sample_rate, int frame_duration);
    void CheckNewVersion(Ota& ota);
    void ShowActivationCode(const std::string& code, const std::string& message);
    void OnClockTimer();
    void SetListeningMode(ListeningMode mode);
    void AudioLoop();
    void EnterAudioTestingMode();
    void ExitAudioTestingMode();

    // DOA: perform once after wake word
    void PerformDoaOnceAfterWakeWord();
    bool CaptureRawInput(int target_sample_rate_hz, int frames, std::vector<int16_t>& interleaved);

    // VAD-driven relaxed emoji control
    bool music_like_active_ = false; // deprecated: replaced by music_detected_
    std::chrono::steady_clock::time_point last_vad_speech_time_;
    std::chrono::steady_clock::time_point last_vad_silence_time_;

    // Music detection state
    bool music_detected_ = false;
    int music_ms_accum_ = 0;
    int nonmusic_ms_accum_ = 0;
    // thresholds
    int music_enter_ms_ = 600;   // require ~0.6s of music
    int music_exit_ms_  = 2000;  // require ~2.0s non-music to exit
    float rms_threshold_ = 120.0f; // amplitude threshold for energy - balanced for music detection
    float zcr_min_ = 0.08f;      // min zero-crossing rate (fraction) - allow more music types
    float zcr_max_ = 0.45f;      // max zero-crossing rate (fraction) - allow more music types

    // Helpers
    bool IsMusicLikeFrame(const std::vector<int16_t>& pcm);
    void UpdateMusicState(bool frame_is_music, int frame_ms);
    // Debug aiding
    int music_dbg_frames_ = 0;
    float ema_rms_ = 0.0f;
    float ema_zcr_ = 0.0f;
};

#endif // _APPLICATION_H_
