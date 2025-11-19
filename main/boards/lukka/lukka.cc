#include "wifi_board.h"
#include "audio_codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"
#include "emoji_widget.h"
#include "mmap_generate_moji_emoji.h"
#include "assets/lang_config.h"
#include "backlight.h"
#include "bmi270.h"
#include "bmi2.h"
#include "gyro_sensor.h"
#include "base_controller.h"
#include "motion_detector.h"
#include <memory>
#include <esp_rom_sys.h>
#include "settings.h"

#include <wifi_station.h>
#include <cmath>
#include <esp_log.h>
#include <esp_efuse_table.h>
#include <driver/i2c_master.h>

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_gc9a01.h>
#include <esp_lcd_co5300.h>

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/touch_sensor.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <queue>

#define TAG "MovecallMojiESP32S3"

// LV_FONT_DECLARE(font_puhui_20_4);
// LV_FONT_DECLARE(font_awesome_20_4);


// class CustomLcdDisplay : public SpiLcdDisplay {
// public:
//     CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle, 
//                     esp_lcd_panel_handle_t panel_handle,
//                     int width,
//                     int height,
//                     int offset_x,
//                     int offset_y,
//                     bool mirror_x,
//                     bool mirror_y,
//                     bool swap_xy) 
//         : SpiLcdDisplay(io_handle, panel_handle, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy,
//                     {
//                         .text_font = &font_puhui_20_4,
//                         .icon_font = &font_awesome_20_4,
//                         .emoji_font = font_emoji_64_init(),
//                     }) {

//         DisplayLockGuard lock(this);
//         // 由于屏幕是圆的，所以状态栏需要增加左右内边距
//         lv_obj_set_style_pad_left(status_bar_, LV_HOR_RES * 0.33, 0);
//         lv_obj_set_style_pad_right(status_bar_, LV_HOR_RES * 0.33, 0);
//     }
// };

// CO5300 AMOLED背光控制类
class Co5300Backlight : public Backlight {
public:
    Co5300Backlight(esp_lcd_panel_handle_t panel_handle) : panel_handle_(panel_handle) {}
    
    void SetBrightnessImpl(uint8_t brightness) override {
        ESP_LOGI(TAG, "Set CO5300 brightness to %d%%", brightness);

        // wait for the emoji player to finish any ongoing transmission
        if (auto disp = Board::GetInstance().GetDisplay()) {
            auto widget = static_cast<moji_anim::EmojiWidget*>(disp);
            if (widget && widget->GetPlayer()) {
                auto player = widget->GetPlayer();
                ESP_LOGI(TAG, "Waiting for emoji player to finish transmission before setting brightness...");
                while (player->IsTransmitBusy()) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                esp_lcd_panel_co5300_set_brightness(panel_handle_, brightness);

                ESP_LOGI(TAG, "Brightness set to %d%% after transmission", brightness);
            } else {
                ESP_LOGW(TAG, "Emoji player not available, setting brightness immediately");
                esp_lcd_panel_co5300_set_brightness(panel_handle_, brightness);
            }
        } else {
            ESP_LOGW(TAG, "Display is null, setting brightness immediately");
            esp_lcd_panel_co5300_set_brightness(panel_handle_, brightness);
        }
    }

private:
    esp_lcd_panel_handle_t panel_handle_;
};

// CO5300显示屏专用初始化命令序列 - 466x466分辨率，单线SPI
// 注意：0x2A和0x2B设置的是默认显示窗口，实际绘制时会由驱动重新设置
// 列地址：0到465 (0x0000到0x01D1), 页地址：0到465 (0x0000到0x01D1)
static const co5300_lcd_init_cmd_t co5300_spi_init_cmds[] = {
    {0xFE, (uint8_t[]){0x00}, 1, 0},                   // Page switch
    {0xC4, (uint8_t[]){0x80}, 1, 0},                   // SPI setting, MIPI remove
    {0x3A, (uint8_t[]){0x55}, 1, 0},                   // 55 RGB565, 77 RGB888
    {0x35, (uint8_t[]){0x00}, 1, 0},                   // Tearing effect off
    {0x53, (uint8_t[]){0x20}, 1, 0},                   // Write control display1
    {0x51, (uint8_t[]){0x19}, 1, 0},                   // Write Display Brightness Value in Normal Mode
    {0x63, (uint8_t[]){0x19}, 1, 0},                   // Write Display Brightness Value in HBM Mode
    {0x2A, (uint8_t[]){0x00, 0x06, 0x01, 0xD7}, 4, 0}, // Column Address Set: 0 to 469 (0x0000 to 0x01D5) - 增加几个像素以覆盖全屏
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xD1}, 4, 0}, // Page Address Set: 0 to 465 (0x0000 to 0x01D1)
    {0x11, (uint8_t[]){0x00}, 0, 60},                  // Sleep out + 60ms delay
    {0x29, (uint8_t[]){0x00}, 0, 0},                   // Display on
};

class Lukka : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    Display* display_;
    esp_lcd_panel_handle_t panel_handle_;  // 保存panel handle用于亮度控制
    std::unique_ptr<Co5300Backlight> backlight_;  // CO5300 AMOLED背光控制
    uint32_t touch_pad_threshold_;
    esp_timer_handle_t touchpad_timer_;
    QueueHandle_t touch_event_queue_;
    TaskHandle_t touch_event_task_handle_;
    esp_timer_handle_t emoji_switch_timer_ = nullptr;
    // 简化的触摸检测状态
    bool is_pressed_ = false;  // 当前是否处于按下状态
    int press_start_time_ = 0;  // 按下开始时间
    int last_press_time_ = 0;  // 上次按下时间，用于防抖
    
    // 触摸检测参数
    const int TOUCH_LONG_PRESS_MS = 2000;  // 长按阈值：2秒
    const int TOUCH_SHORT_MIN_MS = 50;     // 短按最小时间：50ms
    const int TOUCH_DEBOUNCE_MS = 100;     // 防抖间隔：100ms
    const int TOUCH_CONFIRM_SAMPLES = 2;   // 触摸确认采样次数：2次
    const int RELEASE_CONFIRM_SAMPLES = 2; // 释放确认采样次数：2次
    int touch_confirm_count_ = 0;          // 触摸确认计数器
    int release_confirm_count_ = 0;        // 释放确认计数器
    bool sleeping_ = false;                   // 睡眠/唤醒状态
    bool is_playing_animation_ = false;       // 是否正在播放动画和声音
    bool IsPlayingAnimation() {
        if (auto disp = GetDisplay()) {
            auto widget = static_cast<moji_anim::EmojiWidget*>(disp);
            if (widget) {
                return widget->IsPlayingAnimation();
            }
        }
        ESP_LOGW(TAG, "Display or EmojiWidget is null in IsPlayingAnimation check");
        return false;
    }
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_handle_ = nullptr;
    bool do_calibration_ = false;
    uint32_t battery_level_ = 0;
    
    // BMI270 sensor wrapper
    std::unique_ptr<Bmi270Sensor> bmi_sensor_;
    esp_timer_handle_t bmi270_init_timer_ = nullptr; // used to defer sensor init
    std::unique_ptr<BaseController> base_controller_;
    std::unique_ptr<MotionDetector> motion_detector_;

    // 车辆姿态检测相关
    struct {
        int64_t last_animation_time = 0;
        const int64_t ANIMATION_COOLDOWN_US = 3000 * 1000;  // 3秒冷却时间，防止重复触发
        int64_t init_start_time = 0;  // 初始化开始时间
        const int64_t INIT_STABILIZATION_US = 5000 * 1000;  // 5秒稳定时间
        bool detection_enabled = false;  // 检测是否启用
        
        // 定时器切换表情相关
        esp_timer_handle_t emoji_switch_timer_ = nullptr;  // 表情切换定时器
        int default_emoji_id_ = MMAP_MOJI_EMOJI_WINKING_AAF;  // 默认表情ID
        const int64_t ANIMATION_PLAY_DURATION_US = 4000 * 1000;  // 动画播放持续时间：4秒
        
        // IDLE状态表情轮播相关
        esp_timer_handle_t idle_emoji_rotation_timer_ = nullptr;  // IDLE表情轮播定时器
        const int64_t IDLE_EMOJI_ROTATION_INTERVAL_US = 5 * 1000 * 1000;  // 5秒轮播间隔
        int idle_emoji_index_ = 0;  // 当前轮播表情索引
        bool is_playing_rotation_emoji_ = false;  // 是否正在播放列表中的表情
        const int idle_emoji_list_[3] = {
            // MMAP_MOJI_EMOJI_DELICIOUS_AAF,    // 0
            MMAP_MOJI_EMOJI_HAPPY_AAF,       // 1
            // MMAP_MOJI_EMOJI_CONFIDENT_AAF,  // 2
            // MMAP_MOJI_EMOJI_SAYHELLO_AAF,       // 3
            // MMAP_MOJI_EMOJI_LOOKAROUND_AAF, // 4
            MMAP_MOJI_EMOJI_WINKING_AAF,    // 5
            MMAP_MOJI_EMOJI_YAWNING_AAF,    // 6
            // MMAP_MOJI_EMOJI_COMFORT_AAF     // 7
        };
    } vehicle_motion_state_;

    // 触摸事件类型
    enum TouchEventType {
        TOUCH_SHORT_PRESS,
        TOUCH_LONG_PRESS
    };
    
    struct TouchEvent {
        TouchEventType type;
        int duration_ms;
    };

    // check if wifi configuration boot
    bool isWifiConfigBoot(){
        Settings settings("wifi", true);
        return (settings.GetInt("config_mode", 0) == 1);
    }


    static void TouchpadTimerCallback(void* arg) {
        Lukka* board = (Lukka*)arg;
        board->PollTouchpad();
    }
    
    static void TouchEventTask(void* arg) {
        Lukka* board = (Lukka*)arg;
        board->HandleTouchEvents();
    }
    
    static void EmojiSwitchTimerCallback(void* arg) {
        Lukka* board = (Lukka*)arg;
        if (board && board->display_) {
            auto widget = static_cast<moji_anim::EmojiWidget*>(board->display_);
            if (widget && widget->GetPlayer()) {
                // 清除播放状态标志，允许响应新事件
                board->is_playing_animation_ = false;
                
                // 检查设备状态，决定播放哪个表情
                DeviceState current_state = Application::GetInstance().GetDeviceState();
                ESP_LOGI("MovecallMojiESP32S3", "Emoji switch timer triggered, device state: %d", (int)current_state);
                
                switch (current_state) {    
                    case kDeviceStateIdle:
                        // IDLE状态下恢复到DEFAULT表情
                        widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_RELAXED_AAF, true, 2);
                        ESP_LOGI("MovecallMojiESP32S3", "Switched back to DEFAULT emoji in IDLE state");
                        break;
                    case kDeviceStateListening:
                    case kDeviceStateSpeaking:
                    default:
                        // 停止IDLE表情轮播
                        board->StopIdleEmojiRotation();
                        widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_RELAXED_AAF, true, 2);
                        break;
                }
            } else {
                ESP_LOGE("MovecallMojiESP32S3", "Failed to get emoji widget or player in timer callback");
                // 即使失败也要清除播放状态标志
                if (board) {
                    board->is_playing_animation_ = false;
                }
            }
        } else {
            ESP_LOGE("MovecallMojiESP32S3", "Board or display is null in timer callback");
            // 即使失败也要清除播放状态标志
            if (board) {
                board->is_playing_animation_ = false;
            }
        }
    }

    static void EmojiEventCallback(void* arg){

    }


    struct VehicleFeedbackCtx {
        Lukka* board;
        int aaf_id;
        const std::string_view* sound;
        esp_timer_handle_t timer;
    };
    static void VehicleFeedbackTimerCallback(void* arg) {
        auto ctx = (VehicleFeedbackCtx*)arg;
        if (ctx && ctx->board) {
            ctx->board->PlayTimedEmoji(ctx->aaf_id);
            // ctx->board->GetDisplay()->SetEmotion(ctx->aaf_id);
            if (ctx->sound) {
                // 保留旧路径（不再直接调用），车辆音效改为使用 PlayLocalPrompt
            }
        }
        if (ctx && ctx->timer) {
            esp_timer_stop(ctx->timer);
            esp_timer_delete(ctx->timer);
        }
        delete ctx;
    }
    void ScheduleVefhicleFeedback(int aaf_id, const std::string_view& sound) {
        auto* ctx = new VehicleFeedbackCtx{this, aaf_id, &sound, nullptr};
        esp_timer_create_args_t targs = {
            .callback = VehicleFeedbackTimerCallback,
            .arg = ctx,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "vehicle_feedback_timer",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&targs, &ctx->timer) == ESP_OK) {
            esp_timer_start_once(ctx->timer, 10 * 1000); // 50ms
        } else {
            // Fallback: immediate if timer creation failed
            VehicleFeedbackTimerCallback(ctx);
        }
    }
    static void BMI270InitTimerCallback(void* arg) {
        Lukka* board = (Lukka*)arg;
        if (board) {
            // Initialize the BMI270 sensor via the wrapper and register a data callback
            if (!board->bmi_sensor_) board->bmi_sensor_ = std::make_unique<Bmi270Sensor>();
            board->bmi_sensor_->Initialize(board->codec_i2c_bus_, [board](const Bmi270Sensor::SensorData& s){
                if (board->motion_detector_) board->motion_detector_->OnSensorData(
                    s.accel.x, s.accel.y, s.accel.z, s.gyro.x, s.gyro.y, s.gyro.z);
            });

        // Create and initialize base controller (wraps motor controller and base probing)
        board->base_controller_ = std::make_unique<BaseController>();
        board->base_controller_->Initialize();
        // Register placement change callback so board UI/sound reacts to base events
        if (board->base_controller_) {
            board->base_controller_->SetPlacementChangedCallback(
                [board](BaseController::PlacementState newState, BaseController::PlacementState oldState) {
                    board->OnPlacementChanged(newState, oldState);
                }
            );
            board->base_controller_->StartProbeTask();
        }
        }
    }

    static void IdleEmojiRotationTimerCallback(void* arg) {
        ESP_LOGI(TAG, "Idle emoji rotation timer triggered");
        Lukka* board = (Lukka*)arg;
        if (board && board->display_) {
            // 检查设备是否仍处于IDLE状态
            DeviceState current_state = Application::GetInstance().GetDeviceState();
            if (current_state == kDeviceStateIdle) {
                // // 获取列表中的下一个表情
                // int next_emoji = board->vehicle_motion_state_.idle_emoji_list_[board->vehicle_motion_state_.idle_emoji_index_];
                
                // ESP_LOGI(TAG, "IDLE emoji rotation: playing emoji %d (index %d) for 2 seconds", 
                //         next_emoji, board->vehicle_motion_state_.idle_emoji_index_);
                
                // 使用PlayTimedEmoji播放表情2秒钟
                // board->PlayTimedEmoji(next_emoji);
                
                // // 将索引移动到下一个表情
                // int len_emoji_list = 3;
                // board->vehicle_motion_state_.idle_emoji_index_ = 
                //     (board->vehicle_motion_state_.idle_emoji_index_ + 1) % len_emoji_list;

                // play blink emoji
                board->vehicle_motion_state_.is_playing_rotation_emoji_ = true;
                auto widget = static_cast<moji_anim::EmojiWidget*>(board->display_);
                if (widget && widget->GetPlayer()) {
                    ESP_LOGI(TAG, "IDLE emoji rotation: playing BLINK emoji for 2.5 seconds");
                    board->vehicle_motion_state_.is_playing_rotation_emoji_ = true;
                    widget->GetPlayer()->TimedPLay(MMAP_MOJI_EMOJI_BLINK_AAF, 2.5f, 10, [board]() {
                        // 表情播放完成后的回调
                        board->OnEmojiPlaybackComplete();
                    });
                }
                
                // 启动定时器，30秒后切换到下一个表情
                // esp_timer_start_once(board->vehicle_motion_state_.idle_emoji_rotation_timer_, 
                //                    board->vehicle_motion_state_.IDLE_EMOJI_ROTATION_INTERVAL_US);
            } else {
                ESP_LOGI(TAG, "Device no longer in IDLE state, stopping emoji rotation");
            }
        } else {
            ESP_LOGE(TAG, "Board or display is null in idle rotation callback");
        }
    }

    // 表情播放完成回调
    void OnEmojiPlaybackComplete() {
        // 只有在IDLE状态且正在播放轮播表情时才处理
        if (!display_ || Application::GetInstance().GetDeviceState() != kDeviceStateIdle) {
            return;
        }
        
        if (!vehicle_motion_state_.is_playing_rotation_emoji_) {
            return; // 不是轮播表情，忽略
        }
        
        auto widget = static_cast<moji_anim::EmojiWidget*>(display_);
        if (!widget || !widget->GetPlayer()) {
            return;
        }
        
        // 切换回DEFAULT表情（循环播放）
        widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_RELAXED_AAF, true, 2);
        vehicle_motion_state_.is_playing_rotation_emoji_ = false;
        
        ESP_LOGI(TAG, "Rotation emoji playback complete, switched back to DEFAULT emoji");
        
        // 将索引移动到下一个表情
        // vehicle_motion_state_.idle_emoji_index_ = 
        //     (vehicle_motion_state_.idle_emoji_index_ + 1) % 8;
        
        // 启动定时器，60秒后切换到下一个表情
        esp_timer_start_once(vehicle_motion_state_.idle_emoji_rotation_timer_, 
                           vehicle_motion_state_.IDLE_EMOJI_ROTATION_INTERVAL_US);
    }

    // 本地提示音播放：在 Idle 下短暂开启输出，播放完成后自动关闭
    static void SoundDisableOutTimerCb(void* arg) {
        auto* board = static_cast<Lukka*>(arg);
        if (!board) return;
        auto state = Application::GetInstance().GetDeviceState();
        if (state == kDeviceStateIdle) {
            // 清空音频队列并关闭输出，确保不影响下一次播放
            Application::GetInstance().ClearAudioQueueAndDisableOutput();
            ESP_LOGI(TAG, "Vehicle motion sound completed, cleared audio queue and disabled output");
        }
    }
    
    void PlayLocalPrompt(const std::string_view& sound, int64_t disable_after_us = 2000000) {
        if (Application::GetInstance().IsCodecInitDone() == false) {
            ESP_LOGW(TAG, "Audio codec not initialized yet, cannot play local prompt");
            return;
        }
        // 清空音频队列并重置解码器状态，然后重新启用输出
        Application::GetInstance().ClearAudioQueueAndDisableOutput();
        auto codec = GetAudioCodec();
        if (codec) {
            if (!codec->output_enabled()) codec->EnableOutput(true);
        }
        
        // 直接播放声音，与唤醒词保持一致
        Application::GetInstance().PlaySound(sound);
        
        // 等待60ms，与唤醒词逻辑保持一致
        vTaskDelay(pdMS_TO_TICKS(60));

        // 使用定时器在音频播放完成后关闭输出
        // 估算音频播放时间：车辆动作声音通常1-2秒
        esp_timer_handle_t t_off = nullptr;
        esp_timer_create_args_t args_off = {
            .callback = SoundDisableOutTimerCb,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "prompt_sound_off",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&args_off, &t_off) == ESP_OK) {
            esp_timer_start_once(t_off, disable_after_us); // 使用参数指定的延迟时间关闭输出
        }
    }
    void PollTouchpad() {
        uint32_t touch_value;
        esp_err_t ret = touch_pad_read_raw_data(TOUCH_PAD_CHANNEL, &touch_value);
        if (ret != ESP_OK) {
            return;
        }
        
        bool is_touched = (touch_value > touch_pad_threshold_);  // 触摸时值变高
        int current_time = (int)(esp_timer_get_time() / 1000);  // 当前时间（毫秒）
        
        // 检测触摸按下
        if (is_touched && !is_pressed_) {
            touch_confirm_count_++;
            release_confirm_count_ = 0;  // 重置释放确认计数器
            ESP_LOGD(TAG, "Touch detected, confirm count: %d/%d, value: %lu", 
                    touch_confirm_count_, TOUCH_CONFIRM_SAMPLES, touch_value);
            if (touch_confirm_count_ >= TOUCH_CONFIRM_SAMPLES) {
                // 触摸确认，开始按下状态
                is_pressed_ = true;
                press_start_time_ = current_time;
                ESP_LOGI(TAG, "Touch pressed, value: %lu", touch_value);
            }
        }
        // 检测触摸释放
        else if (!is_touched && is_pressed_) {
            release_confirm_count_++;
            ESP_LOGD(TAG, "Release detected, confirm count: %d/%d, value: %lu", 
                    release_confirm_count_, RELEASE_CONFIRM_SAMPLES, touch_value);
            if (release_confirm_count_ >= RELEASE_CONFIRM_SAMPLES) {
                // 触摸释放确认，计算持续时间
                is_pressed_ = false;
                touch_confirm_count_ = 0;  // 重置确认计数器
                release_confirm_count_ = 0;  // 重置释放确认计数器
                int press_duration = (int)(current_time - press_start_time_);
                
                // 防抖检查：忽略太短的触摸
                if (press_duration < TOUCH_SHORT_MIN_MS) {
                    ESP_LOGD(TAG, "Touch too short, ignored: %d ms", press_duration);
                    return;
                }
                
                // 防抖检查：避免连续快速触摸
                if (current_time - last_press_time_ < TOUCH_DEBOUNCE_MS) {
                    ESP_LOGD(TAG, "Touch too frequent, ignored. Interval: %d ms", 
                            current_time - last_press_time_);
                    return;
                }
                
                // 判断事件类型
                if (press_duration >= TOUCH_LONG_PRESS_MS) {
                    // 长按：≥2秒
                    ESP_LOGI(TAG, "Long press detected: %d ms", press_duration);
                    TouchEvent event = {TOUCH_LONG_PRESS, press_duration};
                    if (xQueueSend(touch_event_queue_, &event, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Touch event queue full, dropping long press event");
                    }
                } else {
                    // 短按：50ms-2秒
                    ESP_LOGI(TAG, "Short press detected: %d ms", press_duration);
                    TouchEvent event = {TOUCH_SHORT_PRESS, press_duration};
                    if (xQueueSend(touch_event_queue_, &event, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Touch event queue full, dropping short press event");
                    }
                }
                
                last_press_time_ = current_time;
            }
        }
        // 重置确认计数器（如果触摸中断）
        else if (!is_touched && !is_pressed_) {
            touch_confirm_count_ = 0;
            release_confirm_count_ = 0;
        }
    }
    
    void HandleTouchEvents() {
        TouchEvent event;
        while (true) {
            if (xQueueReceive(touch_event_queue_, &event, portMAX_DELAY) == pdTRUE) {
                switch (event.type) {
                    case TOUCH_LONG_PRESS:
                        ESP_LOGI(TAG, "Handling long press event, duration: %d ms", event.duration_ms);
                        // 使用Schedule避免阻塞事件处理任务
                        /*Application::GetInstance().Schedule([this]() {
                            Application::GetInstance().ToggleChatState();
                        });*/
                        break;
                        
                    case TOUCH_SHORT_PRESS:
                        ESP_LOGI(TAG, "Handling short press event, duration: %d ms", event.duration_ms);
                        // 如果正在播放动画和声音，不响应新的触摸事件
                        if (IsPlayingAnimation()) {
                            ESP_LOGD(TAG, "Animation playing, skipping touch event");
                            break;
                        }
                        // 仅在 Idle 下播放提示音，不切状态
                        {
                            DeviceState current_state = Application::GetInstance().GetDeviceState();
                            if (current_state == kDeviceStateIdle) {
                                // 声音早于动画结束：敲击动画1s，这里设为0.9s
                                PlayLocalPrompt(Lang::Sounds::P3_KNOCKING, 1000000); // 1秒后关闭
                            } else {
                                ESP_LOGI(TAG, "Device not idle, skip touch prompt sound");
                            }
                        }

                        if (display_) {
                            auto widget = static_cast<moji_anim::EmojiWidget*>(display_);
                            if (widget && widget->GetPlayer()) {
                                // 设置播放状态标志，防止重复触发
                                // is_playing_animation_ = true;
                                
                                ESP_LOGI(TAG, "Playing shocked emoji...");
                                // widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_KNOCKING_AAF, false, 4);
                                PlayTimedEmoji(MMAP_MOJI_EMOJI_KNOCKING_AAF, 1.0f);
                                // // 使用定时器延迟切换表情，避免阻塞事件处理
                                // if (emoji_switch_timer_ == nullptr) {
                                //     esp_timer_create_args_t timer_args = {
                                //         .callback = EmojiSwitchTimerCallback,
                                //         .arg = this,
                                //         .dispatch_method = ESP_TIMER_TASK,
                                //         .name = "emoji_switch_timer",
                                //         .skip_unhandled_events = true,
                                //     };
                                //     esp_timer_create(&timer_args, &emoji_switch_timer_);
                                // }
                                // esp_timer_stop(emoji_switch_timer_);
                                // esp_timer_start_once(emoji_switch_timer_, 1000 * 1000); // 1秒后切换
                                // ESP_LOGI(TAG, "Shocked emoji played, timer set for emoji switch");
                            } else {
                                ESP_LOGE(TAG, "Failed to get emoji widget or player");
                            }
                        } else {
                            ESP_LOGE(TAG, "Display is null");
                        }
                        break;
                }
            }
        }
    }
    
    // Motion detection (vehicle posture & shake) moved to MotionDetector to decouple concerns.


    // 统一的表情播放（定时自动恢复），供车辆检测与放置状态切换复用
    void PlayTimedEmoji(int aaf_id, float time=2.0f) {
        if (!display_) {
            ESP_LOGW(TAG, "Display not available for vehicle animation");
            return;
        }
        
        auto widget = static_cast<moji_anim::EmojiWidget*>(display_);
        if (!widget || !widget->GetPlayer()) {
            ESP_LOGW(TAG, "Emoji widget or player not available");
            return;
        }

        if (IsPlayingAnimation()) {
            ESP_LOGI(TAG, "Animation already playing, skipping new emoji: %d", aaf_id);
            return;
        }
        
        // 设置播放状态标志，防止重复触发
        // is_playing_animation_ = true;
        
        // 更新动画播放时间
        // vehicle_motion_state_.last_animation_time = esp_timer_get_time();
        
        // 播放动画（循环播放）
        // widget->GetPlayer()->StartPlayer(aaf_id, true, 2);
        // widget->GetPlayer()->TimedPLay(aaf_id, time, 10, [this]() {
        //     EmojiSwitchTimerCallback(this);
        // });

        widget->PlayEmoji(aaf_id);
        // ESP_LOGI(TAG, "Playing emoji animation: %d (loop play)", aaf_id);
        ESP_LOGI(TAG, "Playing emoji animation: %d (play once)", aaf_id);

        // // 启动定时器，2秒后切换回默认表情
        // if (vehicle_motion_state_.emoji_switch_timer_ == nullptr) {
        //     esp_timer_create_args_t timer_args = {
        //         .callback = EmojiSwitchTimerCallback,
        //         .arg = this,
        //         .dispatch_method = ESP_TIMER_TASK,
        //         .name = "emoji_switch_timer",
        //         .skip_unhandled_events = true,
        //     };
        //     esp_timer_create(&timer_args, &vehicle_motion_state_.emoji_switch_timer_);
        // }
        
        // // // 停止之前的定时器并启动新的
        // esp_timer_stop(vehicle_motion_state_.emoji_switch_timer_);
        // esp_timer_start_once(vehicle_motion_state_.emoji_switch_timer_, time * 1000 * 1000);
    }

    void OnPlacementChanged(BaseController::PlacementState newState, BaseController::PlacementState oldState){
        // Map to previous behavior: enable/disable vehicle detection and play UI/sounds
        if (newState == BaseController::kPlacementIndependent) {
            if(motion_detector_) motion_detector_->SetPlacementIndependent(true);
            if (oldState == BaseController::kPlacementRotatingBase) {
                ESP_LOGI(TAG, "Placement changed to INDEPENDENT");
                if (display_) {
                    auto widget = static_cast<moji_anim::EmojiWidget*>(display_);
                    if (widget && widget->GetPlayer()) {
                        if (IsPlayingAnimation()){
                            ESP_LOGI(TAG, "Animation playing, skipping uninstall emoji");
                            return;
                        }
                        PlayTimedEmoji(MMAP_MOJI_EMOJI_UNINSTALL_AAF);
                        // Application::GetInstance().PlaySound(Lang::Sounds::P3_POPUP);
                        PlayLocalPrompt(Lang::Sounds::P3_POPUP, 2000000);
                        // ESP_LOGI(TAG, "Playing uninstall emoji animation...");
                        // widget->GetPlayer()->PlayOnce(MMAP_MOJI_EMOJI_UNINSTALL_AAF, 2, [this](){
                        //     ESP_LOGI(TAG, "PlayCallback called: uninstall animation completed");
                        //     EmojiSwitchTimerCallback(this);
                        // });
                    }
                }   
            }
        } else if (newState == BaseController::kPlacementRotatingBase) {
            if (motion_detector_) motion_detector_->SetPlacementIndependent(false);
            if (oldState == BaseController::kPlacementIndependent) {
                ESP_LOGI(TAG, "Placement changed to ROTATING_BASE");
                if (base_controller_) base_controller_->ResetMotor();
                if (display_) {
                    auto widget = static_cast<moji_anim::EmojiWidget*>(display_);
                    if (widget && widget->GetPlayer()) {
                        if (IsPlayingAnimation()){
                            ESP_LOGI(TAG, "Animation playing, skipping connecting emoji");
                            return;
                        }
                        PlayTimedEmoji(MMAP_MOJI_EMOJI_CONNECTING_AAF);
                        // Application::GetInstance().PlaySound(Lang::Sounds::P3_POWERUP);
                        PlayLocalPrompt(Lang::Sounds::P3_POWERUP, 2000000);
                        // ESP_LOGI(TAG, "Playing connecting emoji animation...");
                        // widget->GetPlayer()->PlayOnce(MMAP_MOJI_EMOJI_CONNECTING_AAF, 2, [this](){
                        //     ESP_LOGI(TAG, "PlayCallback called: connecting animation completed");
                        //     EmojiSwitchTimerCallback(this);
                        // });
                    }
                }
            }
        }
    }

    // 车辆姿态事件处理
    void OnMotionEvent(MotionDetector::MotionEvent ev) {
        // Board handles gating and playback
        if (IsPlayingAnimation()) {
            ESP_LOGD(TAG, "Animation playing, skipping motion event");
            return;
        }
        DeviceState current_state = Application::GetInstance().GetDeviceState();
        if (current_state != kDeviceStateIdle) {
            ESP_LOGD(TAG, "Device not idle, skipping motion event");
            return;
        }

        const std::string_view* sound = nullptr;
        int aaf_id = MMAP_MOJI_EMOJI_RELAXED_AAF;
        switch (ev) {
            case MotionDetector::MotionEvent::Speeding:
                sound = &Lang::Sounds::P3_SPEEDING;
                aaf_id =  MMAP_MOJI_EMOJI_SPEEDING_AAF;
                break;
            case MotionDetector::MotionEvent::Braking:
                sound = &Lang::Sounds::P3_BRAKING;
                aaf_id =  MMAP_MOJI_EMOJI_BRAKING_AAF;
                break;
            case MotionDetector::MotionEvent::TurnLeft:
                sound = &Lang::Sounds::P3_TURN;
                aaf_id =  MMAP_MOJI_EMOJI_TURNLEFT_AAF;
                break;
            case MotionDetector::MotionEvent::TurnRight:
                sound = &Lang::Sounds::P3_TURN;
                aaf_id =  MMAP_MOJI_EMOJI_TURNRIGHT_AAF;
                break;
        }

        // is_playing_animation_ = true;
        PlayTimedEmoji(aaf_id);
        if (sound) PlayLocalPrompt(*sound, vehicle_motion_state_.ANIMATION_PLAY_DURATION_US - 100000);

        // ensure emoji switch timer will clear the playing flag
        // if (vehicle_motion_state_.emoji_switch_timer_ == nullptr) {
        //     esp_timer_create_args_t timer_args = {
        //         .callback = EmojiSwitchTimerCallback,
        //         .arg = this,
        //         .dispatch_method = ESP_TIMER_TASK,
        //         .name = "emoji_switch_timer",
        //         .skip_unhandled_events = true,
        //     };
        //     esp_timer_create(&timer_args, &vehicle_motion_state_.emoji_switch_timer_);
        // }
        // esp_timer_stop(vehicle_motion_state_.emoji_switch_timer_);
        // esp_timer_start_once(vehicle_motion_state_.emoji_switch_timer_, vehicle_motion_state_.ANIMATION_PLAY_DURATION_US);
    }

    // 晃动事件处理
    void OnShakeEvent() {
        if (IsPlayingAnimation()) return; // 正在播放动画，跳过
        DeviceState current_state = Application::GetInstance().GetDeviceState();
        if (current_state != kDeviceStateIdle) return;
        // is_playing_animation_ = true;
        PlayTimedEmoji(MMAP_MOJI_EMOJI_DIZZY_AAF);
        PlayLocalPrompt(Lang::Sounds::P3_VIBRATION, vehicle_motion_state_.ANIMATION_PLAY_DURATION_US - 100000);
        // if (vehicle_motion_state_.emoji_switch_timer_ == nullptr) {
        //     esp_timer_create_args_t timer_args = {
        //         .callback = EmojiSwitchTimerCallback,
        //         .arg = this,
        //         .dispatch_method = ESP_TIMER_TASK,
        //         .name = "emoji_switch_timer",
        //         .skip_unhandled_events = true,
        //     };
        //     esp_timer_create(&timer_args, &vehicle_motion_state_.emoji_switch_timer_);
        // }
        // esp_timer_stop(vehicle_motion_state_.emoji_switch_timer_);
        // esp_timer_start_once(vehicle_motion_state_.emoji_switch_timer_, vehicle_motion_state_.ANIMATION_PLAY_DURATION_US);
    }

    


    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }
    
    // BMI270 initialization moved into Bmi270Sensor wrapper (gyro_sensor.cc)

    // SPI初始化
    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize SPI bus");
        spi_bus_config_t buscfg = CO5300_PANEL_BUS_SPI_CONFIG(DISPLAY_SPI_SCLK_PIN, DISPLAY_SPI_MOSI_PIN, 
                                    DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    // VCI_EN初始化
    void InitializeVciEn() {
        ESP_LOGI(TAG, "Initialize VCI_EN on GPIO %d", VCI_EN_GPIO);
        
        // 配置GPIO为输出模式，初始输出高电平
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << VCI_EN_GPIO),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));

        ESP_ERROR_CHECK(gpio_set_level(VCI_EN_GPIO, 0));
        vTaskDelay(pdMS_TO_TICKS(120));
        
        // 输出高电平使能显示屏电源
        ESP_ERROR_CHECK(gpio_set_level(VCI_EN_GPIO, 1));
        ESP_LOGI(TAG, "VCI_EN initialized, output high");
    }

    // CO5300 AMOLED显示屏初始化 - 使用官方CO5300驱动，单线SPI模式
    void InitializeGc9a01Display() {
        ESP_LOGI(TAG, "Init CO5300 AMOLED display using official CO5300 driver (SPI mode)");

        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_handle_t io_handle = NULL;
        esp_lcd_panel_io_spi_config_t io_config = CO5300_PANEL_IO_SPI_CONFIG(DISPLAY_SPI_CS_PIN, DISPLAY_SPI_DC_PIN, NULL, NULL);
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &io_handle));
    
        ESP_LOGI(TAG, "Install CO5300 panel driver");
        esp_lcd_panel_handle_t panel_handle = NULL;
        
        // 配置vendor config，使用自定义初始化序列
        co5300_vendor_config_t vendor_config = {
            .init_cmds = co5300_spi_init_cmds,
            .init_cmds_size = sizeof(co5300_spi_init_cmds) / sizeof(co5300_lcd_init_cmd_t),
            .flags = {
                .use_spi_interface = 1,  // 使用SPI接口
            },
        };
        
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_SPI_RESET_PIN;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = &vendor_config;

        ESP_ERROR_CHECK(esp_lcd_new_panel_co5300(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        // 不启用颜色反转
        // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
        // 设置显示窗口偏移，确保全屏显示（如果需要）
        // esp_lcd_panel_set_gap(panel_handle, 0, 0);
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true)); 

        // 注释掉全屏清黑色，避免SPI传输过大
        // 466x466分辨率的屏幕单次全屏传输约434KB，可能导致SPI传输失败
        // ESP_LOGI(TAG, "Clear screen to black");
        // std::vector<uint16_t> black(DISPLAY_WIDTH * DISPLAY_HEIGHT, 0x0000);
        // esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, black.data());

        ESP_LOGI(TAG, "Creating EmojiWidget for display -- ");
        display_ = new moji_anim::EmojiWidget(panel_handle, io_handle);
        ESP_LOGI(TAG, "EmojiWidget created successfully");
        
        // 保存panel handle用于亮度控制
        panel_handle_ = panel_handle;
        
        // 创建CO5300背光控制对象
        backlight_ = std::make_unique<Co5300Backlight>(panel_handle);
        if (backlight_) {
            backlight_->RestoreBrightness();
        }
        
        // 启动IDLE状态的表情轮播
        // ESP_LOGI(TAG, "Starting IDLE emoji rotation after display initialization");
        // StartIdleEmojiRotation();
    }

    void InitializeTouchPad() {
        ESP_LOGI(TAG, "Initialize Touch Sensor on GPIO %d, Channel %d", TOUCH_PAD_GPIO, TOUCH_PAD_CHANNEL);
        
        // 初始化TOUCH SENSOR
        ESP_ERROR_CHECK(touch_pad_init());
        ESP_LOGI(TAG, "Touch pad init completed");
        
        // 配置触摸通道
        ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_CHANNEL));
        ESP_LOGI(TAG, "Touch pad config completed for channel %d", TOUCH_PAD_CHANNEL);
        
        // 设置FSM模式为定时器触发
        ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
        ESP_LOGI(TAG, "Touch pad FSM mode set to timer");
        
        // 启动FSM
        ESP_ERROR_CHECK(touch_pad_fsm_start());
        ESP_LOGI(TAG, "Touch pad FSM started");
        
        // 等待稳定
        ESP_LOGI(TAG, "Waiting for touch sensor to stabilize...");
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 校准TOUCH SENSOR - 增加采样次数和间隔
        uint32_t touch_value_sum = 0;
        const int calibration_samples = 10;
        
        ESP_LOGI(TAG, "Calibrating touch sensor with %d samples...", calibration_samples);
        for (int i = 0; i < calibration_samples; i++) {
            uint32_t touch_value;
            esp_err_t ret = touch_pad_read_raw_data(TOUCH_PAD_CHANNEL, &touch_value);
            if (ret == ESP_OK) {
                touch_value_sum += touch_value;
                ESP_LOGI(TAG, "Sample %d: %lu", i + 1, touch_value);
            } else {
                ESP_LOGE(TAG, "Failed to read touch value: %s", esp_err_to_name(ret));
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        uint32_t avg_touch_value = touch_value_sum / calibration_samples;
        // 调整阈值计算，使用更灵敏的设置
        touch_pad_threshold_ = avg_touch_value * 1.05;  // 降低阈值，提高灵敏度  
        
        ESP_LOGI(TAG, "Touch calibration complete:");
        ESP_LOGI(TAG, "  Average baseline: %lu", avg_touch_value);
        ESP_LOGI(TAG, "  Touch threshold: %lu", touch_pad_threshold_);
        ESP_LOGI(TAG, "  Threshold ratio: %.2f", (float)touch_pad_threshold_ / avg_touch_value);
        
        // 设置触摸阈值
        ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_PAD_CHANNEL, touch_pad_threshold_));
        ESP_LOGI(TAG, "Touch threshold set to %lu", touch_pad_threshold_);
        
        // 创建定时器，20ms间隔轮询
        esp_timer_create_args_t timer_args = {
            .callback = TouchpadTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touchpad_timer",
            .skip_unhandled_events = true,
        };
        
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &touchpad_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(touchpad_timer_, 20 * 1000));  // 20ms
        ESP_LOGI(TAG, "Touch pad timer started with 20ms interval");

        // 创建触摸事件队列
        touch_event_queue_ = xQueueCreate(10, sizeof(TouchEvent)); // 队列大小为10
        if (touch_event_queue_ == NULL) {
            ESP_LOGE(TAG, "Failed to create touch event queue");
            return;
        }

        // 创建触摸事件处理任务
        BaseType_t ret = xTaskCreate(TouchEventTask, "touch_event_task", 2048, this, 1, &touch_event_task_handle_);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create touch event task");
            vQueueDelete(touch_event_queue_);
            return;
        }
    }
    
    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker")); 
        thing_manager.AddThing(iot::CreateThing("Screen"));   
    }
    void InitializeBatteryAdc_ChrgStat() {
        // 初始化 ADC1
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));

        // 配置 ADC 通道
        adc_oneshot_chan_cfg_t chan_config = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, BATTERY_ADC_CHANNEL, &chan_config));

        // 创建 ADC 校准句柄
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle_);
        if (ret == ESP_OK) {
            do_calibration_ = true;
            ESP_LOGI(TAG, "ADC Curve Fitting calibration succeeded");
        } else {
            ESP_LOGW(TAG, "ADC Curve Fitting calibration failed, using raw values");
        }
#endif
    
        // 配置为输入模式，并启用内部上拉
        gpio_config_t gpio_cfg = {
            .pin_bit_mask = (1ULL << BATTERY_CHRG_STAT_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE, // 启用内部上拉
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
    }

    // 读取电池ADC数据
    void ReadBatteryAdcData() {
        int adc_value;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, BATTERY_ADC_CHANNEL, &adc_value));
        ESP_LOGD(TAG, "ADC value: %d", adc_value);

        int voltage = 0; // mV
        
        if (do_calibration_) {
            // 使用校准转换
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle_, adc_value, &voltage));
            ESP_LOGD(TAG, "Calibrated voltage: %d mV", voltage);
        } else {
            // 使用原始值（简单转换）
            voltage = (adc_value * 2500) / 4095;
            ESP_LOGI(TAG, "Raw voltage: %d mV", voltage);
        }

        // 将实际电压乘以2得到调整后的电压
        uint32_t adjusted_voltage = voltage * 2;
        ESP_LOGD(TAG, "Adjusted voltage: %ld mV", adjusted_voltage);

        // 定义电池电量区间（基于调整后的电压值）
        const struct {
            uint16_t voltage;
            uint8_t level;
        } levels[] = {
            {3000, 0},
            {3400, 20},
            {3650, 40},
            {3800, 60},
            {4000, 80},
            {4200, 100}
        };

        // 计算电池电量
        if (adjusted_voltage < levels[0].voltage) {
            battery_level_ = 0;
        } else if (adjusted_voltage >= levels[5].voltage) {
            battery_level_ = 100;
        } else {
            for (int i = 0; i < 5; i++) {
                if (adjusted_voltage >= levels[i].voltage && adjusted_voltage < levels[i+1].voltage) {
                    float ratio = static_cast<float>(adjusted_voltage - levels[i].voltage) / (levels[i+1].voltage - levels[i].voltage);
                    battery_level_ = levels[i].level + ratio * (levels[i+1].level - levels[i].level);
                    break;
                }
            }
        }

        ESP_LOGD(TAG, "Battery level: %ld%%", battery_level_);
    }

public:
    Lukka() : boot_button_(BOOT_BUTTON_GPIO) {  
        InitializeBatteryAdc_ChrgStat();
        InitializeCodecI2c();
        InitializeSpi();
        InitializeVciEn();
        InitializeGc9a01Display();

        // if it's the first startup, set wifi config mode
        if (isFirstStartup()){
            Settings settings("wifi", true);
            if (settings.GetInt("config_mode", 0) != 2) {
                settings.SetInt("config_mode", 1);
                Application::GetInstance().PlaySound(Lang::Sounds::P3_WIFICONFIG);
            }
            else{
                ESP_LOGI(TAG, "WiFi config done, waiting for activation");
            }
        }

        // if device is to enter wifi config mode, break here
        if (isWifiConfigBoot()){
            ESP_LOGI(TAG, "Entering WiFi configuration mode on boot");
            return;
        }

        //播放开机音效，临时放在这里，后续优化
        Application::GetInstance().PlaySound(Lang::Sounds::P3_POWERUP);
        InitializeTouchPad();  // 初始化触摸传感器（非触摸屏触控）
        InitializeButtons();
        InitializeIot();



        // Create motion detector and provide simple event callbacks into this board
        motion_detector_ = std::make_unique<MotionDetector>(
            [this](MotionDetector::MotionEvent ev) {
                OnMotionEvent(ev);
            },
            [this]() {
                OnShakeEvent();
            }
        );


        // 延后初始化BMI270，避免与编解码器I2C初始化阶段冲突
        esp_timer_create_args_t bmi_init_args = {
            .callback = BMI270InitTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "bmi270_init",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&bmi_init_args, &bmi270_init_timer_) == ESP_OK) {
            // 延迟1.5秒执行
            esp_timer_start_once(bmi270_init_timer_, 1500 * 1000);
        }

        // BaseController handles base probing (task started in BaseController)
    }

    virtual Led* GetLed() override {
        static SingleLed led_strip(BUILTIN_LED_GPIO);
        return &led_strip;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        return backlight_.get();  // 返回CO5300 AMOLED背光控制对象
    }

    virtual AudioCodec* GetAudioCodec() override {
        bool use_input_reference = AUDIO_INPUT_REFERENCE;
        if (isWifiConfigBoot()){
            use_input_reference = false;
        }

        static BoxAudioCodec audio_codec(
            codec_i2c_bus_, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7210_ADDR, 
            use_input_reference);
        return &audio_codec;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        ReadBatteryAdcData();
        level = battery_level_;
        
        // 低电平表示正在充电，高电平或高阻态（Hi-Z）表示充电完成
        bool charging_pin_level = gpio_get_level(BATTERY_CHRG_STAT_GPIO);
        charging = !charging_pin_level;  // 低电平表示正在充电
        discharging = charging_pin_level;  // 高电平或高阻态表示充电完成（放电状态）
        ESP_LOGD(TAG, "Charging pin level: %d, charging: %s, discharging: %s (高电平或高阻态=充电完成)", 
                charging_pin_level, charging ? "true" : "false", discharging ? "true" : "false");
        
        return true;
    }

    // base motor control
    void MojiControlMotor(char direction, int steps) override {
        if (direction != 'L' && direction != 'R') return;
        if (!base_controller_) base_controller_ = std::make_unique<BaseController>();
        if (!base_controller_->IsInitialized()) base_controller_->Initialize();
        if (base_controller_->IsInitialized()) base_controller_->ControlMotor(direction, steps);
    }
    void MojiResetMotor() override {
        if (!base_controller_) base_controller_ = std::make_unique<BaseController>();
        if (!base_controller_->IsInitialized()) base_controller_->Initialize();
        if (base_controller_->IsInitialized()) base_controller_->ResetMotor();
    }

    bool isFirstStartup() {
        Settings settings("first_startup", false);
        int first_startup = settings.GetInt("first_startup", 0);
        return (first_startup == 0 );
    }

    void CheckFirstStartup() override {
        if (!isFirstStartup()) {
            ESP_LOGI(TAG, "Not first startup, skipping first startup actions");
            return;
        }
        // the flag will be cleared after activation completes

        ESP_LOGI(TAG, "First startup detected, performing first startup actions");

        // show blink emoji and play sound
        auto play_done = std::make_shared<bool>(false);
        if (display_) {
            auto widget = static_cast<moji_anim::EmojiWidget*>(display_);
            if (widget && widget->GetPlayer()) {
                ESP_LOGI(TAG, "Playing blink emoji for first startup...");
                widget->GetPlayer()->TimedPLay(MMAP_MOJI_EMOJI_BLINK_AAF, 2.5f, 10, [this, play_done]() {
                    *play_done = true;
                    ESP_LOGI(TAG, "Blink emoji play completed");
                    // after blink, play sound and switch to default emoji
                    Application::GetInstance().PlaySound(Lang::Sounds::P3_POWERUP);
                    EmojiSwitchTimerCallback(this);
                });
            }
        }

        // wait for play done
        ESP_LOGI(TAG, "Waiting for blink emoji to finish playing...");
        while (!*play_done) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        ESP_LOGI(TAG, "First startup actions completed");
        // back to app for network setup
        return;
    }

    // IDLE状态表情轮播管理函数
    void StartIdleEmojiRotation() {
        if (!display_) {
            ESP_LOGW(TAG, "Display not available for idle emoji rotation");
            return;
        }
        
        auto widget = static_cast<moji_anim::EmojiWidget*>(display_);
        if (!widget || !widget->GetPlayer()) {
            ESP_LOGW(TAG, "Emoji widget or player not available for idle rotation");
            return;
        }
        
        // 创建定时器（如果还没有创建）
        if (vehicle_motion_state_.idle_emoji_rotation_timer_ == nullptr) {
            esp_timer_create_args_t timer_args = {
                .callback = IdleEmojiRotationTimerCallback,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "idle_emoji_rotation_timer",
                .skip_unhandled_events = true,
            };
            esp_err_t ret = esp_timer_create(&timer_args, &vehicle_motion_state_.idle_emoji_rotation_timer_);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to create idle emoji rotation timer: %s", esp_err_to_name(ret));
                return;
            }
        }
        
        // 停止之前的定时器
        esp_timer_stop(vehicle_motion_state_.idle_emoji_rotation_timer_);
        
        // 重置轮播状态
        vehicle_motion_state_.is_playing_rotation_emoji_ = false;
        
        // 播放DEFAULT表情（循环播放）
        widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_RELAXED_AAF, true, 2);
        
        ESP_LOGI(TAG, "Started IDLE emoji rotation with DEFAULT emoji");
        ESP_LOGI(TAG, "IDLE emoji rotation interval: %lld us (30 seconds)", 
                vehicle_motion_state_.IDLE_EMOJI_ROTATION_INTERVAL_US);
        
        // 启动定时器，30秒后切换到列表中的第一个表情
        esp_timer_start_once(vehicle_motion_state_.idle_emoji_rotation_timer_, 
                           vehicle_motion_state_.IDLE_EMOJI_ROTATION_INTERVAL_US);
        
        ESP_LOGI(TAG, "Idle emoji rotation timer started, will trigger in 30 seconds");
    }
    
    void StopIdleEmojiRotation() {
        if (vehicle_motion_state_.idle_emoji_rotation_timer_) {
            esp_timer_stop(vehicle_motion_state_.idle_emoji_rotation_timer_);
            ESP_LOGI(TAG, "Stopped IDLE emoji rotation");
        }
    }

    // Independent shake detection moved into MotionDetector
};

DECLARE_BOARD(Lukka);
