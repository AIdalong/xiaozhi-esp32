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
#include <esp_rom_sys.h>

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

#define TAG "MovecallMojiESP32S3"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);


class CustomLcdDisplay : public SpiLcdDisplay {
public:
    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle, 
                    esp_lcd_panel_handle_t panel_handle,
                    int width,
                    int height,
                    int offset_x,
                    int offset_y,
                    bool mirror_x,
                    bool mirror_y,
                    bool swap_xy) 
        : SpiLcdDisplay(io_handle, panel_handle, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy,
                    {
                        .text_font = &font_puhui_20_4,
                        .icon_font = &font_awesome_20_4,
                        .emoji_font = font_emoji_64_init(),
                    }) {

        DisplayLockGuard lock(this);
        // 由于屏幕是圆的，所以状态栏需要增加左右内边距
        lv_obj_set_style_pad_left(status_bar_, LV_HOR_RES * 0.33, 0);
        lv_obj_set_style_pad_right(status_bar_, LV_HOR_RES * 0.33, 0);
    }
};

// CO5300 AMOLED背光控制类
class Co5300Backlight : public Backlight {
public:
    Co5300Backlight(esp_lcd_panel_handle_t panel_handle) : panel_handle_(panel_handle) {}
    
    void SetBrightnessImpl(uint8_t brightness) override {
        ESP_LOGI(TAG, "Set CO5300 brightness to %d%%", brightness);
        esp_lcd_panel_co5300_set_brightness(panel_handle_, brightness);
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
    {0x51, (uint8_t[]){0xFF}, 1, 0},                   // Write Display Brightness Value in Normal Mode
    {0x63, (uint8_t[]){0xFF}, 1, 0},                   // Write Display Brightness Value in HBM Mode
    {0x2A, (uint8_t[]){0x00, 0x06, 0x01, 0xD7}, 4, 0}, // Column Address Set: 0 to 469 (0x0000 to 0x01D5) - 增加几个像素以覆盖全屏
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xD1}, 4, 0}, // Page Address Set: 0 to 465 (0x0000 to 0x01D1)
    {0x11, (uint8_t[]){0x00}, 0, 60},                  // Sleep out + 60ms delay
    {0x29, (uint8_t[]){0x00}, 0, 0},                   // Display on
};

class MovecallMojiESP32S3 : public WifiBoard {
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
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_handle_ = nullptr;
    bool do_calibration_ = false;
    uint32_t battery_level_ = 0;
    
    // BMI270陀螺仪相关（官方驱动）
    bmi270_handle_t bmi_handle_ = NULL;
    i2c_master_dev_handle_t bmi_idf_dev_ = NULL;
    esp_timer_handle_t gyro_timer_;
    struct { struct { float x, y, z; } accel; struct { float x, y, z; } gyro; } last_sensor_data_{};
    
    // 晃动检测需要存储所有三个轴的陀螺仪数据
    std::vector<float> gyro_x_buffer;
    std::vector<float> gyro_y_buffer;
    bool gyro_initialized_ = false;
    esp_timer_handle_t bmi270_init_timer_ = nullptr;
    bool motor_uart_initialized_ = false;

    // 设备放置状态
    enum PlacementState {
        kPlacementIndependent = 0,   // 独立放置（未在底座上）
        kPlacementRotatingBase = 1,  // 放在旋转底座上
        kPlacementStaticBase   = 2   // 放在普通底座上（预留）
    };
    PlacementState placement_state_ = kPlacementIndependent;
    TaskHandle_t base_probe_task_handle_ = nullptr;

    // 车辆姿态检测相关
    struct {
        // 数据积累缓冲区
        std::vector<float> accel_x_buffer;
        std::vector<float> accel_y_buffer;
        std::vector<float> gyro_z_buffer;  // 改为Z轴陀螺仪
        const int BUFFER_SIZE = 10;  // 积累10个数据点
        
        // 滤波结果
        float accel_x_filtered = 0.0f;
        float accel_y_filtered = 0.0f;
        float gyro_z_filtered = 0.0f;  // 改为Z轴陀螺仪
        
        int64_t last_animation_time = 0;
        const int64_t ANIMATION_COOLDOWN_US = 3000 * 1000;  // 3秒冷却时间，防止重复触发
        int64_t init_start_time = 0;  // 初始化开始时间
        const int64_t INIT_STABILIZATION_US = 5000 * 1000;  // 5秒稳定时间
        bool detection_enabled = false;  // 检测是否启用
        
        // 定时器切换表情相关
        esp_timer_handle_t emoji_switch_timer_ = nullptr;  // 表情切换定时器
        int default_emoji_id_ = MMAP_MOJI_EMOJI_WINKING_AAF;  // 默认表情ID
        const int64_t ANIMATION_PLAY_DURATION_US = 4000 * 1000;  // 动画播放持续时间：4秒
        
        // 晃动检测相关
        int64_t shake_start_time = 0;  // 晃动开始时间
        int64_t last_shake_animation_time = 0;  // 上次晃动动画时间
        const int64_t SHAKE_DETECTION_DURATION_US = 3000 * 1000;  // 3秒晃动检测时间
        const int64_t SHAKE_COOLDOWN_US = 5000 * 1000;  // 5秒晃动冷却时间
        const float SHAKE_THRESHOLD = 15.0f;  // 晃动检测阈值 (deg/s)
        bool is_shaking = false;  // 是否正在晃动
        
        // IDLE状态表情轮播相关
        esp_timer_handle_t idle_emoji_rotation_timer_ = nullptr;  // IDLE表情轮播定时器
        const int64_t IDLE_EMOJI_ROTATION_INTERVAL_US = 30 * 1000 * 1000;  // 30秒轮播间隔
        int idle_emoji_index_ = 0;  // 当前轮播表情索引
        bool is_playing_rotation_emoji_ = false;  // 是否正在播放列表中的表情
        const int idle_emoji_list_[8] = {
            MMAP_MOJI_EMOJI_DELICIOUS_AAF,    // 0
            MMAP_MOJI_EMOJI_HAPPY_AAF,       // 1
            MMAP_MOJI_EMOJI_CONFIDENT_AAF,  // 2
            MMAP_MOJI_EMOJI_SAYHELLO_AAF,       // 3
            MMAP_MOJI_EMOJI_LOOKAROUND_AAF, // 4
            MMAP_MOJI_EMOJI_WINKING_AAF,    // 5
            MMAP_MOJI_EMOJI_YAWNING_AAF,    // 6
            MMAP_MOJI_EMOJI_COMFORT_AAF     // 7
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

    static void TouchpadTimerCallback(void* arg) {
        MovecallMojiESP32S3* board = (MovecallMojiESP32S3*)arg;
        board->PollTouchpad();
    }
    
    static void TouchEventTask(void* arg) {
        MovecallMojiESP32S3* board = (MovecallMojiESP32S3*)arg;
        board->HandleTouchEvents();
    }
    
    static void EmojiSwitchTimerCallback(void* arg) {
        MovecallMojiESP32S3* board = (MovecallMojiESP32S3*)arg;
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
                        widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_DEFAULT_AAF, true, 2);
                        ESP_LOGI("MovecallMojiESP32S3", "Switched back to DEFAULT emoji in IDLE state");
                        break;
                    case kDeviceStateListening:
                    case kDeviceStateSpeaking:
                    default:
                        // 停止IDLE表情轮播
                        board->StopIdleEmojiRotation();
                        widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_LISTENING_AAF, true, 2);
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
    
    static void GyroTimerCallback(void* arg) {
        MovecallMojiESP32S3* board = (MovecallMojiESP32S3*)arg;
        board->PollGyroscope();
    }

    // 底座探测任务：每秒发送一次 "L0" 指令，并读取串口返回
    static void BaseProbeTask(void* arg) {
        MovecallMojiESP32S3* board = static_cast<MovecallMojiESP32S3*>(arg);
        const TickType_t delay = pdMS_TO_TICKS(1000);
        for (;;) {
            // 确保UART已初始化
            if (!board->motor_uart_initialized_) {
                board->InitializeMotorUart();
            }
            if (board->motor_uart_initialized_) {
                // 向左步进0步用于探测
                board->ControlMotor('L', 0);

                // 读取一段时间内的返回
                uint8_t buf[128];
                int len = uart_read_bytes(MOJI_UART_PORT_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(120));
                if (len > 0) {
                    buf[len] = 0;
                    bool on_rotating = (strstr((const char*)buf, "step") != nullptr);
                    if (on_rotating) {
                        if (board->placement_state_ != kPlacementRotatingBase) {
                            ESP_LOGI(TAG, "Detected rotating base (uart contains 'step')");
                            board->SetPlacementState(kPlacementRotatingBase);
                        }
                    } else {
                        if (board->placement_state_ != kPlacementIndependent) {
                            ESP_LOGI(TAG, "No 'step' found in uart response, switch to independent");
                            board->SetPlacementState(kPlacementIndependent);
                        }
                    }
                } else {
                    // 没有数据也认为不在旋转底座
                    if (board->placement_state_ != kPlacementIndependent) {
                        ESP_LOGI(TAG, "No uart response, switch to independent");
                        board->SetPlacementState(kPlacementIndependent);
                    }
                }
            }
            vTaskDelay(delay);
        }
    }

    void SetPlacementState(PlacementState new_state) {
        if (placement_state_ == new_state) return;
        PlacementState old_state = placement_state_;
        placement_state_ = new_state;
        switch (placement_state_) {
            case kPlacementIndependent:
                ESP_LOGI(TAG, "Placement state -> Independent");
                // 独立状态下不进行车辆动作检测
                vehicle_motion_state_.detection_enabled = false;
                // 从旋转底座返回独立 → 播放 loving + popup
                if (old_state == kPlacementRotatingBase) {
                    if (display_) {
                        auto widget = static_cast<moji_anim::EmojiWidget*>(display_);
                        if (widget && widget->GetPlayer()) {
                            widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_LOVING_AAF, true, 2);
                        }
                    }
                    PlayLocalPrompt(Lang::Sounds::P3_POPUP, 2000000); // 2秒后关闭，配合LOVING动画
                }
                break;
            case kPlacementRotatingBase:
                ESP_LOGI(TAG, "Placement state -> RotatingBase");
                // 允许车辆动作检测（稳定期结束后会自动启用）
                // 不直接置true，仍由稳定期逻辑控制
                // 从独立进入旋转底座 → 播放 safebelt + powerup
                if (old_state == kPlacementIndependent) {
                    // 执行电机复位命令
                    ESP_LOGI(TAG, "Executing motor reset command on rotating base detection");
                    ResetMotor();
                    
                    if (display_) {
                        auto widget = static_cast<moji_anim::EmojiWidget*>(display_);
                        if (widget && widget->GetPlayer()) {
                            widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_SAFEBELT_AAF, true, 2);
                        }
                    }
                    PlayLocalPrompt(Lang::Sounds::P3_POWERUP, 2000000); // 2秒后关闭，配合SAFEBELT动画
                }
                break;
            case kPlacementStaticBase:
                ESP_LOGI(TAG, "Placement state -> StaticBase");
                // 预留：根据需要调整检测策略
                break;
        }
    }
    struct VehicleFeedbackCtx {
        MovecallMojiESP32S3* board;
        int aaf_id;
        const std::string_view* sound;
        esp_timer_handle_t timer;
    };
    static void VehicleFeedbackTimerCallback(void* arg) {
        auto ctx = (VehicleFeedbackCtx*)arg;
        if (ctx && ctx->board) {
            ctx->board->PlayTimedEmoji(ctx->aaf_id);
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
    void ScheduleVehicleFeedback(int aaf_id, const std::string_view& sound) {
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
        MovecallMojiESP32S3* board = (MovecallMojiESP32S3*)arg;
        if (board) {
            board->InitializeBmi270();
        }
    }

    static void IdleEmojiRotationTimerCallback(void* arg) {
        MovecallMojiESP32S3* board = (MovecallMojiESP32S3*)arg;
        if (board && board->display_) {
            // 检查设备是否仍处于IDLE状态
            DeviceState current_state = Application::GetInstance().GetDeviceState();
            if (current_state == kDeviceStateIdle) {
                // 获取列表中的下一个表情
                int next_emoji = board->vehicle_motion_state_.idle_emoji_list_[board->vehicle_motion_state_.idle_emoji_index_];
                
                ESP_LOGI(TAG, "IDLE emoji rotation: playing emoji %d (index %d) for 2 seconds", 
                        next_emoji, board->vehicle_motion_state_.idle_emoji_index_);
                
                // 使用PlayTimedEmoji播放表情2秒钟
                board->PlayTimedEmoji(next_emoji);
                
                // 将索引移动到下一个表情
                board->vehicle_motion_state_.idle_emoji_index_ = 
                    (board->vehicle_motion_state_.idle_emoji_index_ + 1) % 8;
                
                // 启动定时器，30秒后切换到下一个表情
                esp_timer_start_once(board->vehicle_motion_state_.idle_emoji_rotation_timer_, 
                                   board->vehicle_motion_state_.IDLE_EMOJI_ROTATION_INTERVAL_US);
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
        widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_DEFAULT_AAF, true, 2);
        vehicle_motion_state_.is_playing_rotation_emoji_ = false;
        
        ESP_LOGI(TAG, "Rotation emoji playback complete, switched back to DEFAULT emoji");
        
        // 将索引移动到下一个表情
        vehicle_motion_state_.idle_emoji_index_ = 
            (vehicle_motion_state_.idle_emoji_index_ + 1) % 8;
        
        // 启动定时器，60秒后切换到下一个表情
        esp_timer_start_once(vehicle_motion_state_.idle_emoji_rotation_timer_, 
                           vehicle_motion_state_.IDLE_EMOJI_ROTATION_INTERVAL_US);
    }
    void InitializeMotorUart() {
        if (motor_uart_initialized_) return;
        uart_config_t uart_config = {
            .baud_rate = MOJI_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        int intr_alloc_flags = 0;
        if (uart_driver_install(MOJI_UART_PORT_NUM, 1024, 0, 0, NULL, intr_alloc_flags) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install UART driver for motor");
            return;
        }
        if (uart_param_config(MOJI_UART_PORT_NUM, &uart_config) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to config UART for motor");
            return;
        }
        if (uart_set_pin(MOJI_UART_PORT_NUM, MOJI_UART_TXD, MOJI_UART_RXD, MOJI_UART_RTS, MOJI_UART_CTS) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set UART pins for motor");
            return;
        }
        motor_uart_initialized_ = true;
    }

    void SendMotorCommand(const char* cmd) {
        if (!motor_uart_initialized_) InitializeMotorUart();
        if (!motor_uart_initialized_) return;
        size_t len = strlen(cmd);
        uart_write_bytes(MOJI_UART_PORT_NUM, cmd, len);
        ESP_LOGI(TAG, "Motor cmd: %s", cmd);
    }

    void ControlMotor(char direction, int steps) {
        // direction: 'L' or 'R'
        char buffer[16];
        int n = snprintf(buffer, sizeof(buffer), "%c%d\r\n", direction, steps);
        if (n > 0) {
            SendMotorCommand(buffer);
        }
    }

    void ResetMotor() {
        SendMotorCommand("X\r\n");
    }

    // 本地提示音播放：在 Idle 下短暂开启输出，播放完成后自动关闭
    static void SoundDisableOutTimerCb(void* arg) {
        auto* board = static_cast<MovecallMojiESP32S3*>(arg);
        if (!board) return;
        auto state = Application::GetInstance().GetDeviceState();
        if (state == kDeviceStateIdle) {
            // 清空音频队列并关闭输出，确保不影响下一次播放
            Application::GetInstance().ClearAudioQueueAndDisableOutput();
            ESP_LOGI(TAG, "Vehicle motion sound completed, cleared audio queue and disabled output");
        }
    }
    
    void PlayLocalPrompt(const std::string_view& sound, int64_t disable_after_us = 2000000) {
        // 清空音频队列并重置解码器状态，然后重新启用输出
        Application::GetInstance().ClearAudioQueueAndDisableOutput();
        auto codec = GetAudioCodec();
        if (codec) {
            codec->EnableOutput(true);
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
                        if (is_playing_animation_) {
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
                                is_playing_animation_ = true;
                                
                                ESP_LOGI(TAG, "Playing shocked emoji...");
                                widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_KNOCKING_AAF, false, 4);
                                // 使用定时器延迟切换表情，避免阻塞事件处理
                                if (emoji_switch_timer_ == nullptr) {
                                    esp_timer_create_args_t timer_args = {
                                        .callback = EmojiSwitchTimerCallback,
                                        .arg = this,
                                        .dispatch_method = ESP_TIMER_TASK,
                                        .name = "emoji_switch_timer",
                                        .skip_unhandled_events = true,
                                    };
                                    esp_timer_create(&timer_args, &emoji_switch_timer_);
                                }
                                esp_timer_stop(emoji_switch_timer_);
                                esp_timer_start_once(emoji_switch_timer_, 1000 * 1000); // 1秒后切换
                                ESP_LOGI(TAG, "Shocked emoji played, timer set for emoji switch");
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
    
    static inline float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width) {
        double power = 2;
        float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
        return (9.80665f * val * g_range) / half_scale;
    }
    static inline float lsb_to_dps(int16_t val, float dps, uint8_t bit_width) {
        double power = 2;
        float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
        return (dps / half_scale) * val;
    }
    void PollGyroscope() {
        if (!gyro_initialized_ || bmi_handle_ == NULL) {
            return;
        }
        struct bmi2_sens_data data = {};
        struct bmi2_dev *dev = (struct bmi2_dev*)bmi_handle_;
        int8_t rslt = bmi2_get_sensor_data(&data, dev);
        if (rslt != 0) {
            ESP_LOGW(TAG, "Failed to read BMI270 data, rslt=%d", (int)rslt);
            return;
        }
        if (!((data.status & BMI2_DRDY_ACC) && (data.status & BMI2_DRDY_GYR))) {
            return;
        }
        last_sensor_data_.accel.x = lsb_to_mps2(data.acc.x, 2.0f, 16);
        last_sensor_data_.accel.y = lsb_to_mps2(data.acc.y, 2.0f, 16);
        last_sensor_data_.accel.z = lsb_to_mps2(data.acc.z, 2.0f, 16);
        last_sensor_data_.gyro.x = lsb_to_dps(data.gyr.x, 2000.0f, 16);
        last_sensor_data_.gyro.y = lsb_to_dps(data.gyr.y, 2000.0f, 16);
        last_sensor_data_.gyro.z = lsb_to_dps(data.gyr.z, 2000.0f, 16);
        
        // 处理车辆姿态检测
        ProcessVehicleMotion();
        
        // 处理晃动检测（不受放置状态限制）
        ProcessShakeDetection();
        
        // 通用运动检测已删除，只保留车辆姿态检测
        // 周期性调试信息已注释掉
        // static int debug_counter = 0;
        // if (++debug_counter >= 50) {
        //     ESP_LOGI(TAG, "BMI270 - Accel: %.2f,%.2f,%.2f | Gyro: %.2f,%.2f,%.2f",
        //             last_sensor_data_.accel.x, last_sensor_data_.accel.y, last_sensor_data_.accel.z,
        //             last_sensor_data_.gyro.x, last_sensor_data_.gyro.y, last_sensor_data_.gyro.z);
        //     ESP_LOGI(TAG, "Vehicle Motion - Filtered Accel: %.3f,%.3f g | Filtered Gyro: %.1f deg/s",
        //             vehicle_motion_state_.accel_x_filtered / 9.80665f, 
        //             vehicle_motion_state_.accel_y_filtered / 9.80665f,
        //             vehicle_motion_state_.gyro_magnitude_filtered);
        //     debug_counter = 0;
        // }
    }

    // 车辆姿态检测相关函数
    void ProcessVehicleMotion() {
        int64_t current_time = esp_timer_get_time();
        // 独立状态下不进行车辆动作检测
        if (placement_state_ == kPlacementIndependent) {
            return;
        }
        
        // 设置初始化开始时间（仅在第一次调用时设置）
        if (vehicle_motion_state_.init_start_time == 0) {
            vehicle_motion_state_.init_start_time = current_time;
            ESP_LOGI(TAG, "Vehicle motion detection starting, stabilization period: 5 seconds");
        }
        
        // 检查是否还在稳定期内
        if (current_time - vehicle_motion_state_.init_start_time < vehicle_motion_state_.INIT_STABILIZATION_US) {
            // 稳定期内只进行数据积累，不进行检测
            vehicle_motion_state_.accel_x_buffer.push_back(last_sensor_data_.accel.x);
            vehicle_motion_state_.accel_y_buffer.push_back(last_sensor_data_.accel.y);
            vehicle_motion_state_.gyro_z_buffer.push_back(last_sensor_data_.gyro.z);  // 使用Z轴陀螺仪
            
            // 为晃动检测添加X和Y轴陀螺仪数据
            gyro_x_buffer.push_back(last_sensor_data_.gyro.x);
            gyro_y_buffer.push_back(last_sensor_data_.gyro.y);
            
            // 保持缓冲区大小
            if (vehicle_motion_state_.accel_x_buffer.size() > vehicle_motion_state_.BUFFER_SIZE) {
                vehicle_motion_state_.accel_x_buffer.erase(vehicle_motion_state_.accel_x_buffer.begin());
                vehicle_motion_state_.accel_y_buffer.erase(vehicle_motion_state_.accel_y_buffer.begin());
                vehicle_motion_state_.gyro_z_buffer.erase(vehicle_motion_state_.gyro_z_buffer.begin());
            }
            
            // 保持晃动检测缓冲区大小
            if (gyro_x_buffer.size() > vehicle_motion_state_.BUFFER_SIZE) {
                gyro_x_buffer.erase(gyro_x_buffer.begin());
                gyro_y_buffer.erase(gyro_y_buffer.begin());
            }
            return;
        }
        
        // 稳定期结束后启用检测
        if (!vehicle_motion_state_.detection_enabled) {
            vehicle_motion_state_.detection_enabled = true;
            ESP_LOGI(TAG, "Vehicle motion detection enabled after stabilization period");
        }
        
        // 将数据添加到缓冲区
        vehicle_motion_state_.accel_x_buffer.push_back(last_sensor_data_.accel.x);
        vehicle_motion_state_.accel_y_buffer.push_back(last_sensor_data_.accel.y);
        vehicle_motion_state_.gyro_z_buffer.push_back(last_sensor_data_.gyro.z);  // 使用Z轴陀螺仪
        
        // 为晃动检测添加X和Y轴陀螺仪数据
        gyro_x_buffer.push_back(last_sensor_data_.gyro.x);
        gyro_y_buffer.push_back(last_sensor_data_.gyro.y);
        
        // 保持缓冲区大小
        if (vehicle_motion_state_.accel_x_buffer.size() > vehicle_motion_state_.BUFFER_SIZE) {
            vehicle_motion_state_.accel_x_buffer.erase(vehicle_motion_state_.accel_x_buffer.begin());
            vehicle_motion_state_.accel_y_buffer.erase(vehicle_motion_state_.accel_y_buffer.begin());
            vehicle_motion_state_.gyro_z_buffer.erase(vehicle_motion_state_.gyro_z_buffer.begin());
        }
        
        // 保持晃动检测缓冲区大小
        if (gyro_x_buffer.size() > vehicle_motion_state_.BUFFER_SIZE) {
            gyro_x_buffer.erase(gyro_x_buffer.begin());
            gyro_y_buffer.erase(gyro_y_buffer.begin());
        }
        
        // 只有当缓冲区满了才进行滤波和检测
        if (vehicle_motion_state_.accel_x_buffer.size() >= vehicle_motion_state_.BUFFER_SIZE) {
            // 计算缓冲区内的平均值作为滤波结果
            float sum_x = 0.0f, sum_y = 0.0f, sum_gz = 0.0f;
            for (int i = 0; i < vehicle_motion_state_.BUFFER_SIZE; i++) {
                sum_x += vehicle_motion_state_.accel_x_buffer[i];
                sum_y += vehicle_motion_state_.accel_y_buffer[i];
                sum_gz += vehicle_motion_state_.gyro_z_buffer[i];
            }
            
            vehicle_motion_state_.accel_x_filtered = sum_x / vehicle_motion_state_.BUFFER_SIZE;
            vehicle_motion_state_.accel_y_filtered = sum_y / vehicle_motion_state_.BUFFER_SIZE;
            vehicle_motion_state_.gyro_z_filtered = sum_gz / vehicle_motion_state_.BUFFER_SIZE;
            
            // 检查动画冷却时间
            if (current_time - vehicle_motion_state_.last_animation_time < vehicle_motion_state_.ANIMATION_COOLDOWN_US) {
                return;
            }
            
            // 检测车辆姿态（只有在缓冲区满了才检测）
            DetectVehiclePosture(current_time);
        }
    }

    void DetectVehiclePosture(int64_t current_time) {
        // 如果正在播放动画和声音，不响应新的车辆动作检测
        if (is_playing_animation_) {
            ESP_LOGD(TAG, "Animation playing, skipping vehicle posture detection");
            return;
        }
        
        const float ACCEL_X_THRESHOLD_ACCEL = 0.3f * 9.80665f;  // 0.3g in m/s² (加速)
        const float ACCEL_X_THRESHOLD_BRAKE = 0.6f * 9.80665f;  // 0.6g in m/s² (刹车)
        const float GYRO_Z_THRESHOLD = 10.0f;  // 10°/s (Z轴角速度阈值)
        
        // 检测加速 (根据坐标系修正：ax < -阈值)
        if (vehicle_motion_state_.accel_x_filtered < -ACCEL_X_THRESHOLD_ACCEL) {
            ESP_LOGI(TAG, "Vehicle accelerating: ax=%.3f g (neg axis)", vehicle_motion_state_.accel_x_filtered / 9.80665f);
            // 仅在 Idle 下触发，不切换设备状态
            if (Application::GetInstance().GetDeviceState() == kDeviceStateIdle) {
                PlayTimedEmoji(MMAP_MOJI_EMOJI_SPEEDING_AAF);
                // 声音早于动画结束：略短于动画时长
                PlayLocalPrompt(Lang::Sounds::P3_SPEEDING, vehicle_motion_state_.ANIMATION_PLAY_DURATION_US - 100000);
            }
            return;
        }
        
        // 检测刹车 (根据坐标系修正：ax > +阈值)
        if (vehicle_motion_state_.accel_x_filtered > ACCEL_X_THRESHOLD_BRAKE) {
            ESP_LOGI(TAG, "Vehicle braking: ax=%.3f g (pos axis)", vehicle_motion_state_.accel_x_filtered / 9.80665f);
            if (Application::GetInstance().GetDeviceState() == kDeviceStateIdle) {
                PlayTimedEmoji(MMAP_MOJI_EMOJI_BRAKING_AAF);
                // 声音早于动画结束：略短于动画时长
                PlayLocalPrompt(Lang::Sounds::P3_BRAKING, vehicle_motion_state_.ANIMATION_PLAY_DURATION_US - 100000);
            }
            return;
        }
        
        // 检测左转 (gyro_z > 10°/s) - 使用陀螺仪Z轴角速度
        if (vehicle_motion_state_.gyro_z_filtered > GYRO_Z_THRESHOLD) {
            ESP_LOGI(TAG, "Vehicle turning left: gz=%.1f deg/s", vehicle_motion_state_.gyro_z_filtered);
            if (Application::GetInstance().GetDeviceState() == kDeviceStateIdle) {
                PlayTimedEmoji(MMAP_MOJI_EMOJI_TURNLEFT_AAF);
                // 声音早于动画结束：略短于动画时长
                PlayLocalPrompt(Lang::Sounds::P3_TURN, vehicle_motion_state_.ANIMATION_PLAY_DURATION_US - 100000);
            }
            return;
        }
        
        // 检测右转 (gyro_z < -10°/s) - 使用陀螺仪Z轴角速度
        if (vehicle_motion_state_.gyro_z_filtered < -GYRO_Z_THRESHOLD) {
            ESP_LOGI(TAG, "Vehicle turning right: gz=%.1f deg/s", vehicle_motion_state_.gyro_z_filtered);
            if (Application::GetInstance().GetDeviceState() == kDeviceStateIdle) {
                PlayTimedEmoji(MMAP_MOJI_EMOJI_TURNRIGHT_AAF);
                // 声音早于动画结束：略短于动画时长
                PlayLocalPrompt(Lang::Sounds::P3_TURN, vehicle_motion_state_.ANIMATION_PLAY_DURATION_US - 100000);
            }
            return;
        }
        
        // 堵车检测功能已删除
    }

    void DetectShake(int64_t current_time) {
        // 如果正在播放动画和声音，不响应新的晃动检测
        if (is_playing_animation_) {
            ESP_LOGD(TAG, "Animation playing, skipping shake detection");
            return;
        }
        
        // 检查晃动冷却时间
        if (current_time - vehicle_motion_state_.last_shake_animation_time < vehicle_motion_state_.SHAKE_COOLDOWN_US) {
            return;
        }
        
        // 计算所有三个轴的陀螺仪数据的总幅度（晃动强度）
        float gyro_x = last_sensor_data_.gyro.x;
        float gyro_y = last_sensor_data_.gyro.y;
        float gyro_z = last_sensor_data_.gyro.z;
        
        // 计算晃动强度（所有三个轴的总幅度）
        float shake_intensity = sqrt(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
        
        // 检查是否超过晃动阈值
        if (shake_intensity > vehicle_motion_state_.SHAKE_THRESHOLD) {
            if (!vehicle_motion_state_.is_shaking) {
                // 开始晃动
                vehicle_motion_state_.is_shaking = true;
                vehicle_motion_state_.shake_start_time = current_time;
                ESP_LOGI(TAG, "Independent shake started: intensity=%.1f deg/s", shake_intensity);
            } else {
                // 持续晃动，检查是否超过3秒
                int64_t shake_duration = current_time - vehicle_motion_state_.shake_start_time;
                if (shake_duration >= vehicle_motion_state_.SHAKE_DETECTION_DURATION_US) {
                    ESP_LOGI(TAG, "Independent shake detected for %.1f seconds, intensity=%.1f deg/s", 
                            shake_duration / 1000000.0f, shake_intensity);
                    
                    // 仅在 Idle 下触发，不切换设备状态
                    if (Application::GetInstance().GetDeviceState() == kDeviceStateIdle) {
                        PlayTimedEmoji(MMAP_MOJI_EMOJI_DIZZY_AAF);
                        // 播放晃动音效，延迟4秒关闭
                        PlayLocalPrompt(Lang::Sounds::P3_VIBRATION, vehicle_motion_state_.ANIMATION_PLAY_DURATION_US - 100000);
                        
                        // 更新晃动动画时间
                        vehicle_motion_state_.last_shake_animation_time = current_time;
                    }
                    
                    // 重置晃动状态
                    vehicle_motion_state_.is_shaking = false;
                    vehicle_motion_state_.shake_start_time = 0;
                }
            }
        } else {
            // 晃动强度不够，重置晃动状态
            if (vehicle_motion_state_.is_shaking) {
                ESP_LOGD(TAG, "Independent shake stopped: intensity=%.1f deg/s", shake_intensity);
                vehicle_motion_state_.is_shaking = false;
                vehicle_motion_state_.shake_start_time = 0;
            }
        }
    }

    // 统一的表情播放（定时自动恢复），供车辆检测与放置状态切换复用
    void PlayTimedEmoji(int aaf_id) {
        if (!display_) {
            ESP_LOGW(TAG, "Display not available for vehicle animation");
            return;
        }
        
        auto widget = static_cast<moji_anim::EmojiWidget*>(display_);
        if (!widget || !widget->GetPlayer()) {
            ESP_LOGW(TAG, "Emoji widget or player not available");
            return;
        }
        
        // 设置播放状态标志，防止重复触发
        is_playing_animation_ = true;
        
        // 更新动画播放时间
        vehicle_motion_state_.last_animation_time = esp_timer_get_time();
        
        // 播放动画（循环播放）
        widget->GetPlayer()->StartPlayer(aaf_id, true, 2);
        ESP_LOGI(TAG, "Playing emoji animation: %d (loop play)", aaf_id);
        
        // 启动定时器，2秒后切换回默认表情
        if (vehicle_motion_state_.emoji_switch_timer_ == nullptr) {
            esp_timer_create_args_t timer_args = {
                .callback = EmojiSwitchTimerCallback,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "emoji_switch_timer",
                .skip_unhandled_events = true,
            };
            esp_timer_create(&timer_args, &vehicle_motion_state_.emoji_switch_timer_);
        }
        
        // 停止之前的定时器并启动新的
        esp_timer_stop(vehicle_motion_state_.emoji_switch_timer_);
        esp_timer_start_once(vehicle_motion_state_.emoji_switch_timer_, 2 * 1000 * 1000); // 2秒后切换
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
    
    void InitializeBmi270() {
        ESP_LOGI(TAG, "Initializing BMI270 gyroscope (official driver)...");

        // 复用已存在的 IDF I2C master 总线 (codec_i2c_bus_)，为 BMI270 创建设备句柄
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BMI270_I2C_ADDR,
            .scl_speed_hz = BMI270_I2C_FREQ_HZ,
        };
        esp_err_t err = i2c_master_bus_add_device(codec_i2c_bus_, &dev_cfg, &bmi_idf_dev_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add BMI270 device to existing I2C bus: %s", esp_err_to_name(err));
            return;
        }

        // 用 BMI2 低层 API 适配 IDF 的 i2c_master 接口
        static struct bmi2_dev bmi2_dev_ctx = {};
        bmi2_dev_ctx = {};
        bmi2_dev_ctx.intf = BMI2_I2C_INTF;
        bmi2_dev_ctx.read = [](uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) -> int8_t {
            i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
            esp_err_t e = i2c_master_transmit_receive(dev, &reg_addr, 1, data, len, 1000);
            return (e == ESP_OK) ? 0 : -1;
        };
        bmi2_dev_ctx.write = [](uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) -> int8_t {
            i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
            uint8_t buf[1 + 32];
            if (len <= 32) {
                buf[0] = reg_addr;
                memcpy(&buf[1], data, len);
                esp_err_t e = i2c_master_transmit(dev, buf, 1 + len, 1000);
                return (e == ESP_OK) ? 0 : -1;
            }
            uint8_t *dyn = (uint8_t*)malloc(1 + len);
            if (!dyn) return -1;
            dyn[0] = reg_addr;
            memcpy(&dyn[1], data, len);
            esp_err_t e = i2c_master_transmit(dev, dyn, 1 + len, 1000);
            free(dyn);
            return (e == ESP_OK) ? 0 : -1;
        };
        bmi2_dev_ctx.delay_us = [](uint32_t period, void * /*intf_ptr*/) { esp_rom_delay_us(period); };
        bmi2_dev_ctx.intf_ptr = (void*)bmi_idf_dev_;
        bmi2_dev_ctx.read_write_len = 32;
        bmi2_dev_ctx.config_file_ptr = NULL;

        int8_t rslt = bmi270_init(&bmi2_dev_ctx);
        if (rslt != 0) {
            ESP_LOGE(TAG, "bmi270_init failed: %d", (int)rslt);
            return;
        }
        // 保存句柄供后续调用
        bmi_handle_ = (bmi270_handle_t)&bmi2_dev_ctx;
        
        struct bmi2_sens_config config[2];
        config[0].type = BMI2_ACCEL;
        config[1].type = BMI2_GYRO;
        // 从 bmi2_dev 获取配置
        struct bmi2_dev *dev = (struct bmi2_dev*)bmi_handle_;
        rslt = bmi2_get_sensor_config(config, 2, dev);
        if (rslt != 0) {
            ESP_LOGE(TAG, "bmi2_get_sensor_config failed: %d", (int)rslt);
            return;
        }

        config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
        config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
        config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
        config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
        config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
        config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
        config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
        config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
        config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        rslt = bmi2_set_sensor_config(config, 2, dev);
        if (rslt != 0) {
            ESP_LOGE(TAG, "bmi2_set_sensor_config failed: %d", (int)rslt);
            return;
        }

        uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
        rslt = bmi2_sensor_enable(sensor_list, 2, dev);
        if (rslt != 0) {
            ESP_LOGE(TAG, "bmi2_sensor_enable failed: %d", (int)rslt);
            return;
        }

        esp_timer_create_args_t gyro_timer_args = {
            .callback = GyroTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "gyro_timer",
            .skip_unhandled_events = true,
        };
        err = esp_timer_create(&gyro_timer_args, &gyro_timer_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create gyro timer: %s", esp_err_to_name(err));
            return;
        }
        err = esp_timer_start_periodic(gyro_timer_, 10 * 1000);  // 改为10ms轮询，提高检测精度
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start gyro timer: %s", esp_err_to_name(err));
            esp_timer_delete(gyro_timer_);
            return;
        }
        
        gyro_initialized_ = true;
        ESP_LOGI(TAG, "BMI270 gyroscope initialized successfully (official)");
        ESP_LOGI(TAG, "Vehicle motion detection configured with thresholds:");
        ESP_LOGI(TAG, "  - Acceleration X: 0.3g (accelerate) / 0.6g (brake) - Rotating base only");
        ESP_LOGI(TAG, "  - Gyroscope Z: ±10°/s (left/right turn) - Rotating base only");
        ESP_LOGI(TAG, "  - Shake detection: 15°/s threshold, 3s duration - All placement states");
        ESP_LOGI(TAG, "  - Animation cooldown: 3 seconds");
        ESP_LOGI(TAG, "  - Shake cooldown: 5 seconds");
        ESP_LOGI(TAG, "  - Animation mode: Loop play with 4s timer switch");
        ESP_LOGI(TAG, "  - IDLE emoji rotation: 60s interval, 8 emojis");
        ESP_LOGI(TAG, "  - Base detection: SAFEBELT emoji + POWERUP sound");
        ESP_LOGI(TAG, "  - Sampling rate: 200Hz, Polling interval: 10ms");
        ESP_LOGI(TAG, "  - Stabilization period: 5 seconds (detection disabled during startup)");
        ESP_LOGI(TAG, "  - Data buffer: 10 samples (100ms window)");
    }

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

        display_ = new moji_anim::EmojiWidget(panel_handle, io_handle);
        
        // 保存panel handle用于亮度控制
        panel_handle_ = panel_handle;
        
        // 创建CO5300背光控制对象
        backlight_ = std::make_unique<Co5300Backlight>(panel_handle);
        if (backlight_) {
            backlight_->RestoreBrightness();
        }
        
        // 启动IDLE状态的表情轮播
        ESP_LOGI(TAG, "Starting IDLE emoji rotation after display initialization");
        StartIdleEmojiRotation();
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
        ESP_LOGI(TAG, "ADC value: %d", adc_value);

        int voltage = 0; // mV
        
        if (do_calibration_) {
            // 使用校准转换
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle_, adc_value, &voltage));
            ESP_LOGI(TAG, "Calibrated voltage: %d mV", voltage);
        } else {
            // 使用原始值（简单转换）
            voltage = (adc_value * 2500) / 4095;
            ESP_LOGI(TAG, "Raw voltage: %d mV", voltage);
        }

        // 将实际电压乘以2得到调整后的电压
        uint32_t adjusted_voltage = voltage * 2;
        ESP_LOGI(TAG, "Adjusted voltage: %ld mV", adjusted_voltage);

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

        ESP_LOGI(TAG, "Battery level: %ld%%", battery_level_);
    }

public:
    MovecallMojiESP32S3() : boot_button_(BOOT_BUTTON_GPIO) {  
        InitializeBatteryAdc_ChrgStat();
        InitializeCodecI2c();
        InitializeSpi();
        InitializeVciEn();
        InitializeGc9a01Display();
        //播放开机音效，临时放在这里，后续优化
        Application::GetInstance().PlaySound(Lang::Sounds::P3_POWERUP);
        InitializeTouchPad();  // 初始化触摸传感器（非触摸屏触控）
        InitializeButtons();
        InitializeIot();
        InitializeMotorUart();

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

        // 启动底座探测任务
        BaseType_t rt = xTaskCreate(BaseProbeTask, "base_probe_task", 3072, this, 1, &base_probe_task_handle_);
        if (rt != pdPASS) {
            ESP_LOGE(TAG, "Failed to create base probe task");
        }
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
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        ReadBatteryAdcData();
        level = battery_level_;
        
        // 低电平表示正在充电，高电平或高阻态（Hi-Z）表示充电完成
        bool charging_pin_level = gpio_get_level(BATTERY_CHRG_STAT_GPIO);
        charging = !charging_pin_level;  // 低电平表示正在充电
        discharging = charging_pin_level;  // 高电平或高阻态表示充电完成（放电状态）
        ESP_LOGI(TAG, "Charging pin level: %d, charging: %s, discharging: %s (高电平或高阻态=充电完成)", 
                charging_pin_level, charging ? "true" : "false", discharging ? "true" : "false");
        
        return true;
    }

    // Moji motor MCP hooks
    void MojiControlMotor(char direction, int steps) override {
        if (direction != 'L' && direction != 'R') return;
        ControlMotor(direction, steps);
    }
    void MojiResetMotor() override {
        ResetMotor();
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
        widget->GetPlayer()->StartPlayer(MMAP_MOJI_EMOJI_DEFAULT_AAF, true, 2);
        
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

    // 独立的晃动检测函数（不受放置状态限制）
    void ProcessShakeDetection() {
        int64_t current_time = esp_timer_get_time();
        
        // 如果正在播放动画和声音，不响应新的晃动检测
        if (is_playing_animation_) {
            ESP_LOGD(TAG, "Animation playing, skipping independent shake detection");
            return;
        }
        
        // 检查晃动冷却时间
        if (current_time - vehicle_motion_state_.last_shake_animation_time < vehicle_motion_state_.SHAKE_COOLDOWN_US) {
            return;
        }
        
        // 计算所有三个轴的陀螺仪数据的总幅度（晃动强度）
        float gyro_x = last_sensor_data_.gyro.x;
        float gyro_y = last_sensor_data_.gyro.y;
        float gyro_z = last_sensor_data_.gyro.z;
        
        // 计算晃动强度（所有三个轴的总幅度）
        float shake_intensity = sqrt(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
        
        // 检查是否超过晃动阈值
        if (shake_intensity > vehicle_motion_state_.SHAKE_THRESHOLD) {
            if (!vehicle_motion_state_.is_shaking) {
                // 开始晃动
                vehicle_motion_state_.is_shaking = true;
                vehicle_motion_state_.shake_start_time = current_time;
                ESP_LOGI(TAG, "Independent shake started: intensity=%.1f deg/s", shake_intensity);
            } else {
                // 持续晃动，检查是否超过3秒
                int64_t shake_duration = current_time - vehicle_motion_state_.shake_start_time;
                if (shake_duration >= vehicle_motion_state_.SHAKE_DETECTION_DURATION_US) {
                    ESP_LOGI(TAG, "Independent shake detected for %.1f seconds, intensity=%.1f deg/s", 
                            shake_duration / 1000000.0f, shake_intensity);
                    
                    // 仅在 Idle 下触发，不切换设备状态
                    if (Application::GetInstance().GetDeviceState() == kDeviceStateIdle) {
                        PlayTimedEmoji(MMAP_MOJI_EMOJI_DIZZY_AAF);
                        // 播放晃动音效，延迟4秒关闭
                        PlayLocalPrompt(Lang::Sounds::P3_VIBRATION, vehicle_motion_state_.ANIMATION_PLAY_DURATION_US - 100000);
                        
                        // 更新晃动动画时间
                        vehicle_motion_state_.last_shake_animation_time = current_time;
                    }
                    
                    // 重置晃动状态
                    vehicle_motion_state_.is_shaking = false;
                    vehicle_motion_state_.shake_start_time = 0;
                }
            }
        } else {
            // 晃动强度不够，重置晃动状态
            if (vehicle_motion_state_.is_shaking) {
                ESP_LOGD(TAG, "Independent shake stopped: intensity=%.1f deg/s", shake_intensity);
                vehicle_motion_state_.is_shaking = false;
                vehicle_motion_state_.shake_start_time = 0;
            }
        }
    }
};

DECLARE_BOARD(MovecallMojiESP32S3);
