#include <cstring>
#include "display/lcd_display.h"
#include <esp_log.h>
//#include "mmap_generate_emoji.h"
#include "emoji_widget.h"
#include "mmap_generate_moji_emoji.h"
#include "config.h"
#include "assets/lang_config.h"
#include "board.h"
#include "application.h"
#include "font_awesome_symbols.h"

#include <functional>
#include <tuple>
#include <unordered_map>
#include <string>
#include <utility>

#include <esp_lcd_panel_io.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

// convert from rgb 0~1 to BRG565
// __G
// _R_
// B
#define COLOR(r, g, b) (uint16_t)(((uint8_t)(b*31) << 11) | ((uint8_t)(r*63) << 5) | ((uint8_t)(g*31)))

static const char *TAG = "moji_emoji";
#define EMOJI_FPS 10

namespace moji_anim {

bool EmojiPlayer::OnFlushIoReady(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    auto* disp_drv = static_cast<anim_player_handle_t*>(user_ctx);
    auto* self = static_cast<EmojiPlayer*>(anim_player_get_user_data(disp_drv));
    if (self) {
        self->transmit_busy_ = false;
    }
    anim_player_flush_ready(disp_drv);
    return true;
}

void EmojiPlayer::OnFlush(anim_player_handle_t handle, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    auto* self = static_cast<EmojiPlayer*>(anim_player_get_user_data(handle));
    auto* panel = self->panel_;

    // CO5300屏幕的显示窗口从列6开始（0x0006），所以需要在坐标上加上偏移
    // 初始化序列中0x2A设置为{0x00, 0x06, 0x01, 0xD7}，表示列地址6-471
    // 动画播放器传入的坐标是0-465，需要加上6的偏移使其变为6-471
    const int COLUMN_OFFSET = 6;

    // draw status points at the bottom center
    const int STATUS_POINT_RADIUS = 8;
    const int STATUS_POINT_SPACING = 24;
    // blue, orange, red
    // const uint16_t STATUS_POINT_COLORS[3] = {0x001F, 0x7BEF, 0xF800};

    // const uint16_t test_color[3] = {
    //     COLOR(1.0, 0.0, 0.0), // Red
    //     COLOR(0.0, 1.0, 0.0), // Green
    //     COLOR(0.0, 0.0, 1.0), // Blue
    // };


    for (int i = 0; i < 3; ++i) {
        int cx = (466 / 2) - STATUS_POINT_SPACING + i * STATUS_POINT_SPACING;
        int cy = 466 - STATUS_POINT_RADIUS - 12;
        for (int y = -STATUS_POINT_RADIUS; y <= STATUS_POINT_RADIUS; y++) {
            for (int x = -STATUS_POINT_RADIUS; x <= STATUS_POINT_RADIUS; x++) {
                if (x * x + y * y <= STATUS_POINT_RADIUS * STATUS_POINT_RADIUS) {
                    int draw_x = cx + x;
                    int draw_y = cy + y;
                    // Check if the pixel is within the current flush area
                    if (draw_x >= x_start && draw_x < x_end && draw_y >= y_start && draw_y < y_end) {
                        // Calculate the position in the color_data buffer
                        int buffer_x = draw_x - x_start;
                        int buffer_y = draw_y - y_start;
                        int buffer_index = buffer_y * (x_end - x_start) + buffer_x;
                        uint16_t* pixel_ptr = (uint16_t*)((uint8_t*)color_data + buffer_index * sizeof(uint16_t));
                        // Set color
                        *pixel_ptr = self->status_point_colors_[i];
                        // blink
                        if (!self->status_points_visible_ && (i==2)) {
                            *pixel_ptr = 0x0000; // Black when not visible
                        }
                        // *pixel_ptr = test_color[i];
                    }
                }
            }
        }
    }

    self->transmit_busy_ = true;
    esp_lcd_panel_draw_bitmap(panel, x_start + COLUMN_OFFSET, y_start, x_end + COLUMN_OFFSET, y_end, color_data);
}

EmojiPlayer::EmojiPlayer(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io)
    : panel_(panel) // 初始化成员变量
{
    ESP_LOGI(TAG, "Create EmojiPlayer, panel: %p, panel_io: %p", panel, panel_io);
    const mmap_assets_config_t assets_cfg = {
        .partition_label = "ota_0",
        .max_files = MMAP_MOJI_EMOJI_FILES,
        .checksum = MMAP_MOJI_EMOJI_CHECKSUM,
        .flags = {.mmap_enable = true, .full_check = true}
    };

    mmap_assets_new(&assets_cfg, &assets_handle_);

    anim_player_config_t player_cfg = {
        .flush_cb = OnFlush,
        .update_cb = OnUpdate,  // 添加事件回调处理
        .user_data = this, // 传递 this 指针
        .flags = {.swap = true},
        .task = ANIM_PLAYER_INIT_CONFIG()
    };

    player_handle_ = anim_player_init(&player_cfg);

    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = OnFlushIoReady,
    };
    esp_lcd_panel_io_register_event_callbacks(panel_io, &cbs, player_handle_);

    // start blinking timer
    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            auto* self = static_cast<EmojiPlayer*>(arg);
            self->status_points_visible_ = !self->status_points_visible_;
        },
        .arg = this,
        .name = "status_point_blink_timer"
    };
    esp_timer_create(&timer_args, &status_point_timer_);
    esp_timer_start_periodic(status_point_timer_, 500 * 1000); // 500ms

    StartPlayer(MMAP_MOJI_EMOJI_RELAXED_AAF, true, EMOJI_FPS);
}

EmojiPlayer::~EmojiPlayer()
{
    if (player_handle_) {
        anim_player_update(player_handle_, PLAYER_ACTION_STOP);
        anim_player_deinit(player_handle_);
        player_handle_ = nullptr;
    }

    if (assets_handle_) {
        mmap_assets_del(assets_handle_);
        assets_handle_ = NULL;
    }
}

void EmojiPlayer::PlayOnce(int aaf, int fps, std::function<void()> on_complete)
{
    if (player_handle_) {

        uint32_t start, end;
        const void *src_data;
        size_t src_len;

        src_data = mmap_assets_get_mem(assets_handle_, aaf);
        src_len = mmap_assets_get_size(assets_handle_, aaf);

        anim_player_set_src_data(player_handle_, src_data, src_len);
        anim_player_get_segment(player_handle_, &start, &end);
        anim_player_set_segment(player_handle_, start, end, fps, false);
        anim_player_update(player_handle_, PLAYER_ACTION_START);

        // 获取动画帧总宽高（假设每帧分块x_start=0, x_end=width, y_start=0, y_end=height）
        // 这里用anim_player_get_width/height等API，如果没有则在OnFlush首块记录
        // 先清零偏移
        x_offset_ = 0;
        y_offset_ = 0;

        // store completion callback (may be empty)
        play_once_callback_ = std::move(on_complete);
    }
}



void EmojiPlayer::StartPlayer(int aaf, bool repeat, int fps)
{
    if (player_handle_) {
        // 单次播放恢复逻辑s

        
        // 已移除全屏清黑色代码
        uint32_t start, end;
        const void *src_data;
        size_t src_len;

        src_data = mmap_assets_get_mem(assets_handle_, aaf);
        src_len = mmap_assets_get_size(assets_handle_, aaf);

        anim_player_set_src_data(player_handle_, src_data, src_len);
        anim_player_get_segment(player_handle_, &start, &end);
        // if(MMAP_MOJI_EMOJI_WAKE_AAF == aaf){
        //     start = 7;
        // }
        anim_player_set_segment(player_handle_, start, end, fps, repeat);
        anim_player_update(player_handle_, PLAYER_ACTION_START);

        // 获取动画帧总宽高（假设每帧分块x_start=0, x_end=width, y_start=0, y_end=height）
        // 这里用anim_player_get_width/height等API，如果没有则在OnFlush首块记录
        // 先清零偏移
        x_offset_ = 0;
        y_offset_ = 0;



    }
}

void EmojiPlayer::TimedPLay(int aaf, float time, int fps,  std::function<void()> on_complete)
{
    if (player_handle_) {
        if (timed_play_active_) {
            esp_timer_stop(timed_play_timer_);
            esp_timer_delete(timed_play_timer_);
            timed_play_timer_ = nullptr;
            timed_play_active_ = false;
            StopPlayer();
        }
        if (on_complete) {
            timed_play_callback_ = std::move(on_complete);
        } else {
            timed_play_callback_ = {};
        }
        
        StopPlayer();
        StartPlayer(aaf, true, fps);
        // setup timer to stop after time seconds
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                auto* self = static_cast<EmojiPlayer*>(arg);
                self->StopPlayer();
                self->timed_play_active_ = false;
                if (self->timed_play_callback_) {
                    auto cb = std::move(self->timed_play_callback_);
                    self->timed_play_callback_ = {};
                    try {
                        cb();
                    } catch (...) {
                        ESP_LOGW(TAG, "TimedPlay callback threw an exception");
                    }
                }
            },
            .arg = this,
            .name = "timed_play_timer"
        };
        esp_timer_create(&timer_args, &timed_play_timer_);
        esp_timer_start_once(timed_play_timer_, static_cast<uint64_t>(time * 1000000)); // convert to microseconds
        timed_play_active_ = true;
    }
}

void EmojiPlayer::StopPlayer()
{
    if (player_handle_) {
        anim_player_update(player_handle_, PLAYER_ACTION_STOP);
    }
}

void EmojiPlayer::OnUpdate(anim_player_handle_t handle, player_event_t event)
{
    auto* self = static_cast<EmojiPlayer*>(anim_player_get_user_data(handle));
    if (!self) {
        return;
    }
    
    switch (event) {
        case PLAYER_EVENT_ONE_FRAME_DONE:
            // 单帧播放完成，不需要特殊处理
            // ESP_LOGI(TAG, "EmojiPlayer: One frame done");
            break;
            
        case PLAYER_EVENT_ALL_FRAME_DONE:
            // 所有帧播放完成 — if PlayOnce had a completion callback, invoke it and clear
            // ESP_LOGI(TAG, "EmojiPlayer: All frames done");
            if (self->play_once_callback_) {
                // move it out and clear to avoid reentrancy issues
                auto cb = std::move(self->play_once_callback_);
                self->play_once_callback_ = {};
                try {
                    cb();
                } catch (...) {
                    ESP_LOGW(TAG, "PlayOnce callback threw an exception");
                }
            }
            break;
            
        case PLAYER_EVENT_IDLE:
            // ESP_LOGI(TAG, "EmojiPlayer: Idle");
            // 播放器进入空闲状态
            break;
            
        default:
            // ESP_LOGI(TAG, "EmojiPlayer: Unknown event");
            break;
    }
}

void EmojiPlayer::SetStatusPointColors(uint16_t colors[3])
{
    for (int i = 0; i < 3; ++i) {
        status_point_colors_[i] = colors[i];
    }
}

EmojiWidget::EmojiWidget(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io)
{
    InitializePlayer(panel, panel_io);

}

EmojiWidget::~EmojiWidget()
{

}

void EmojiWidget::SetEmotion(const char* emotion)
{
    if (!player_) {
        return;
    }

    using Param = std::tuple<int, bool, int>;
    static const std::unordered_map<std::string, Param> emotion_map = {
        {"happy",       {MMAP_MOJI_EMOJI_HAPPY_AAF, true, EMOJI_FPS}},
        // {"laughing",    {MMAP_MOJI_EMOJI_LAUGHING_AAF, true, EMOJI_FPS}},
        // {"funny",       {MMAP_MOJI_EMOJI_FUNNY_AAF, true, EMOJI_FPS}},
        // {"loving",      {MMAP_MOJI_EMOJI_LOVING_AAF, true, EMOJI_FPS}},
        // {"embarrassed", {MMAP_MOJI_EMOJI_EMBARRASSED_AAF, true, EMOJI_FPS}},
        // {"confident",   {MMAP_MOJI_EMOJI_CONFIDENT_AAF, true, EMOJI_FPS}},
        // {"delicious",   {MMAP_MOJI_EMOJI_DELICIOUS_AAF, true, EMOJI_FPS}},
        {"sad",         {MMAP_MOJI_EMOJI_SAD_AAF,   true, EMOJI_FPS}},
        // {"crying",      {MMAP_MOJI_EMOJI_CRYING_AAF,   true, EMOJI_FPS}},
        // {"sleepy",      {MMAP_MOJI_EMOJI_SLEEPY_AAF,   true, EMOJI_FPS}},
        // {"silly",       {MMAP_MOJI_EMOJI_SILLY_AAF,   true, EMOJI_FPS}},
        {"angry",       {MMAP_MOJI_EMOJI_ANGRY_AAF, true, EMOJI_FPS}},
        // {"surprised",   {MMAP_MOJI_EMOJI_SURPRISE_AAF, true, EMOJI_FPS}},
        // {"shocked",     {MMAP_MOJI_EMOJI_SHOCKED_AAF, true, EMOJI_FPS}},
        {"thinking",    {MMAP_MOJI_EMOJI_THINKING_AAF, true, EMOJI_FPS}},
        {"winking",     {MMAP_MOJI_EMOJI_WINKING_AAF, true, EMOJI_FPS}},
        {"relaxed",     {MMAP_MOJI_EMOJI_RELAXED_AAF, true, EMOJI_FPS}},
        // {"confused",    {MMAP_MOJI_EMOJI_CONFUSED_AAF, true, EMOJI_FPS}},
        {"music",     {MMAP_MOJI_EMOJI_MUSIC_AAF, true, EMOJI_FPS}}
    };

    auto it = emotion_map.find(emotion);
    if (it != emotion_map.end()) {
        const auto& [aaf, repeat, fps] = it->second;
        PlayEmoji(aaf);
    } 
    else {
        ESP_LOGI(TAG, "SetEmoji called --- unknown emotion: %s", emotion);
    }
}

void EmojiWidget::PlayEmoji(int aaf_id, float time)
{
    if (player_) {
        StopIdleEmojiRotation();
        if (aaf_id == MMAP_MOJI_EMOJI_RELAXED_AAF || Application::GetInstance().GetDeviceState() == kDeviceStateIdle) {
            StartIdleEmojiRotation();
            return;
        }
        if (time > 0) {
            player_->TimedPLay(aaf_id, time, EMOJI_FPS, [this]() {
                ESP_LOGI(TAG, "PlayEmoji completed");
                // Reset the emoji to neutral after the timed play
                this->player_->TimedPLay(MMAP_MOJI_EMOJI_RELAXED_AAF, 2.0f, EMOJI_FPS, [this]() {
                    ESP_LOGI(TAG, "Returned to RELAXED emoji after timed play");

                    
                    this->StartIdleEmojiRotation();
                });
            });
            ESP_LOGI(TAG, "PlayEmoji called --- Play AAF ID: %d for %.2f seconds", aaf_id, time);
        } else {
            player_->StartPlayer(aaf_id, true, EMOJI_FPS);
            ESP_LOGI(TAG, "PlayEmoji called --- Start AAF ID: %d indefinitely", aaf_id);
        }   
    }
}

void EmojiWidget::SetStatus(const char* status)
{
    if (player_) {
        if (strcmp(status, Lang::Strings::LISTENING) ==0 || strcmp(status, Lang::Strings::SPEAKING) == 0) {
            StopIdleEmojiRotation();
            player_->StartPlayer(MMAP_MOJI_EMOJI_WINKING_AAF, true, EMOJI_FPS);
        } else {
            StartIdleEmojiRotation();
        }
    }
}

void EmojiWidget::StartIdleEmojiRotation()
{
    if (idle_rotation_active_){
        return;
    }

    if (!idle_rotation_timer_){
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                auto* self = static_cast<EmojiWidget*>(arg);
                ESP_LOGI(TAG, "Idle emoji rotation timer triggered");
                if (self->player_) {
                    auto& app = Application::GetInstance();
                    if (app.GetDeviceState() != kDeviceStateIdle) {
                        ESP_LOGI(TAG, "Device no longer in IDLE state, stopping emoji rotation");
                        self->StopIdleEmojiRotation();
                        return;
                    }
                    self->idle_last_periods_++;

                    if (self->idle_last_periods_ >= 15) {
                        if (self->idle_last_periods_ >=30) {
                            self->idle_emoji = MMAP_MOJI_EMOJI_DEEPSLEEP_AAF; // 2 minutes
                        } else {
                            self->idle_emoji = MMAP_MOJI_EMOJI_YAWNING_AAF; // 1 minute
                        }
                    } else {
                        self->idle_emoji = MMAP_MOJI_EMOJI_RELAXED_AAF; // default
                    }

                    if (self->idle_emoji == MMAP_MOJI_EMOJI_RELAXED_AAF) {
                        self->player_->TimedPLay(MMAP_MOJI_EMOJI_BLINK_AAF, 0.8f, 10, [self]() {
                            ESP_LOGI(TAG, "IDLE emoji rotation: BLINK emoji play completed");
                            // after blink, play default idle emoji
                            self->player_->StartPlayer(self->idle_emoji, true, EMOJI_FPS);
                        });
                    } else {
                        self->player_->StartPlayer(self->idle_emoji, true, EMOJI_FPS);
                    }
                }
            },
            .arg = this,
            .name = "idle_emoji_rotation_timer"
        };
        esp_timer_create(&timer_args, &idle_rotation_timer_);
    }

    if (player_) {
        player_->StartPlayer(idle_emoji, true, EMOJI_FPS);
        ESP_LOGI(TAG, "Started IDLE emoji rotation with idle emoji");
        esp_timer_start_periodic(idle_rotation_timer_, 4 * 1000000); // 4 seconds
        idle_rotation_active_ = true;
    }
}

void EmojiWidget::StopIdleEmojiRotation()
{
    if (!idle_rotation_active_){
        return;
    }

    if (idle_rotation_timer_){
        esp_timer_stop(idle_rotation_timer_);
    }

    idle_rotation_active_ = false;
    idle_last_periods_ = 0;
    idle_emoji = MMAP_MOJI_EMOJI_RELAXED_AAF;
}


void EmojiWidget::UpdateStatusBar(bool update_all)
{
    auto& board = Board::GetInstance();
    uint16_t colors[3];

    // battry
    int battery_level;
    bool charging, discharging;

    if (board.GetBatteryLevel(battery_level, charging, discharging)) {
        if (charging) {
            display_status_.power_status = display_status_.CHARGING;
            colors[2] = COLOR(0.251f, 0.89f, 0.0f); // Green
        } else if (battery_level >= 20) {
            display_status_.power_status = display_status_.MEDIUM;
            colors[2] = COLOR(0.0f, 0.0f, 0.0f); // Black
        } else {
            display_status_.power_status = display_status_.LOW;
            colors[2] = COLOR(1.0f, 0.07f, 0.0f); // Red
        }
    }

    // network
    if (strcmp(board.GetNetworkStateIcon(), FONT_AWESOME_WIFI_OFF) == 0) {
        display_status_.network_status = display_status_.DISCONNECTED;
        colors[0] = 0x0000;
    } else {
        display_status_.network_status = display_status_.CONNECTED;
        colors[0] = COLOR(0.0f, 0.706f, 1.0f); // Blue
    }

    // privacy mode
    auto& app = Application::GetInstance();
    if (app.GetDeviceState() == kDeviceStateListening || app.GetDeviceState() == kDeviceStateConnecting)
    {
        display_status_.privacy_status = display_status_.NORMAL;
        colors[1] = COLOR(1.0f, 0.5f, 0.0f); // Orange
    } 
    else 
    {
        // if (display_status_.privacy_status != display_status_.PRIVACY){
        //     // if listening previously, change back  only after speaking
        //     if (app.GetDeviceState() == kDeviceStateSpeaking) {
        //         display_status_.privacy_status = display_status_.PRIVACY;
        //         colors[1] = 0x0000; // Black
        //         return;
        //     } else {
        //         return; // do not change
        //     }
        // }
        colors[1] = 0x0000; // Black
    }

    player_->SetStatusPointColors(colors);
}




void EmojiWidget::InitializePlayer(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io)
{
    player_ = std::make_unique<EmojiPlayer>(panel, panel_io);
}

bool EmojiWidget::Lock(int timeout_ms)
{
    return true;
}

void EmojiWidget::Unlock()
{
}

} // namespace moji_anim 