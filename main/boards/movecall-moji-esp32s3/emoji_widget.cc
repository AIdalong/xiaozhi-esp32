#include <cstring>
#include "display/lcd_display.h"
#include <esp_log.h>
//#include "mmap_generate_emoji.h"
#include "emoji_widget.h"
#include "mmap_generate_moji_emoji.h"
#include "config.h"

#include <esp_lcd_panel_io.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

static const char *TAG = "moji_emoji";
#define EMOJI_FPS 2

namespace moji_anim {

bool EmojiPlayer::OnFlushIoReady(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    auto* disp_drv = static_cast<anim_player_handle_t*>(user_ctx);
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
    esp_lcd_panel_draw_bitmap(panel, x_start + COLUMN_OFFSET, y_start, x_end + COLUMN_OFFSET, y_end, color_data);
}

EmojiPlayer::EmojiPlayer(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io)
    : panel_(panel) // 初始化成员变量
{
    ESP_LOGI(TAG, "Create EmojiPlayer, panel: %p, panel_io: %p", panel, panel_io);
    const mmap_assets_config_t assets_cfg = {
        .partition_label = "assets_A",
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
    
    StartPlayer(MMAP_MOJI_EMOJI_WINKING_AAF, true, EMOJI_FPS);
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

void EmojiPlayer::StartPlayer(int aaf, bool repeat, int fps)
{
    if (player_handle_) {
        // 单次播放恢复逻辑已删除
        
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
            break;
            
        case PLAYER_EVENT_ALL_FRAME_DONE:
            // 所有帧播放完成
            break;
            
        case PLAYER_EVENT_IDLE:
            // 播放器进入空闲状态
            break;
            
        default:
            break;
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
        {"laughing",    {MMAP_MOJI_EMOJI_LAUGHING_AAF, true, EMOJI_FPS}},
        {"funny",       {MMAP_MOJI_EMOJI_FUNNY_AAF, true, EMOJI_FPS}},
        {"loving",      {MMAP_MOJI_EMOJI_LOVING_AAF, true, EMOJI_FPS}},
        {"embarrassed", {MMAP_MOJI_EMOJI_EMBARRASSED_AAF, true, EMOJI_FPS}},
        {"confident",   {MMAP_MOJI_EMOJI_CONFIDENT_AAF, true, EMOJI_FPS}},
        {"delicious",   {MMAP_MOJI_EMOJI_DELICIOUS_AAF, true, EMOJI_FPS}},
        {"sad",         {MMAP_MOJI_EMOJI_SAD_AAF,   true, EMOJI_FPS}},
        {"crying",      {MMAP_MOJI_EMOJI_CRYING_AAF,   true, EMOJI_FPS}},
        {"sleepy",      {MMAP_MOJI_EMOJI_SLEEPY_AAF,   true, EMOJI_FPS}},
        {"silly",       {MMAP_MOJI_EMOJI_SILLY_AAF,   true, EMOJI_FPS}},
        {"angry",       {MMAP_MOJI_EMOJI_ANGRY_AAF, true, EMOJI_FPS}},
        {"surprised",   {MMAP_MOJI_EMOJI_SURPRISE_AAF, true, EMOJI_FPS}},
        {"shocked",     {MMAP_MOJI_EMOJI_SHOCKED_AAF, true, EMOJI_FPS}},
        {"thinking",    {MMAP_MOJI_EMOJI_THINKING_AAF, true, EMOJI_FPS}},
        {"winking",     {MMAP_MOJI_EMOJI_WINKING_AAF, true, EMOJI_FPS}},
        {"relaxed",     {MMAP_MOJI_EMOJI_RELAXED_AAF, true, EMOJI_FPS}},
        {"confused",    {MMAP_MOJI_EMOJI_CONFUSED_AAF, true, EMOJI_FPS}},
    };

    auto it = emotion_map.find(emotion);
    if (it != emotion_map.end()) {
        const auto& [aaf, repeat, fps] = it->second;
        player_->StartPlayer(aaf, repeat, fps);
    } else if (strcmp(emotion, "neutral") == 0) {
    }
}

void EmojiWidget::SetStatus(const char* status)
{
    if (player_) {
        if (strcmp(status, "聆听中...") == 0) {
            player_->StartPlayer(MMAP_MOJI_EMOJI_LISTENING_AAF, true, EMOJI_FPS);
        } else if (strcmp(status, "待命") == 0) {
            player_->StartPlayer(MMAP_MOJI_EMOJI_DEFAULT_AAF, true, EMOJI_FPS);
        }
    }
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