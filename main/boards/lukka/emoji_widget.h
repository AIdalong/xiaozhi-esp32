#pragma once

#include "display/lcd_display.h"
#include <memory>
#include <functional>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include "anim_player.h"
//#include "mmap_generate_emoji.h"
#include <esp_mmap_assets.h>
#include "mmap_generate_moji_emoji.h"

// extern const uint8_t assets_A_bin_start[] asm("_binary_assets_A_bin_start");
// extern const uint8_t assets_A_bin_end[]   asm("_binary_assets_A_bin_end");

namespace moji_anim {

class EmojiPlayer;

using FlushIoReadyCallback = std::function<bool(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void*)>;
using FlushCallback = std::function<void(anim_player_handle_t, int, int, int, int, const void*)>;

class EmojiPlayer {
public:
    EmojiPlayer(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io);
    ~EmojiPlayer();

    void StartPlayer(int aaf, bool repeat, int fps);
            // Play the given animation once. When playback completes, on_complete will be invoked
    // on the player's task context. The callback will be cleared after invocation.
    void PlayOnce(int aaf, int fps, std::function<void()> on_complete = {});
    void TimedPLay(int aaf, float time, int fps,  std::function<void()> on_complete = {});
    void StopPlayer();
    void SetStatusPointColors(uint16_t colors[3]);
    bool IsTransmitBusy() const { return transmit_busy_; }

private:
    static bool OnFlushIoReady(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
    static void OnFlush(anim_player_handle_t handle, int x_start, int y_start, int x_end, int y_end, const void *color_data);
    static void OnUpdate(anim_player_handle_t handle, player_event_t event);

    anim_player_handle_t player_handle_;
    mmap_assets_handle_t assets_handle_;
    esp_lcd_panel_handle_t panel_; // 新增成员变量
    int x_offset_ = 0;
    int y_offset_ = 0;
    // optional callback invoked when PlayOnce finishes
    std::function<void()> play_once_callback_;
    std::function<void()> timed_play_callback_;

    // timer for blinking status points
    esp_timer_handle_t status_point_timer_;
    bool status_points_visible_ = true;

    // timer for timed play
    esp_timer_handle_t timed_play_timer_;
    bool timed_play_active_ = false;

    // store colors of status points
    uint16_t status_point_colors_[3] = {0x0000, 0x0000, 0x0000};

    bool transmit_busy_ = false;
};

class EmojiWidget : public Display {
public:
    EmojiWidget(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io);
    virtual ~EmojiWidget();

    virtual void SetEmotion(const char* emotion) override;
    void PlayEmoji(int aaf_id, float time=2.0f);
    virtual void SetStatus(const char* status) override;
    virtual void UpdateStatusBar(bool update_all = false) override;
    moji_anim::EmojiPlayer* GetPlayer()
    {
        return player_.get();
    }
    bool IsPlayingAnimation() const {
        return is_playing_animation_;
    }

private:
    void InitializePlayer(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io);
    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;
    bool is_playing_animation_ = false;

    bool idle_rotation_active_ = false;
    int idle_emoji = MMAP_MOJI_EMOJI_RELAXED_AAF;
    int idle_last_periods_ = 0; // 4s per period
    esp_timer_handle_t idle_rotation_timer_ = nullptr;
    void StartIdleEmojiRotation();
    void StopIdleEmojiRotation();

    // status for the bottom status points
    struct {
        enum { DISCONNECTED, CONNECTED } network_status;
        enum { PRIVACY, NORMAL } privacy_status;
        enum { LOW, MEDIUM, CHARGING } power_status;
    } display_status_;

    std::unique_ptr<moji_anim::EmojiPlayer> player_;
};

} // namespace moji_anim 