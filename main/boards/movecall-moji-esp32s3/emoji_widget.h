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
    void StopPlayer();

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
};

class EmojiWidget : public Display {
public:
    EmojiWidget(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io);
    virtual ~EmojiWidget();

    virtual void SetEmotion(const char* emotion) override;
    virtual void SetStatus(const char* status) override;
    moji_anim::EmojiPlayer* GetPlayer()
    {
        return player_.get();
    }
private:
    void InitializePlayer(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io);
    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

    std::unique_ptr<moji_anim::EmojiPlayer> player_;
};

} // namespace moji_anim 