#pragma once

#include <string>

#include "sdkconfig.h"
#include <esp_err.h>
#include <esp_log.h>

#if CONFIG_BT_BLE_ENABLED
#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#else
#warning "Bluetooth is disabled in sdkconfig!"
#endif

// Lightweight singleton to advertise the device name over BLE during WiFi config mode.
// If BLE is disabled in sdkconfig, Start() becomes a no-op and logs a message.
class BleBroadcaster {
public:
    static BleBroadcaster& GetInstance();

    // Start non-connectable advertising with the given device name.
    // Safe to call multiple times; subsequent calls are ignored once advertising.
    esp_err_t Start(const char* device_name);
    // Configure custom advertising payload: Name 'LUKKA' + manufacturer data (firmware BCD + MAC).
    // version_digits: array of 3 bytes each containing BCD-coded digit (e.g. {0x01,0x04,0x09} for 1.4.9)
    esp_err_t StartCustom();

private:
    BleBroadcaster() = default;
    BleBroadcaster(const BleBroadcaster&) = delete;
    BleBroadcaster& operator=(const BleBroadcaster&) = delete;

#if CONFIG_BT_BLE_ENABLED
    static void GapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
    esp_err_t InitStack_();
    esp_err_t BeginAdv_();
    esp_err_t BeginAdvCustom_(const uint8_t version_digits[3]);
#endif

    bool initialized_ = false;
    bool advertising_ = false;
    std::string name_;
};
