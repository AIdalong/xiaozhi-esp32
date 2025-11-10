#include "ble_broadcaster.h"
#include "esp_app_desc.h"
#include "esp_mac.h"

static const char* TAG_BLE = "BleBroadcaster";

BleBroadcaster& BleBroadcaster::GetInstance() {
    static BleBroadcaster instance;
    return instance;
}

esp_err_t BleBroadcaster::Start(const char* device_name) {
    if (!device_name || *device_name == '\0') {
        ESP_LOGW(TAG_BLE, "Empty device name, skip BLE advertising");
        return ESP_ERR_INVALID_ARG;
    }
    if (advertising_) {
        ESP_LOGI(TAG_BLE, "BLE advertising already started");
        return ESP_OK;
    }
    name_ = device_name;

#if !CONFIG_BT_BLE_ENABLED
    ESP_LOGW(TAG_BLE, "BLE not enabled in sdkconfig. Skipping BLE advertising.");
    return ESP_ERR_NOT_SUPPORTED;
#else
    esp_err_t err = ESP_OK;
    if (!initialized_) {
        err = InitStack_();
        if (err != ESP_OK) {
            ESP_LOGE(TAG_BLE, "Failed to init BLE stack: %s", esp_err_to_name(err));
            return err;
        }
        initialized_ = true;
    }

    // Set device name to keep controller state consistent (optional when using raw data)
    if ((err = esp_ble_gap_set_device_name(name_.c_str())) != ESP_OK) {
        ESP_LOGW(TAG_BLE, "esp_ble_gap_set_device_name failed: %s", esp_err_to_name(err));
        // continue even if name set fails
    }

    // Default to simple name advertising if custom not requested elsewhere.
    // Build raw ADV with Complete Local Name for provided name_ only.
    // Note: Some stacks expect ASCII; ensure name length <= 29 (to fit AD header).
    const uint8_t* nm = reinterpret_cast<const uint8_t*>(name_.c_str());
    uint8_t name_len = (uint8_t)strlen(name_.c_str());
    if (name_len > 29) name_len = 29; // safety cap

    uint8_t adv_raw[31] = {0};
    uint8_t idx = 0;
    // AD1: Complete Local Name
    adv_raw[idx++] = (uint8_t)(1 /*type*/ + name_len);
    adv_raw[idx++] = 0x09; // Complete Local Name
    memcpy(&adv_raw[idx], nm, name_len); idx += name_len;

    // Configure the raw advertising data
    err = esp_ble_gap_config_adv_data_raw(adv_raw, idx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BLE, "esp_ble_gap_config_adv_data_raw failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG_BLE, "Configuring BLE raw advertising data (name='%s')", name_.c_str());
    return ESP_OK;
#endif
}

#if CONFIG_BT_BLE_ENABLED
esp_err_t BleBroadcaster::InitStack_() {
    esp_err_t err;

    // Release classic BT memory to save RAM when only BLE is used.
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        return err;
    }
    if ((err = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        return err;
    }
    if ((err = esp_bluedroid_init()) != ESP_OK) {
        return err;
    }
    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        return err;
    }

    if ((err = esp_ble_gap_register_callback(BleBroadcaster::GapCallback)) != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG_BLE, "BLE stack initialized");
    return ESP_OK;
}

esp_err_t BleBroadcaster::BeginAdv_() {
    // Non-connectable, undirected advertising.
    static esp_ble_adv_params_t adv_params = {};
    adv_params.adv_int_min = 0x40;  // 40ms
    adv_params.adv_int_max = 0x60;  // 60ms
    adv_params.adv_type = ADV_TYPE_NONCONN_IND;
    adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    adv_params.channel_map = ADV_CHNL_ALL;
    adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    esp_err_t err = esp_ble_gap_start_advertising(&adv_params);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_BLE, "BLE advertising started");
    } else {
        ESP_LOGE(TAG_BLE, "Failed to start advertising: %s", esp_err_to_name(err));
    }
    return err;
}

void BleBroadcaster::GapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    auto& self = BleBroadcaster::GetInstance();
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            self.BeginAdv_();
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param && param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                self.advertising_ = true;
            } else {
                ESP_LOGE(TAG_BLE, "ADV start failed: status=%d", param ? param->adv_start_cmpl.status : -1);
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param && param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                self.advertising_ = false;
                ESP_LOGI(TAG_BLE, "Advertising stopped");
            }
            break;
        default:
            break;
    }
}
#endif

esp_err_t BleBroadcaster::StartCustom() {
#if !CONFIG_BT_BLE_ENABLED
    ESP_LOGW(TAG_BLE, "BLE not enabled in sdkconfig. Skipping BLE advertising.");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (advertising_) {
        ESP_LOGI(TAG_BLE, "BLE advertising already started");
        return ESP_OK;
    }
    if (!initialized_) {
        esp_err_t err = InitStack_();
        if (err != ESP_OK) return err;
        initialized_ = true;
    }

    // Device name fixed to 'LUKKA'
    const char* fixed_name = "LUKKA";
    esp_ble_gap_set_device_name(fixed_name);

    // Build raw ADV payload with two AD structures:
    // 1) Complete Local Name: 06 09 'L''U''K''K''A'
    // 2) Manufacturer Specific Data: 0A FF ver1 ver2 ver3 MAC[6]
    uint8_t adv_raw[31] = {0};
    uint8_t idx = 0;

    // AD1: Complete Local Name 'LUKKA'
    const uint8_t name_bytes[5] = {0x4C, 0x55, 0x4B, 0x4B, 0x41};
    adv_raw[idx++] = 1 + sizeof(name_bytes); // length of type+data
    adv_raw[idx++] = 0x09; // Complete Local Name
    memcpy(&adv_raw[idx], name_bytes, sizeof(name_bytes)); idx += sizeof(name_bytes);

    // AD2: Manufacturer Specific Data with version (3 BCD bytes) + 6-byte Wifi MAC
    auto app_desc = esp_app_get_description();
    std::string ver_str = app_desc->version;
    // parse version string like "1.4.9" into BCD bytes
    uint8_t ver[3] = {0x00, 0x00, 0x00};
    if (ver_str.length() == 5) {
        ver[0] = ver_str[0] - '0';
        ver[1] = ver_str[2] - '0';
        ver[2] = ver_str[4] - '0';
        ESP_LOGD(TAG_BLE, "Parsed version string '%s' into BCD: %02X.%02X.%02X",
                 ver_str.c_str(), ver[0], ver[1], ver[2]);
    } else {
        ESP_LOGW(TAG_BLE, "Unexpected version string format: %s", ver_str.c_str());
    }

    uint8_t wifi_mac[6];
    esp_read_mac(wifi_mac, ESP_MAC_WIFI_STA);

    adv_raw[idx++] = 1 + 3 + 6; // length of type+data
    adv_raw[idx++] = 0xFF; // Manufacturer Specific Data
    adv_raw[idx++] = ver[0];
    adv_raw[idx++] = ver[1];
    adv_raw[idx++] = ver[2];
    memcpy(&adv_raw[idx], wifi_mac, 6); idx += 6;


    // Configure raw advertising payload
    esp_err_t err = esp_ble_gap_config_adv_data_raw(adv_raw, idx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BLE, "esp_ble_gap_config_adv_data_raw failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG_BLE, "Configuring custom BLE advertising: name='LUKKA', ver=%02X.%02X.%02X",
             ver[0], ver[1], ver[2]);
    ESP_LOGD(TAG_BLE, "Payload Hex:");
    for (uint8_t i = 0; i < idx; i++) {
        ESP_LOGD(TAG_BLE, "%02X", adv_raw[i]);
    }
    return ESP_OK;
#endif
}

#if CONFIG_BT_BLE_ENABLED
esp_err_t BleBroadcaster::BeginAdvCustom_(const uint8_t version_digits[3]) {
    // Reuse BeginAdv_ parameters; raw data already set.
    return BeginAdv_();
}
#endif
