#include "gyro_sensor.h"
#include "config.h"
#include <esp_log.h>
#include <cstring>
#include <cmath>

static const char* TAG = "Bmi270Sensor";

Bmi270Sensor::Bmi270Sensor() {}

Bmi270Sensor::~Bmi270Sensor() {
    Deinitialize();
}

bool Bmi270Sensor::IsInitialized() const { return initialized_; }

bool Bmi270Sensor::Initialize(i2c_master_bus_handle_t i2c_bus, DataCallback cb) {
    if (initialized_) return true;
    i2c_bus_ = i2c_bus;
    cb_ = cb;

    // Add device to bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI270_I2C_ADDR,
        .scl_speed_hz = BMI270_I2C_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {},
    };
    esp_err_t err = i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &idf_dev_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BMI270 device to I2C bus: %s", esp_err_to_name(err));
        return false;
    }

    // Setup bmi2 device context and init
    static struct bmi2_dev bmi2_dev_ctx = {};
    memset(&bmi2_dev_ctx, 0, sizeof(bmi2_dev_ctx));
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
    bmi2_dev_ctx.intf_ptr = (void*)idf_dev_;
    bmi2_dev_ctx.read_write_len = 32;
    bmi2_dev_ctx.config_file_ptr = NULL;

    int8_t rslt = bmi270_init(&bmi2_dev_ctx);
    if (rslt != 0) {
        ESP_LOGE(TAG, "bmi270_init failed: %d", (int)rslt);
        return false;
    }
    handle_ = (bmi270_handle_t)&bmi2_dev_ctx;

    struct bmi2_sens_config config[2];
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;
    struct bmi2_dev *dev = (struct bmi2_dev*)handle_;
    rslt = bmi2_get_sensor_config(config, 2, dev);
    if (rslt != 0) {
        ESP_LOGE(TAG, "bmi2_get_sensor_config failed: %d", (int)rslt);
        return false;
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
        return false;
    }

    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sensor_list, 2, dev);
    if (rslt != 0) {
        ESP_LOGE(TAG, "bmi2_sensor_enable failed: %d", (int)rslt);
        return false;
    }

    // create periodic timer for polling
    esp_timer_create_args_t gyro_timer_args = {
        .callback = TimerCb,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gyro_timer",
        .skip_unhandled_events = true,
    };
    esp_err_t err2 = esp_timer_create(&gyro_timer_args, &timer_);
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create gyro timer: %s", esp_err_to_name(err2));
        return false;
    }
    err2 = esp_timer_start_periodic(timer_, 10 * 1000); // 10ms
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start gyro timer: %s", esp_err_to_name(err2));
        esp_timer_delete(timer_);
        timer_ = nullptr;
        return false;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "BMI270 initialized and polling started");
    return true;
}

void Bmi270Sensor::Deinitialize() {
    if (timer_) {
        esp_timer_stop(timer_);
        esp_timer_delete(timer_);
        timer_ = nullptr;
    }
    // We don't own idf_dev_ cleanup here – the i2c_master_bus api may provide cleanup elsewhere.
    handle_ = nullptr;
    initialized_ = false;
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

void Bmi270Sensor::OnTimer() {
    if (!initialized_ || handle_ == nullptr) return;
    struct bmi2_sens_data data = {};
    struct bmi2_dev *dev = (struct bmi2_dev*)handle_;
    int8_t rslt = bmi2_get_sensor_data(&data, dev);
    if (rslt != 0) {
        ESP_LOGW(TAG, "Failed to read BMI270 data, rslt=%d", (int)rslt);
        return;
    }
    if (!((data.status & BMI2_DRDY_ACC) && (data.status & BMI2_DRDY_GYR))) {
        return;
    }
    SensorData s{};
    s.accel.x = lsb_to_mps2(data.acc.x, 2.0f, 16);
    s.accel.y = lsb_to_mps2(data.acc.y, 2.0f, 16);
    s.accel.z = lsb_to_mps2(data.acc.z, 2.0f, 16);
    s.gyro.x = lsb_to_dps(data.gyr.x, 2000.0f, 16);
    s.gyro.y = lsb_to_dps(data.gyr.y, 2000.0f, 16);
    s.gyro.z = lsb_to_dps(data.gyr.z, 2000.0f, 16);

    if (cb_) cb_(s);
}
