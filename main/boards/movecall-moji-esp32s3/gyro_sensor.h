#pragma once

#include <functional>
#include <cstdint>
#include <esp_timer.h>
#include "bmi270.h"
#include "bmi2.h"
#include "driver/i2c_master.h"

class Bmi270Sensor {
public:
    struct Vec3 { float x, y, z; };
    struct SensorData { Vec3 accel; Vec3 gyro; };

    using DataCallback = std::function<void(const SensorData&)>;

    Bmi270Sensor();
    ~Bmi270Sensor();

    // Initialize the sensor using an existing IDF I2C master bus handle.
    // Provide a callback that will be called each time new data is polled.
    bool Initialize(i2c_master_bus_handle_t i2c_bus, DataCallback cb);
    void Deinitialize();
    bool IsInitialized() const;

private:
    bool SetupDevice();
    void OnTimer();

    static void TimerCb(void* arg) { static_cast<Bmi270Sensor*>(arg)->OnTimer(); }

    bmi270_handle_t handle_ = nullptr;
    i2c_master_dev_handle_t idf_dev_ = nullptr;
    esp_timer_handle_t timer_ = nullptr;
    DataCallback cb_;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    bool initialized_ = false;
};
