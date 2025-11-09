#pragma once

#include <cstdint>
#include <functional>
#include <vector>

class MotionDetector {
public:
    enum class MotionEvent : uint8_t {
        Speeding,
        Braking,
        TurnLeft,
        TurnRight,
    };

    // on_motion: called when a vehicle posture event is detected. The int parameter is the suggested AAF id to play.
    // on_shake: called when a shake event is detected (suggested AAF id can be passed if needed).
    MotionDetector(std::function<void(MotionEvent)> on_motion,
                   std::function<void()> on_shake);
    ~MotionDetector();

    // Provide raw sensor floats
    void OnSensorData(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z);
    void SetPlacementIndependent(bool independent);

private:
    // copy of last sensor values
    struct SensorVals { float ax, ay, az; float gx, gy, gz; } last_{};

    // buffers for shake detection
    std::vector<float> gyro_x_buffer;
    std::vector<float> gyro_y_buffer;

    // vehicle motion buffers/state
    std::vector<float> accel_x_buffer;
    std::vector<float> accel_y_buffer;
    std::vector<float> gyro_z_buffer;

    int BUFFER_SIZE = 10;
    float accel_x_filtered = 0.0f;
    float accel_y_filtered = 0.0f;
    float gyro_z_filtered = 0.0f;

    int64_t last_animation_time = 0;
    const int64_t ANIMATION_COOLDOWN_US = 3000 * 1000;
    int64_t init_start_time = 0;
    const int64_t INIT_STABILIZATION_US = 5000 * 1000;
    bool detection_enabled = false;

    int64_t shake_start_time = 0;
    int64_t last_shake_animation_time = 0;
    const int64_t SHAKE_DETECTION_DURATION_US = 3000 * 1000;
    const int64_t SHAKE_COOLDOWN_US = 5000 * 1000;
    const float SHAKE_THRESHOLD = 15.0f;
    bool is_shaking = false;

    // event callbacks
    std::function<void(MotionEvent)> on_motion_;
    std::function<void()> on_shake_;

    bool placement_independent_ = true;

    void ProcessVehicleMotion();
    void DetectVehiclePosture(int64_t current_time);
    void DetectShake(int64_t current_time);
};
