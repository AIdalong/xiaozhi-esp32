#include "motion_detector.h"
#include <esp_log.h>
#include <cmath>
#include "esp_timer.h"
#include "mmap_generate_moji_emoji.h"

static const char* TAG_MOTION = "MotionDetector";

MotionDetector::MotionDetector(std::function<void(MotionEvent)> on_motion,
                               std::function<void()> on_shake)
        : on_motion_(on_motion), on_shake_(on_shake) {}

MotionDetector::~MotionDetector() {}

void MotionDetector::SetPlacementIndependent(bool independent) {
    placement_independent_ = independent;
    // reset stabilization timer when placement changes
    if (independent) {
        detection_enabled = false;
        init_start_time = 0;
    } else {
        init_start_time = 0; // will be set on next data
    }
}

void MotionDetector::OnSensorData(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z) {
    last_.ax = accel_x; last_.ay = accel_y; last_.az = accel_z;
    last_.gx = gyro_x; last_.gy = gyro_y; last_.gz = gyro_z;

    // Append gyro X/Y for shake detection
    gyro_x_buffer.push_back(last_.gx);
    gyro_y_buffer.push_back(last_.gy);
    if (gyro_x_buffer.size() > (size_t)BUFFER_SIZE) gyro_x_buffer.erase(gyro_x_buffer.begin());
    if (gyro_y_buffer.size() > (size_t)BUFFER_SIZE) gyro_y_buffer.erase(gyro_y_buffer.begin());

    ProcessVehicleMotion();

    int64_t now = esp_timer_get_time();
    DetectShake(now);
}

void MotionDetector::ProcessVehicleMotion() {
    if (placement_independent_) return; // skip when independent

    int64_t current_time = esp_timer_get_time();
    if (init_start_time == 0) {
        init_start_time = current_time;
        ESP_LOGI(TAG_MOTION, "Vehicle motion detection starting, stabilization period: 5 seconds");
    }

    if (current_time - init_start_time < INIT_STABILIZATION_US) {
        accel_x_buffer.push_back(last_.ax);
        accel_y_buffer.push_back(last_.ay);
        gyro_z_buffer.push_back(last_.gz);
        if (accel_x_buffer.size() > (size_t)BUFFER_SIZE) accel_x_buffer.erase(accel_x_buffer.begin());
        if (accel_y_buffer.size() > (size_t)BUFFER_SIZE) accel_y_buffer.erase(accel_y_buffer.begin());
        if (gyro_z_buffer.size() > (size_t)BUFFER_SIZE) gyro_z_buffer.erase(gyro_z_buffer.begin());
        return;
    }

    if (!detection_enabled) {
        detection_enabled = true;
        ESP_LOGI(TAG_MOTION, "Vehicle motion detection enabled after stabilization period");
    }

    accel_x_buffer.push_back(last_.ax);
    accel_y_buffer.push_back(last_.ay);
    gyro_z_buffer.push_back(last_.gz);
    if (accel_x_buffer.size() > (size_t)BUFFER_SIZE) accel_x_buffer.erase(accel_x_buffer.begin());
    if (accel_y_buffer.size() > (size_t)BUFFER_SIZE) accel_y_buffer.erase(accel_y_buffer.begin());
    if (gyro_z_buffer.size() > (size_t)BUFFER_SIZE) gyro_z_buffer.erase(gyro_z_buffer.begin());

    if (accel_x_buffer.size() >= (size_t)BUFFER_SIZE) {
        float sum_x = 0, sum_y = 0, sum_gz = 0;
        for (int i = 0; i < BUFFER_SIZE; ++i) {
            sum_x += accel_x_buffer[i];
            sum_y += accel_y_buffer[i];
            sum_gz += gyro_z_buffer[i];
        }
        accel_x_filtered = sum_x / BUFFER_SIZE;
        accel_y_filtered = sum_y / BUFFER_SIZE;
        gyro_z_filtered = sum_gz / BUFFER_SIZE;

        // emit motion events; board will decide whether to act (play animation/sound)
        DetectVehiclePosture(esp_timer_get_time());
    }
}

void MotionDetector::DetectVehiclePosture(int64_t current_time) {
    // Board is responsible for gating animations; MotionDetector only emits events
    const float ACCEL_X_THRESHOLD_ACCEL = 0.3f * 9.80665f;
    const float ACCEL_X_THRESHOLD_BRAKE = 0.6f * 9.80665f;
    const float GYRO_Z_THRESHOLD = 18.0f;

    if (accel_x_filtered < -ACCEL_X_THRESHOLD_ACCEL) {
        ESP_LOGI(TAG_MOTION, "Vehicle accelerating: ax=%.3f g", accel_x_filtered / 9.80665f);
        if (on_motion_) on_motion_(MotionEvent::Speeding);
        return;
    }

    if (accel_x_filtered > ACCEL_X_THRESHOLD_BRAKE) {
        ESP_LOGI(TAG_MOTION, "Vehicle braking: ax=%.3f g", accel_x_filtered / 9.80665f);
        if (on_motion_) on_motion_(MotionEvent::Braking);
        return;
    }

    if (gyro_z_filtered > GYRO_Z_THRESHOLD) {
        ESP_LOGI(TAG_MOTION, "Vehicle turning left: gz=%.1f deg/s", gyro_z_filtered);
        if (on_motion_) on_motion_(MotionEvent::TurnLeft);
        return;
    }

    if (gyro_z_filtered < -GYRO_Z_THRESHOLD) {
        ESP_LOGI(TAG_MOTION, "Vehicle turning right: gz=%.1f deg/s", gyro_z_filtered);
        if (on_motion_) on_motion_(MotionEvent::TurnRight);
        return;
    }
}

void MotionDetector::DetectShake(int64_t current_time) {
    if (current_time - last_shake_animation_time < SHAKE_COOLDOWN_US) return;

    float gx = last_.gx, gy = last_.gy, gz = last_.gz;
    float shake_intensity = std::sqrt(gx*gx + gy*gy + gz*gz);
    if (shake_intensity > SHAKE_THRESHOLD) {
        if (!is_shaking) {
            is_shaking = true;
            shake_start_time = current_time;
            ESP_LOGD(TAG_MOTION, "Independent shake started: intensity=%.1f deg/s", shake_intensity);
        } else {
            int64_t shake_duration = current_time - shake_start_time;
            if (shake_duration >= SHAKE_DETECTION_DURATION_US) {
                ESP_LOGI(TAG_MOTION, "Independent shake detected for %.1f seconds", shake_duration / 1000000.0f);
                if (on_shake_) on_shake_();
                last_shake_animation_time = current_time;
                is_shaking = false;
                shake_start_time = 0;
            }
        }
    } else {
        if (is_shaking) {
            ESP_LOGD(TAG_MOTION, "Independent shake stopped: intensity=%.1f deg/s", shake_intensity);
            is_shaking = false;
            shake_start_time = 0;
        }
    }
}
