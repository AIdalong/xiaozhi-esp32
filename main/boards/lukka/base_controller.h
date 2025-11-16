#pragma once

#include <string>
#include <cstdint>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class BaseController {
public:
    enum PlacementState {
        kPlacementIndependent = 0,
        kPlacementRotatingBase = 1,
        kPlacementStaticBase = 2
    };

    BaseController();
    ~BaseController();

    bool Initialize();
    bool IsInitialized() const;

    // Motor control API
    // Initialize UART for motor control
    bool SendMotorCommand(const char* cmd);
    void ControlMotor(char direction, int steps);
    void ResetMotor();

    // Probe task control
    bool StartProbeTask();
    void StopProbeTask();

    PlacementState GetPlacementState() const { return placement_state_; }
    void SetPlacementState(PlacementState s);

    // Callback invoked when placement state changes: (new, old)
    void SetPlacementChangedCallback(std::function<void(PlacementState, PlacementState)> cb) { placement_changed_cb_ = cb; }

private:
    // Motor initialized flag (replaces separate MotorController instance)
    bool initialized_ = false;
    TaskHandle_t probe_task_handle_ = nullptr;
    PlacementState placement_state_ = kPlacementIndependent;
    std::function<void(PlacementState, PlacementState)> placement_changed_cb_;

    static void ProbeTask(void* arg);
};
