#include "base_controller.h"
#include <string.h>
#include "config.h"
#include <esp_log.h>
#include "driver/uart.h"

static const char* TAG_BASE = "BaseController";

BaseController::BaseController() {}
BaseController::~BaseController() { StopProbeTask(); }

bool BaseController::Initialize() {
    if (initialized_) return true;
    uart_config_t uart_config = {};
    uart_config.baud_rate = MOJI_UART_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    int intr_alloc_flags = 0;
    if (uart_driver_install(MOJI_UART_PORT_NUM, 1024, 0, 0, NULL, intr_alloc_flags) != ESP_OK) {
        ESP_LOGE(TAG_BASE, "Failed to install UART driver for motor");
        return false;
    }
    if (uart_param_config(MOJI_UART_PORT_NUM, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG_BASE, "Failed to config UART for motor");
        return false;
    }
    if (uart_set_pin(MOJI_UART_PORT_NUM, MOJI_UART_TXD, MOJI_UART_RXD, MOJI_UART_RTS, MOJI_UART_CTS) != ESP_OK) {
        ESP_LOGE(TAG_BASE, "Failed to set UART pins for motor");
        return false;
    }
    initialized_ = true;
    return true;
}

bool BaseController::IsInitialized() const {
    return initialized_;
}

void BaseController::ControlMotor(char direction, int steps) {
    if (!IsInitialized()) {
        // try to initialize lazily
        const_cast<BaseController*>(this)->Initialize();
    }
    if (!IsInitialized()) return;
    char buffer[16];
    int n = snprintf(buffer, sizeof(buffer), "%c%d\r\n", direction, steps);
    if (n > 0) SendMotorCommand(buffer);
}

void BaseController::ResetMotor() {
    if (!IsInitialized()) {
        const_cast<BaseController*>(this)->Initialize();
    }
    if (!IsInitialized()) return;
    SendMotorCommand("X\r\n");
}

bool BaseController::SendMotorCommand(const char* cmd) {
    if (!IsInitialized()) Initialize();
    if (!IsInitialized()) return false;
    size_t len = strlen(cmd);
    uart_write_bytes(MOJI_UART_PORT_NUM, cmd, len);
    ESP_LOGI(TAG_BASE, "Motor cmd: %s", cmd);
    return true;
}

void BaseController::SetPlacementState(PlacementState s) {
    if (placement_state_ == s) return;
    PlacementState old = placement_state_;
    placement_state_ = s;
    if (placement_changed_cb_) placement_changed_cb_(placement_state_, old);
}

bool BaseController::StartProbeTask() {
    if (probe_task_handle_) return true;
    BaseType_t rt = xTaskCreate(ProbeTask, "base_probe_task", 3072, this, 1, &probe_task_handle_);
    if (rt != pdPASS) {
        ESP_LOGE(TAG_BASE, "Failed to create base probe task");
        probe_task_handle_ = nullptr;
        return false;
    }
    return true;
}

void BaseController::StopProbeTask() {
    if (probe_task_handle_) {
        vTaskDelete(probe_task_handle_);
        probe_task_handle_ = nullptr;
    }
}

void BaseController::ProbeTask(void* arg) {
    BaseController* self = static_cast<BaseController*>(arg);
    const TickType_t delay = pdMS_TO_TICKS(1000);
    for (;;) {
        if (!self->IsInitialized()) {
            self->Initialize();
        }
        if (self->IsInitialized()) {
            self->ControlMotor('L', 0);

            uint8_t buf[128];
            int len = uart_read_bytes(MOJI_UART_PORT_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(120));
            if (len > 0) {
                buf[len] = 0;
                bool on_rotating = (strstr((const char*)buf, "step") != nullptr);
                if (on_rotating) {
                    if (self->placement_state_ != kPlacementRotatingBase) {
                        ESP_LOGI(TAG_BASE, "Detected rotating base (uart contains 'step')");
                        self->SetPlacementState(kPlacementRotatingBase);
                    }
                } else {
                    if (self->placement_state_ != kPlacementIndependent) {
                        ESP_LOGI(TAG_BASE, "No 'step' found in uart response, switch to independent");
                        self->SetPlacementState(kPlacementIndependent);
                    }
                }
            } else {
                if (self->placement_state_ != kPlacementIndependent) {
                    ESP_LOGI(TAG_BASE, "No uart response, switch to independent");
                    self->SetPlacementState(kPlacementIndependent);
                }
            }
        }
        vTaskDelay(delay);
    }
}
