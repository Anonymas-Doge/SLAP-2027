#pragma once

#include <cstdint>

#include "quadruped_types.hpp"
#include "layer1/layer1_config.hpp"

namespace quadruped::layer1 {

struct ControllerStatus {
    uint8_t can_id{0};
    float position{0.0f};       // radians
    float velocity{0.0f};       // radians/second
    float effort{0.0f};         // controller-reported unit; valid only when flagged
    uint32_t received_at_ms{0};
    bool position_valid{false};
    bool velocity_valid{false};
    bool effort_valid{false};
    bool controller_fault{false};
};

// Driver boundary. Implementations should do bounded, non-blocking work per
// call and translate the selected safe action to the configured moteus mode.
class CanInterface {
public:
    virtual ~CanInterface() = default;
    virtual bool begin() = 0;
    virtual bool sendPosition(uint8_t can_id, float position_rad,
                              float velocity_rad_s, float feedforward_torque) = 0;
    virtual bool pollStatus(ControllerStatus& status) = 0;
    virtual bool requestSafeState(uint8_t can_id, config::SafeAction action,
                                  float hold_position_rad) = 0;
};

struct ImuReport {
    ImuData data{};
    uint32_t received_at_ms{0};
    bool orientation_valid{false};
    bool angular_velocity_valid{false};
    bool linear_acceleration_valid{false};
};

class ImuInterface {
public:
    virtual ~ImuInterface() = default;
    virtual bool begin() = 0;
    virtual bool poll(ImuReport& report) = 0;
};

} // namespace quadruped::layer1
