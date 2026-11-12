#pragma once

#include <array>
#include <cstdint>

#include "quadruped_types.hpp"
#include "layer1/hardware_interfaces.hpp"

namespace quadruped::layer1 {

enum Fault : uint32_t {
    FAULT_NONE = 0,
    FAULT_CAN_INIT = 1u << 0,
    FAULT_IMU_INIT = 1u << 1,
    FAULT_COMMAND_STALE = 1u << 2,
    FAULT_STATUS_STALE = 1u << 3,
    FAULT_CONTROLLER = 1u << 4,
    FAULT_TRACKING_ERROR = 1u << 5,
    FAULT_INVALID_COMMAND = 1u << 6,
    FAULT_ESTOP = 1u << 7,
    FAULT_CAN_TX = 1u << 8,
};

class Layer1Manager {
public:
    Layer1Manager(CanInterface& can, ImuInterface& imu);

    // Call once during setup. Returns true only if both hardware adapters start.
    bool init(uint32_t now_ms);

    // Supply the latest Layer 2 command. Calls to submitCommand, update, and
    // the E-stop entry point must be serialized by the application scheduler.
    void submitCommand(const JointCommand& command, uint32_t received_at_ms);

    // ISR-safe: sets a flag only. Wire the physical driver-disable line
    // directly as well; this software latch is not the hardware cutoff.
    void onEstopInterrupt();
    void clearEstopLatch(); // Only call after an explicit, deliberate reset.

    // Bounded periodic work: poll one CAN status frame, poll IMU, check safety,
    // and send the latest targets when no fault is latched.
    void update(uint32_t now_ms);

    const JointState& jointState() const { return joint_state_; }
    const std::array<bool, NUM_JOINTS>& jointVelocityValid() const { return velocity_valid_; }
    const std::array<bool, NUM_JOINTS>& jointEffortValid() const { return effort_valid_; }
    const ImuData& imuData() const { return imu_data_; }
    uint32_t faults() const { return faults_; }
    bool isSafeStopped() const { return safe_stopped_; }

private:
    CanInterface& can_;
    ImuInterface& imu_;
    JointCommand command_{};
    JointState joint_state_{};
    ImuData imu_data_{};
    std::array<uint32_t, NUM_JOINTS> joint_last_status_ms_{};
    std::array<bool, NUM_JOINTS> velocity_valid_{};
    std::array<bool, NUM_JOINTS> effort_valid_{};
    std::array<uint32_t, NUM_JOINTS> tracking_error_since_ms_{};
    std::array<bool, NUM_JOINTS> tracking_error_active_{};
    uint32_t command_received_at_ms_{0};
    uint32_t initialized_at_ms_{0};
    uint32_t faults_{FAULT_NONE};
    bool command_received_{false};
    bool command_seen_once_{false};
    bool estop_latched_{false};
    volatile bool estop_interrupt_pending_{false};
    bool safe_stopped_{false};
    bool can_ready_{false};
    bool imu_ready_{false};
    bool safe_state_sent_{false};

    void consumeStatus(const ControllerStatus& status, uint32_t now_ms);
    void checkSafety(uint32_t now_ms);
    void requestSafeState();
    bool validateAndBoundCommand(JointCommand& command);
};

} // namespace quadruped::layer1
