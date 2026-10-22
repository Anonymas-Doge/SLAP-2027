#pragma once

#include <rclcpp/rclcpp.hpp>
#include "quadruped_fsm/quadruped_types.hpp"

namespace quadruped::fsm {

enum class SafetyStatus {
    NOMINAL = 0,
    TILT_WARNING,      // Triggers RECOVERY
    CRITICAL_FAULT     // Triggers ESTOP
};

class SafetyWatchdog {
public:
    SafetyWatchdog() = default;

    void updateImu(const ImuData& imu, rclcpp::Time stamp);
    void updateJoints(const JointState& joints, rclcpp::Time stamp);
    void updateTelemetry(const RobotTelemetry& telemetry);

    [[nodiscard]] SafetyStatus evaluateSafety(rclcpp::Time current_time, std::string& fault_reason);

private:
    ImuData latest_imu_{};
    JointState latest_joints_{};
    RobotTelemetry latest_telemetry_{};

    rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_joint_stamp_{0, 0, RCL_ROS_TIME};

    constexpr static float MAX_SAFE_TILT_RAD = 0.610f;  // ~35 degrees
    constexpr static float CRITICAL_TILT_RAD = 1.221f;  // ~70 degrees
    constexpr static double SENSOR_TIMEOUT_SEC = 0.100; // 100 ms
};

} // namespace quadruped::fsm