#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "quadruped_fsm/quadruped_types.hpp"

namespace quadruped::fsm {

class CommandMux {
public:
    CommandMux() = default;

    void setTeleopVel(const Twist2D& vel, rclcpp::Time stamp);
    void setNav2Vel(const Twist2D& vel, rclcpp::Time stamp);
    void setTeleopPose(const EulerAngles& pose);

    [[nodiscard]] Twist2D getActiveVelocity(rclcpp::Time current_time);
    [[nodiscard]] EulerAngles getActivePose() const;

private:
    Twist2D teleop_vel_{};
    Twist2D nav2_vel_{};
    EulerAngles teleop_pose_{};

    rclcpp::Time last_teleop_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_nav2_stamp_{0, 0, RCL_ROS_TIME};

    constexpr static double TELEOP_TIMEOUT_SEC = 0.5;
    constexpr static float DEADZONE_VEL = 0.02f;
};

} // namespace quadruped::fsm