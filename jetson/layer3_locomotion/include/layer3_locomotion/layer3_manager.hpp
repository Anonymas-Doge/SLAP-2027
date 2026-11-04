#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/u_int8.hpp>


#include "quadruped_types.hpp"

namespace quadruped::locomotion {

class Layer3Node : public rclcpp::Node {
public:
    explicit Layer3Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ROS 2 Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_locomotion_mode_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

    // ROS 2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_foot_targets_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_targets_;

    // Main Control Loop Timer (50 Hz)
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State Variables
    Twist2D current_cmd_vel_;
    LocomotionMode active_mode_{LocomotionMode::ALGORITHMIC};
    ImuData latest_imu_;
    JointState latest_joint_state_;
    std::array<float, NUM_JOINTS> previous_actions_{0.0f};
};

} // namespace quadruped::locomotion