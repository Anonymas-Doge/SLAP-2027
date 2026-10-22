#pragma once

#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "quadruped_fsm/fsm_state_base.hpp"
#include "quadruped_fsm/command_mux.hpp"
#include "quadruped_fsm/safety_watchdog.hpp"
#include "quadruped_fsm/quadruped_types.hpp"

namespace quadruped::fsm {

class Layer4Node : public rclcpp::Node {
public:
    explicit Layer4Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void changeState(RobotState new_state);
    void setLocomotionMode(LocomotionMode mode);

    // Getters for states
    CommandMux& getMux() { return command_mux_; }
    SafetyWatchdog& getWatchdog() { return watchdog_; }

private:
    // ROS 2 Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav2_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_teleop_pose_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

    // ROS 2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_cmd_pose_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_locomotion_mode_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_robot_state_;

    // Loop Timer (50 Hz)
    rclcpp::TimerBase::SharedPtr fsm_timer_;

    // FSM Core
    std::unordered_map<RobotState, std::shared_ptr<FSMState>> states_;
    std::shared_ptr<FSMState> current_state_handler_{nullptr};
    RobotState current_state_enum_{RobotState::OFF};
    LocomotionMode current_mode_{LocomotionMode::ALGORITHMIC};

    // Subsystems
    CommandMux command_mux_;
    SafetyWatchdog watchdog_;

    void timerCallback();
    void publishCommands();
};

} // namespace quadruped::fsm