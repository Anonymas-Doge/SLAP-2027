

#include <memory>

#include "layer3_locomotion/layer3_manager.hpp"

namespace quadruped::locomotion {

    Layer3Node::Layer3Node(const rclcpp::NodeOptions& options) : rclcpp::Node("layer3", options) {
        RCLCPP_INFO(this->get_logger(), "Initializing...");
    }

} // namespace quadruped::locomotion
