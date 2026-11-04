


#include <memory>

#include "layer3_locomotion/layer3_manager.hpp"



int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<quadruped::locomotion::Layer3Node>());

    rclcpp::shutdown();
    return 0;
}