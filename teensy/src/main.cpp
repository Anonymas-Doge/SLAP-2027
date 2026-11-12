#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <cmath>
#include <limits>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/time_sync.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/pose_array.h>
#include <geometry_msgs/msg/pose.h>

#include "layer1/layer1_config.hpp"
#include "layer1/layer1_manager.hpp"
#include "layer1/teensy_hardware.hpp"

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

using namespace quadruped;
using namespace quadruped::layer1;

namespace {
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_publisher_t joint_pub;
rcl_publisher_t imu_pub;
rcl_subscription_t joint_sub;
rcl_subscription_t mode_sub;
rcl_subscription_t estop_sub;
rcl_subscription_t feet_sub;
rcl_subscription_t pose_sub;

sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState joint_target_msg;
std_msgs__msg__UInt8 mode_msg;
std_msgs__msg__Bool estop_msg;
geometry_msgs__msg__PoseArray foot_target_msg;
geometry_msgs__msg__Pose pose_target_msg;
double target_position[NUM_JOINTS];
double target_velocity[NUM_JOINTS];
double target_effort[NUM_JOINTS];
rosidl_runtime_c__String target_names[NUM_JOINTS];
char target_name_storage[NUM_JOINTS][32];
geometry_msgs__msg__Pose foot_pose_storage[NUM_LEGS];
char foot_frame_storage[64];
double joint_position[NUM_JOINTS];
double joint_velocity[NUM_JOINTS];
double joint_effort[NUM_JOINTS];
double imu_orientation_covariance[9];
double imu_angular_covariance[9];
double imu_acceleration_covariance[9];

TeensyCanInterface can_interface;
Bno085SpiInterface imu_interface;
Layer1Manager layer1_manager(can_interface, imu_interface);
volatile bool physical_estop_pending = false;
uint8_t current_mode = 0;
uint32_t last_control_ms = 0;
uint32_t last_joint_publish_ms = 0;
uint32_t last_imu_publish_ms = 0;

void estop_isr() {
    if (config::DRIVER_ENABLE_PIN != 255)
        digitalWriteFast(config::DRIVER_ENABLE_PIN, !config::DRIVER_ENABLE_ACTIVE);
    physical_estop_pending = true;
}

void mode_callback(const void* input) {
    const auto* msg = static_cast<const std_msgs__msg__UInt8*>(input);
    if (msg->data <= 1) current_mode = msg->data;
}

void joint_target_callback(const void* input) {
    const auto* msg = static_cast<const sensor_msgs__msg__JointState*>(input);
    if (current_mode != 1 || msg->position.size != NUM_JOINTS) return;
    JointCommand command{};
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        const double p = msg->position.data[i];
        if (!std::isfinite(p)) return;
        command.position_target[i] = static_cast<float>(p);
        if (i < msg->velocity.size && std::isfinite(msg->velocity.data[i]))
            command.velocity_target[i] = static_cast<float>(msg->velocity.data[i]);
    }
    command.timestamp_ms = millis();
    layer1_manager.submitCommand(command, command.timestamp_ms);
}

void estop_callback(const void* input) {
    const auto* msg = static_cast<const std_msgs__msg__Bool*>(input);
    if (msg->data) layer1_manager.onEstopInterrupt();
}

void feet_callback(const void* input) {
    const auto* msg = static_cast<const geometry_msgs__msg__PoseArray*>(input);
    // Layer 2 consumes these targets; Layer 1 owns no Cartesian transforms.
    if (msg->poses.size == NUM_LEGS) foot_target_msg = *msg;
}

void pose_callback(const void* input) {
    // Retain the latest posture request for the Layer 2 integration point.
    pose_target_msg = *static_cast<const geometry_msgs__msg__Pose*>(input);
}

void publishJointState(uint32_t now) {
    const auto& state = layer1_manager.jointState();
    const auto& velocity_valid = layer1_manager.jointVelocityValid();
    const auto& effort_valid = layer1_manager.jointEffortValid();
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        joint_position[i] = state.encoder_valid[i] ? state.position[i] : unavailable;
        joint_velocity[i] = velocity_valid[i] ? state.velocity[i] : unavailable;
        joint_effort[i] = effort_valid[i] ? state.effort[i] : unavailable;
    }
    joint_state_msg.position.size = NUM_JOINTS;
    joint_state_msg.velocity.size = NUM_JOINTS;
    joint_state_msg.effort.size = NUM_JOINTS;
    int64_t epoch_ms = rmw_uros_epoch_millis();
    if (epoch_ms < 0) epoch_ms = now;
    joint_state_msg.header.stamp.sec = epoch_ms / 1000;
    joint_state_msg.header.stamp.nanosec = (epoch_ms % 1000) * 1000000UL;
    RCSOFTCHECK(rcl_publish(&joint_pub, &joint_state_msg, nullptr));
}

void publishImu(uint32_t now) {
    const auto& data = layer1_manager.imuData();
    int64_t epoch_ms = rmw_uros_epoch_millis();
    if (epoch_ms < 0) epoch_ms = now;
    imu_msg.header.stamp.sec = epoch_ms / 1000;
    imu_msg.header.stamp.nanosec = (epoch_ms % 1000) * 1000000UL;
    imu_msg.orientation.x = data.orientation.x;
    imu_msg.orientation.y = data.orientation.y;
    imu_msg.orientation.z = data.orientation.z;
    imu_msg.orientation.w = data.orientation.w;
    imu_msg.angular_velocity.x = data.angular_velocity.x;
    imu_msg.angular_velocity.y = data.angular_velocity.y;
    imu_msg.angular_velocity.z = data.angular_velocity.z;
    imu_msg.linear_acceleration.x = data.linear_acceleration.x;
    imu_msg.linear_acceleration.y = data.linear_acceleration.y;
    imu_msg.linear_acceleration.z = data.linear_acceleration.z;
    if (data.timestamp_ms == 0) {
        imu_msg.orientation_covariance[0] = -1.0;
        imu_msg.angular_velocity_covariance[0] = -1.0;
        imu_msg.linear_acceleration_covariance[0] = -1.0;
    }
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, nullptr));
}
}

void error_loop() {
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(3000000);
    set_microros_serial_transports(Serial);
    pinMode(config::ESTOP_PIN, config::ESTOP_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    if (config::DRIVER_ENABLE_PIN != 255) {
        pinMode(config::DRIVER_ENABLE_PIN, OUTPUT);
        digitalWrite(config::DRIVER_ENABLE_PIN, !config::DRIVER_ENABLE_ACTIVE);
    }
    attachInterrupt(digitalPinToInterrupt(config::ESTOP_PIN), estop_isr,
                    config::ESTOP_ACTIVE_LOW ? FALLING : RISING);
    const uint32_t now = millis();
    layer1_manager.init(now);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
    RCCHECK(rclc_node_init_default(&node, "teensy_control_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&joint_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states"));
    RCCHECK(rclc_publisher_init_default(&imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data"));
    RCCHECK(rclc_subscription_init_default(&mode_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/locomotion_mode"));
    RCCHECK(rclc_subscription_init_default(&joint_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_targets"));
    RCCHECK(rclc_subscription_init_default(&estop_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/e_stop"));
    RCCHECK(rclc_subscription_init_default(&feet_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseArray), "/foot_targets"));
    RCCHECK(rclc_subscription_init_default(&pose_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose), "/cmd_pose"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &mode_sub, &mode_msg, mode_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &joint_sub, &joint_target_msg, joint_target_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg, estop_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &feet_sub, &foot_target_msg, feet_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &pose_sub, &pose_target_msg, pose_callback, ON_NEW_DATA));
    (void)rmw_uros_sync_session(1000);

    joint_state_msg.position.data = joint_position;
    joint_state_msg.position.capacity = joint_state_msg.position.size = NUM_JOINTS;
    joint_state_msg.velocity.data = joint_velocity;
    joint_state_msg.velocity.capacity = joint_state_msg.velocity.size = NUM_JOINTS;
    joint_state_msg.effort.data = joint_effort;
    joint_state_msg.effort.capacity = joint_state_msg.effort.size = NUM_JOINTS;
    joint_target_msg.position.data = target_position;
    joint_target_msg.position.capacity = NUM_JOINTS;
    joint_target_msg.velocity.data = target_velocity;
    joint_target_msg.velocity.capacity = NUM_JOINTS;
    joint_target_msg.effort.data = target_effort;
    joint_target_msg.effort.capacity = NUM_JOINTS;
    joint_target_msg.name.data = target_names;
    joint_target_msg.name.capacity = NUM_JOINTS;
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        target_names[i].data = target_name_storage[i];
        target_names[i].capacity = sizeof(target_name_storage[i]);
    }
    foot_target_msg.poses.data = foot_pose_storage;
    foot_target_msg.poses.capacity = NUM_LEGS;
    foot_target_msg.header.frame_id.data = foot_frame_storage;
    foot_target_msg.header.frame_id.capacity = sizeof(foot_frame_storage);
    for (size_t i = 0; i < 9; ++i) {
        imu_msg.orientation_covariance[i] = imu_orientation_covariance[i];
        imu_msg.angular_velocity_covariance[i] = imu_angular_covariance[i];
        imu_msg.linear_acceleration_covariance[i] = imu_acceleration_covariance[i];
    }
    joint_state_msg.header.frame_id.data = const_cast<char*>(config::JOINT_STATE_FRAME_ID);
    joint_state_msg.header.frame_id.size = sizeof(config::JOINT_STATE_FRAME_ID) - 1;
    joint_state_msg.header.frame_id.capacity = sizeof(config::JOINT_STATE_FRAME_ID);
    imu_msg.header.frame_id.data = const_cast<char*>(config::IMU_FRAME_ID);
    imu_msg.header.frame_id.size = sizeof(config::IMU_FRAME_ID) - 1;
    imu_msg.header.frame_id.capacity = sizeof(config::IMU_FRAME_ID);
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
    if (physical_estop_pending) {
        noInterrupts(); physical_estop_pending = false; interrupts();
        layer1_manager.onEstopInterrupt();
    }
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - last_control_ms) >= config::CONTROL_PERIOD_US / 1000) {
        last_control_ms = now;
        layer1_manager.update(now);
    }
    if (static_cast<uint32_t>(now - last_joint_publish_ms) >= 20) {
        last_joint_publish_ms = now; publishJointState(now);
    }
    if (static_cast<uint32_t>(now - last_imu_publish_ms) >= 10) {
        last_imu_publish_ms = now; publishImu(now);
    }
}
