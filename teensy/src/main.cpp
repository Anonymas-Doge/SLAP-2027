#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int8.h>
#include <geometry_msgs/msg/pose_array.h>

// Macros for error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Global ROS Objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Publishers
rcl_publisher_t joint_pub;
rcl_publisher_t imu_pub;
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__Imu imu_msg;

// Subscribers
rcl_subscription_t joint_sub;
rcl_subscription_t mode_sub;
sensor_msgs__msg__JointState joint_target_msg;
std_msgs__msg__UInt8 mode_msg;

// State Variables
uint8_t current_mode = 0; // 0: ALGORITHMIC, 1: NEURAL_RL
float target_joint_positions[12] = {0.0};
float actual_joint_positions[12] = {0.0};
float actual_joint_velocities[12] = {0.0};

IntervalTimer controlTimer;

void error_loop() {
    while(1) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

// ISR: Layer 1 Control Loop (1 kHz)
void controlLoopISR() {
    // 1. Read Motor Encoders -> Populate actual_joint_positions & actual_joint_velocities
    // 2. Execute Joint PID Loops using target_joint_positions
    // 3. Perform Encoder Tracking Error & Safety Checks
}

// Subscriber Callback: Mode Switch
void mode_callback(const void * msin) {
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msin;
    current_mode = msg->data;
}

// Subscriber Callback: Joint Targets (NEURAL_RL mode)
void joint_target_callback(const void * msin) {
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msin;
    if (current_mode == 1) { // NEURAL_RL mode passthrough
        for (int i = 0; i < 12 && i < msg->position.size; i++) {
            target_joint_positions[i] = msg->position.data[i];
        }
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(3000000);

    // Set Micro-ROS transport to USB Serial
    set_microros_serial_transports(Serial);

    delay(2000);

    allocator = rcl_get_default_allocator();

    // Initialize support and node
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "teensy_control_node", "", &support));

    // Initialize Publishers
    RCCHECK(rclc_publisher_init_default(
        &joint_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states"));

    RCCHECK(rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"));

    // Initialize Subscribers
    RCCHECK(rclc_subscription_init_default(
        &mode_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "/locomotion_mode"));

    RCCHECK(rclc_subscription_init_default(
        &joint_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_targets"));

    // Initialize Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &mode_sub, &mode_msg, &mode_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &joint_sub, &joint_target_msg, &joint_target_callback, ON_NEW_DATA));

    // Start Layer 1 Timer Interrupt (1000 Hz = 1000 microseconds)
    controlTimer.begin(controlLoopISR, 1000);
}

void loop() {
    // Spin Micro-ROS executor to handle incoming/outgoing messages
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    // Publish Joint States & IMU Telemetry (e.g., at 50 Hz)
    static uint32_t last_pub = 0;
    if (millis() - last_pub > 20) {
        last_pub = millis();
        // Populate and publish joint_state_msg and imu_msg here
        RCSOFTCHECK(rcl_publish(&joint_pub, &joint_state_msg, NULL));
    }
}