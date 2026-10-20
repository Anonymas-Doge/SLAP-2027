# Teensy 4.1 Software Architecture & Micro-ROS Setup Guide

**Target Hardware:** Teensy 4.1 Microcontroller (ARM Cortex-M7 @ 600 MHz)  
**Role:** Hard Real-Time Control, Encoder Feedback Decoding, Analytical Inverse Kinematics, IMU Filtering, & Hardware Safety  
**Communication Link:** Micro-ROS over USB CDC Serial (`3,000,000` Baud) to NVIDIA Jetson Orin Nano  

---

## 1. Teensy Software Architecture Overview

The Teensy 4.1 executes all hard real-time tasks. It operates on a hybrid architecture: high-frequency timer interrupt service routines (ISRs) handle motor PID control and encoder decoding, while a non-blocking main execution loop processes Micro-ROS client tasks and Layer 2 Inverse Kinematics.

```
+---------------------------------------------------------------------------------------------------+
| TEENSY 4.1 MICROCONTROLLER                                                                        |
|                                                                                                   |
|  +---------------------------------------------------------------------------------------------+  |
|  | Micro-ROS Client Transport & Executor (Main Loop / 100 - 200 Hz)                            |  |
|  |  - Subscribes: /foot_targets, /joint_targets, /locomotion_mode, /cmd_pose, /e_stop          |  |
|  |  - Publishes:  /joint_states, /imu/data                                                     |  |
|  +---------------------------------------+-----------------------------------------------------+  |
|                                          |                                                        |
|                    +---------------------+---------------------+                                  |
|                    | (Mode 0: ALGORITHMIC)                     | (Mode 1: NEURAL_RL)              |
|                    v                                           v                                  |
|  +---------------------------------------+   +-------------------------------------------------+  |
|  | Layer 2: Geometric IK & Pose Balance  |   | Layer 2: Direct Passthrough Buffer              |  |
|  |  - 3-DOF Geometric IK (4 Legs)        |   |  - Routes incoming /joint_targets directly      |  |
|  |  - IMU Orientation Body Compensation  |   |    to Layer 1 target array                      |  |
|  |  - Output: Target Joint Angles (q_tgt)|   |  - Output: Target Joint Angles (q_tgt)          |  |
|  +-------------------+-------------------+   +------------------------+------------------------+  |
|                      |                                                |                           |
|                      +-------------------+----------------------------+                           |
|                                          | Joint Command Vector (q_target)                        |
|                                          v                                                        |
|  +---------------------------------------------------------------------------------------------+  |
|  | Layer 1: Hardware Control, Encoder Processing & Safety (Timer ISR @ 1 kHz)                  |  |
|  |                                                                                             |  |
|  |  +----------------------------------+   +------------------------------------------------+  |  |
|  |  | Encoder Signal Processing        |   | IMU Sensor Filtering                           |  |  |
|  |  |  - Read Quadrature / SPI / CAN   |   |  - Sample Accelerometer & Gyroscope            |  |  |
|  |  |  - Compute q_actual & q_dot      |   |  - Run Mahony / Madgwick Filter                |  |  |
|  |  +---------------+------------------+   +-----------------------+------------------------+  |  |
|  |                  |                                              |                           |  |
|  |                  v                                              v                           |  |
|  |  +----------------------------------+   +------------------------------------------------+  |  |
|  |  | Closed-Loop Motor Control        |   | Safety & Fault Interlocks                      |  |  |
|  |  |  - Joint Position/Velocity PID   |   |  - Tracking Error: |q_tgt - q_act| > threshold |  |  |
|  |  |  - Feedforward Torque Output     |   |  - Joint Limit Clamping                        |  |  |
|  |  |  - Output to Motor Drivers/Servos|   |  - Serial Watchdog (> 50 ms timeout -> E-Stop) |  |  |
|  |  +----------------------------------+   +------------------------------------------------+  |  |
|  |                                                                                             |  |
|  +------------------------------+--------------------------------------------------------------+  |
|                                 |                 ^               ^                               |
+---------------------------------|-----------------|---------------|-------------------------------+
                                  |                 |               |                               
                                  v                 |               |                               
                              [Motors]            [IMU]     [Motor Encoders]                        

```

---

## 2. Layer Breakdown & Implementation

### Layer 2: Inverse Kinematics & Pose Balancing ($100 - 500\text{ Hz}$)
Executes within the main loop or a high-priority software timer.

1. **Pathway 0 (`ALGORITHMIC` Mode):**
   * Receives 4 Cartesian foot target coordinates $(x_i, y_i, z_i)$ from `/foot_targets`.
   * Applies chassis stance transformation matrix $T(\phi, \theta, \psi, \Delta z)$ received from `/cmd_pose`.
   * Injects IMU roll/pitch feedback compensation to keep the body level.
   * Solves closed-form 3-DOF geometric IK for each leg to derive target joint angles:
     $$\mathbf{q}_{\text{target}} = [\theta_{1}, \theta_{2}, \dots, \theta_{12}]^T$$

2. **Pathway 1 (`NEURAL_RL` Passthrough Mode):**
   * Bypasses geometric IK calculations entirely.
   * Copies incoming `/joint_targets` from the Jetson's RL policy node directly into the joint command vector $\mathbf{q}_{\text{target}}$.

---

### Layer 1: Hardware I/O, PID Control & Safety Interrupt ($1\text{ kHz}$)
Runs inside a dedicated hardware timer interrupt service routine (e.g., Teensy `IntervalTimer` at $1000\text{ Hz}$).

#### 1. Motor Encoder Processing
* Samples hardware quadrature pins, magnetic absolute SPI encoders, or CAN telemetry.
* Calculates actual joint positions $\mathbf{q}_{\text{actual}}$ and computes filtered angular velocity $\mathbf{\dot{q}}_{\text{actual}}$ using a digital low-pass filter:
  $$\mathbf{\dot{q}}[k] = (1 - \alpha) \cdot \mathbf{\dot{q}}[k-1] + \alpha \cdot \frac{\mathbf{q}[k] - \mathbf{q}[k-1]}{\Delta t}$$

#### 2. Closed-Loop Motor PID Control
* Evaluates error for each joint: $e_i = q_{\text{target}, i} - q_{\text{actual}, i}$.
* Computes PID torque/PWM output:
  $$u_i(t) = K_p e_i(t) + K_i \int e_i(t) dt + K_d \frac{de_i(t)}{dt} + \tau_{\text{feedforward}}$$

#### 3. IMU Processing ($200 - 500\text{ Hz}$)
* Reads 6-axis / 9-axis raw IMU register data over SPI ($20\text{ MHz}$) or $I^2C$ ($400\text{ kHz}$).
* Runs a Mahony or Madgwick orientation filter to compute body quaternions $\mathbf{q}_{\text{imu}}$, roll/pitch angles, and angular velocity rates $\boldsymbol{\omega}_{\text{gyro}}$.

#### 4. Hardware Safety Interlocks
* **Encoder Tracking Error Guard:** If $|q_{\text{target}, i} - q_{\text{actual}, i}| > \theta_{\text{max\_error}}$ for $> 50\text{ ms}$, disables motor drivers immediately.
* **Joint Angle Limit Clamping:** Hard-stops target commands if they fall outside physical leg bounds.
* **Serial Watchdog:** If no valid Micro-ROS heartbeat is received from the Jetson for $> 50\text{ ms}$, forces the robot into a passive damping hold or controlled sit.
* **E-Stop Line:** Dedicated digital interrupt pin that triggers instant hardware driver shutdown.

---

## 3. Micro-ROS Topic Specifications

| Topic Name | Message Type | Direction | Rate | Usage |
| :--- | :--- | :--- | :--- | :--- |
| `/foot_targets` | `geometry_msgs/msg/PoseArray` | Sub | $50-100\text{ Hz}$ | Cartesian foot positions $(x,y,z) \times 4$ (`ALGORITHMIC` mode). |
| `/joint_targets` | `sensor_msgs/msg/JointState` | Sub | $50-100\text{ Hz}$ | Direct 12-joint target angles (`NEURAL_RL` mode). |
| `/locomotion_mode` | `std_msgs/msg/UInt8` | Sub | Event | Switches between `0` (`ALGORITHMIC`) and `1` (`NEURAL_RL`). |
| `/cmd_pose` | `geometry_msgs/msg/Pose` | Sub | $20-50\text{ Hz}$ | Body posture adjustment vectors $(\phi, \theta, \psi, \Delta z)$. |
| `/e_stop` | `std_msgs/msg/Bool` | Sub | Event | Triggers emergency stop. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Pub | $50-100\text{ Hz}$ | Encoder position ($\mathbf{q}$), velocity ($\mathbf{\dot{q}}$), and effort ($\boldsymbol{\tau}$). |
| `/imu/data` | `sensor_msgs/msg/Imu` | Pub | $100-200\text{ Hz}$ | Orientation quaternion, angular rates, linear acceleration. |

---

## 4. Micro-ROS Installation & Development Setup for Teensy 4.1

The recommended development environment for compiling Micro-ROS for Teensy 4.1 is **PlatformIO** (VS Code Extension).

### Step 1: Install VS Code and PlatformIO
1. Download and install [Visual Studio Code](https://code.visualstudio.com/).
2. Open VS Code, navigate to Extensions (`Ctrl+Shift+X`), search for **PlatformIO IDE**, and click **Install**.

---

### Step 2: Create PlatformIO Project
1. Open PlatformIO Home in VS Code.
2. Click **New Project**.
3. Set **Name:** `teensy_quadruped_firmware`.
4. Set **Board:** `Teensy 4.1`.
5. Set **Framework:** `Arduino`.

---

### Step 3: Configure `platformio.ini`
Replace the contents of your project's `platformio.ini` file with the following configuration to link `micro_ros_platformio`:

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino

; Serial transport configuration for Micro-ROS over USB CDC
board_microros_transport = serial

; Dependencies
lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio.git
    PWMServo ; or CAN / Motor driver libraries
    Adafruit BNO055 ; or your IMU library

; Compiler Flags
build_flags =
    -D TEENSY_DUAL_SERIAL
    -O3

; Serial Monitor Setup
monitor_speed = 3000000
```

---

### Step 4: Firmware Implementation (`src/main.cpp`)

Below is a complete starter template implementing Micro-ROS transport, topic publishers, topic subscribers, and the 1 kHz control ISR loop on the Teensy 4.1.

```cpp
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
```

---

### Step 5: Build and Flash Firmware
1. Connect the Teensy 4.1 to your development PC via USB.
2. Click the **PlatformIO Build** button (Checkmark icon in bottom status bar) or press `Ctrl+Alt+B`.
3. Click the **PlatformIO Upload** button (Right arrow icon in bottom status bar) or press `Ctrl+Alt+U`.

---

### Step 6: Verify Connection with Jetson Micro-ROS Agent
Once the firmware is flashed and the Teensy is connected to the Jetson Orin Nano, run the Micro-ROS agent on the Jetson:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 3000000
```

Verify that the topics appear in the ROS 2 graph on the Jetson:

```bash
ros2 topic list
# Output should include:
# /joint_states
# /imu/data
# /joint_targets
# /locomotion_mode
```