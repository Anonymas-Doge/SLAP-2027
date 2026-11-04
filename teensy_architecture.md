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
|  |  +----------------------------------+   +------------------------------------------------+  |  |
|  |                                                                                             |  |
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
* **Encoder Tracking Error Guard:** If $|q_{\text{target}, i} - q_{\text{actual}, i}| > \theta_{\mathit{maxerror}}$ for $> 50\text{ ms}$, disables motor drivers immediately.
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

## 4. Development Setup for Teensy 4.1

The recommended development environment for compiling Micro-ROS for Teensy 4.1 is **PlatformIO** (VS Code Extension).

### Install VS Code and PlatformIO
1. Download and install [Visual Studio Code](https://code.visualstudio.com/).
2. Open VS Code, navigate to Extensions (`Ctrl+Shift+X`), search for **PlatformIO IDE**, and click **Install**.

---

### Building and Flashing Firmware
1. Open PlatformIO Home in VS Code.
1. Connect the Teensy 4.1 to your development PC via USB.
2. Click the **PlatformIO Build** button (Checkmark icon in bottom status bar) or press `Ctrl+Alt+B`.
3. Click the **PlatformIO Upload** button (Right arrow icon in bottom status bar) or press `Ctrl+Alt+U`.

---

### Verify Connection with Jetson Micro-ROS Agent
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