# Teensy 4.1 Software Architecture & Micro-ROS Setup Guide

**Target Hardware:** Teensy 4.1 Microcontroller (ARM Cortex-M7 @ 600 MHz)  
**Role:** Real-Time Command Coordination, Analytical Inverse Kinematics, BNO085 Orientation Handling, & Hardware Safety  
**Communication Link:** Micro-ROS over USB CDC Serial (`3,000,000` Baud) to NVIDIA Jetson Orin Nano  

---

## 1. Teensy Software Architecture Overview

The Teensy 4.1 executes real-time command coordination and robot-level processing. A non-blocking main loop handles Micro-ROS and Layer 2 inverse kinematics. A periodic control task sends joint commands over CAN-FD through the MCP2518FD to 12 moteus r4.11 controllers and consumes their status feedback. Each moteus controller closes its own motor control loop, including PID; the Teensy does not implement a second joint PID loop or directly decode motor encoder signals. The BNO085 provides fused orientation reports to the Teensy.

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
|  | Layer 1: CAN-FD Commands, Feedback & Safety (Periodic Task)                                 |  |
|  |                                                                                             |  |
|  |  +----------------------------------+   +------------------------------------------------+  |  |
|  |  | Moteus CAN-FD Interface          |   | BNO085 Orientation Reports                     |  |  |
|  |  |  - MCP2518FD to 12 moteus r4.11  |   |  - Read fused rotation / accel / gyro          |  |  |
|  |  |  - Send joint targets; read state|   |  - Validate reports and coordinate frames      |  |  |
|  |  +----------------------------------+   +------------------------------------------------+  |  |
|  |                                                                                             |  |
|  |  +----------------------------------+   +------------------------------------------------+  |  |
|  |  | Joint Command Coordination       |   | Safety & Fault Interlocks                      |  |  |
|  |  |  - Map 12 joint targets to IDs   |   |  - CAN and controller fault monitoring         |  |  |
|  |  |  - Position/velocity/feedforward |   |  - Joint Limit Clamping                        |  |  |
|  |  |  - No Teensy-side motor PID      |   |  - Command-age watchdog and E-Stop             |  |  |
|  |  +----------------------------------+   +------------------------------------------------+  |  |
|  |                                                                                             |  |
|  +------------------------------+--------------------------------------------------------------+  |
|                                 |                 ^               ^                               |
+---------------------------------|-----------------|---------------|-------------------------------+
                                  |                 |               |                               
                                  v                 |               |                               
                                  +---[MCP2518FD]---+           [BNO085]                                           
                                        |     ^                                               
                                        v     |   
                                   [12 moteus r4.11]                                      
                                                                              

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

### Layer 1: CAN-FD I/O, Sensor Data & Safety (Periodic Task)
Runs as a scheduled, bounded task. Keep Micro-ROS callbacks and CAN transactions out of long blocking sections; choose command and status update rates to match the CAN-FD bus and controller configuration.

#### 1. Moteus CAN-FD Command and Feedback
* Communicates with 12 moteus r4.11 motor controllers through the MCP2518FD CAN-FD controller.
* Maps each joint to a configured moteus CAN ID and sends position targets (and optional velocity or torque feedforward fields, as configured).
* Reads controller status, including measured position, velocity, and faults, for `/joint_states` and safety checks. The Teensy does not sample motor encoder pins.
* The moteus controllers perform motor-level closed-loop control/PID. Do not implement a duplicate joint PID loop on the Teensy. CAN bitrates, command cadence, IDs, termination, and MCP2518FD driver configuration must match the physical network.

#### 2. Joint Command and Feedback Handling
* Converts the 12-element joint target vector into the units and per-joint limits expected by the configured moteus controllers.
* Publishes controller-reported position, velocity, and effort/torque telemetry where available. Mark unavailable or invalid telemetry rather than fabricating values.
* Validate commands, enforce joint bounds, detect stale feedback, and handle controller faults before continuing operation.

#### 3. BNO085 IMU Processing
* Reads BNO085 reports over SPI. Use its fused rotation-vector/orientation report instead of running a separate Mahony or Madgwick fusion filter on the Teensy.
* Publish orientation, angular velocity, and linear acceleration with correct frame IDs, units, and timestamps. Check report validity and handle sensor reset/reinitialization; configure report types and update rates for the application.

#### 4. Hardware Safety Interlocks
* **Joint Tracking Error Guard:** If valid moteus-reported position differs from target by more than $\theta_{\mathit{maxerror}}$ for $> 50\text{ ms}$, request the configured safe motor state and raise a fault.
* **Joint Angle Limit Clamping:** Hard-stops target commands if they fall outside physical leg bounds.
* **Command Watchdog:** If no valid command is received from the Jetson for the configured timeout, request the configured safe state from all moteus controllers. Choose and validate the safe action for the robot mechanics.
* **CAN/Controller Watchdog:** Detect missing responses, stale status, and reported moteus faults; transition to the configured safe state and report the fault.
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

Joint state and safety logic use moteus status telemetry carried over CAN-FD. Motor position PID and encoder-level feedback run on the moteus r4.11 controllers, not in Teensy firmware.

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
