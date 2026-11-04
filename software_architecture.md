# Quadruped Software Architecture Specification

**Platform:** NVIDIA Jetson Orin Nano + Teensy 4.1 Microcontroller  
**Control Paradigms:** Dual-Mode Architecture (Algorithmic Gait/IK & NVIDIA Isaac Lab RL Policy)  
**Middleware:** ROS 2 (Humble/Jazzy) + Micro-ROS over High-Speed USB Serial

---

## 1. System Overview

This document specifies the software architecture for a 12-DOF quadruped (robot dog). High-level autonomy, state estimation, gait planning, and neural network inference execute on an **NVIDIA Jetson Orin Nano**. Inverse kinematics, command coordination, BNO085 report handling, and robot-level safety checks execute on a **Teensy 4.1 microcontroller**. The Teensy communicates with 12 **moteus r4.11** motor controllers over CAN-FD through an **MCP2518FD** controller. Motor commutation, encoder feedback, and motor PID loops run on the moteus controllers; Teensy firmware does not implement a duplicate motor PID. Jetson-Teensy communication occurs via **Micro-ROS over USB Serial CDC**.

### Key Control Inputs

1. **2D Target Velocity Vector ($v_x, v_y, \omega_z$):** Controls forward, lateral, and rotational velocities during locomotion.
2. **Body Orientation Vector ($\phi, \theta, \psi, \Delta z$):** Controls roll, pitch, yaw, and height offsets of the chassis while in `STAND` mode.

---

## 2. Hardware-Software Topology

```
                                +---------------------------------------+
                                |      INPUT COMMANDS (Teleop/Nav2)     |
                                |  - 2D Velocity Vector (vx, vy, w)     |
                                |  - Body Orientation (roll, pitch, yaw)|
                                +-------------------+-------------------+
                                                    |
                                                    v
+---------------------------------------------------------------------------------------------------+
| JETSON ORIN NANO                                                                                  |
|                                                                                                   |
|  +---------------------------------------------------------------------------------------------+  |
|  | Layer 4: High-Level State Machine & Locomotion Switcher (10 - 50 Hz)                        |  |
|  |  - FSM Modes: OFF, INIT, SIT, STAND, LOCOMOTION, RECOVERY, ESTOP                            |  |
|  |  - Mode Selector: [ALGORITHMIC] <---> [NEURAL_RL]                                           |  |
|  +---------------------------------------+-----------------------------------------------------+  |
|                                          |                                                        |
|                    +---------------------+---------------------+                                  |
|                    | (Mode: ALGORITHMIC)                       | (Mode: NEURAL_RL)                |
|                    v                                           v                                  |
|  +---------------------------------------+   +-------------------------------------------------+  |
|  | Layer 3A: Classical Gait Generator    |   | Layer 3B: Isaac Lab RL Policy Node              |  |
|  |           & Foot Trajectory Planner   |   |           (TensorRT / ONNX Engine)              |  |
|  |  - Phase Clock (Trot, Walk, Bound)    |   |  - Obs:[v_cmd, w_gyro, proj_g, q, q_dot, prev_a]|  |
|  |  - 3D Bézier Swing Foot Curves        |   |  - Inferencing rate: 50 Hz                      |  |
|  |  - Output: Foot Targets (x,y,z)_4     |   |  - Output: Joint Position Targets (q_1..12)     |  |
|  +-------------------+-------------------+   +------------------------+------------------------+  |
|                      |                                                |                           |
+----------------------|------------------------------------------------|---------------------------+
                       |                                                |
         USB Serial: /foot_targets (50-100 Hz)              USB Serial: /joint_targets (50-100 Hz)
                       |                                                |
+----------------------|------------------------------------------------|---------------------------+
| TEENSY 4.1           v                                                v                           |
|  +---------------------------------------+                            |                           |
|  | Layer 2: Geometric IK & Pose Balance  |                            |                           |
|  |  - 3-DOF Analytical IK per leg        |                            |                           |
|  |  - IMU Roll/Pitch Dynamic Balancing   |                            |                           |
|  +-------------------+-------------------+                            |                           |
|                      |                                                |                           |
|                      +-------------------+----------------------------+                           |
|                                          | Target Joint Commands (q_1..12)                        |
|                                          v                                                        |
|  +---------------------------------------------------------------------------------------------+  |
|  | Layer 1: CAN-FD Commands, Feedback & Safety (Periodic Task)                                 |  |
|  |  - MCP2518FD CAN-FD link to 12 moteus r4.11 controllers                                     |  |
|  |  - Send joint targets; receive position, velocity, and fault telemetry                      |  |
|  |  - BNO085 fused orientation reports & robot-level safety checks                             |  |
|  +--------------------------------+------------------------------------------------------------+  |
|                                   |           ^              ^                                    |
+-----------------------------------|-----------|--------------|------------------------------------+
                                    |           |              |                                    
                                    v           |              |
                                     [MCP2518FD]            [BNO085]                                           
                                       |     ^                                               
                                       v     |   
                                  [12 moteus r4.11]                      
```

---

## 3. Layer Breakdown & Dual Locomotion Pathways

### Layer 4: High-Level State Machine & Command Router (Jetson Orin Nano)
* **Execution Environment:** ROS 2 Node on Linux ($10 - 50\text{ Hz}$).
* **Responsibilities:**
  * Arbitrates input streams from teleoperation (gamepad, web UI) or autonomous navigation (`Nav2`).
  * Manages global system states (`OFF`, `INIT`, `SIT`, `STAND`, `LOCOMOTION`, `RECOVERY`, `ESTOP`).
  * Routes velocity commands ($v_x, v_y, \omega_z$) during `LOCOMOTION` and stance pose vectors ($\phi, \theta, \psi, \Delta z$) during `STAND`.
  * Manages smooth mode transitions between `ALGORITHMIC` IK and `NEURAL_RL` control pathways.

### Layer 3: Trajectory & Action Generation (Jetson Orin Nano)

#### Option 3A: Classical Gait Generator (`ALGORITHMIC` Mode)
* **Execution Environment:** ROS 2 Node ($50 - 100\text{ Hz}$).
* **Responsibilities:**
  * Runs a cyclic phase clock maintaining leg duty cycle timing (e.g., alternating diagonal pairs for a trot gait).
  * Converts $v_x, v_y, \omega_z$ into linear stance vectors and 3D Bézier curves for swing phase foot clearance.
  * Outputs Cartesian positions $[x, y, z]$ for all 4 feet relative to their respective hip pivot frames to `/foot_targets`.

#### Option 3B: Isaac Lab Reinforcement Learning Policy (`NEURAL_RL` Mode)
* **Execution Environment:** C++ / Python ROS 2 Node leveraging TensorRT / ONNX Runtime ($50\text{ Hz}$).
* **Responsibilities:**
  * Assembles state observation vector $O_t$ using measured joint position/velocity telemetry from the moteus controllers, relayed by the Teensy via `/joint_states`:
    $$O_t = [v_{\text{cmd}}, \boldsymbol{\omega}_{\text{gyro}}, \mathbf{g}_{\text{projected}}, \mathbf{q}_{\text{actual}}, \mathbf{\dot{q}}_{\text{actual}}, \mathbf{a}_{t-1}]$$
  * Runs forward inference pass through an ONNX/TensorRT compiled actor network trained in NVIDIA Isaac Lab.
  * Scales output action vector $A_t$ to joint targets:
    $$\mathbf{q}_{\text{target}} = \mathbf{q}_{\text{default}} + K_{\text{scale}} \cdot A_t$$
  * Publishes target joint angles directly to `/joint_targets`, bypassing Layer 2 IK on the Teensy.

### Layer 2: Inverse Kinematics & Body Pose Balancing (Teensy 4.1)
* **Execution Environment:** Micro-ROS Executor / High-rate thread on Teensy ($100 - 500\text{ Hz}$).
* **Responsibilities:**
  * **In `ALGORITHMIC` Mode:** Receives Cartesian foot positions $[x, y, z]$ from Layer 3A, applies body orientation matrix transformations ($\phi, \theta, \psi, \Delta z$), integrates IMU-based dynamic balance adjustments, and solves analytical 3-DOF geometric IK for all 12 joints.
  * **In `NEURAL_RL` Mode (Passthrough):** Bypasses Cartesian IK calculations and passes incoming `/joint_targets` directly to Layer 1.

### Layer 1: CAN-FD I/O, Sensor Reports & Safety (Teensy 4.1)
* **Execution Environment:** Bounded periodic task coordinated with the Micro-ROS loop. Select command and feedback rates to fit the CAN-FD bus and moteus configuration.
* **Responsibilities:**
  * **Moteus CAN-FD interface:** Sends position targets (and configured velocity or torque feedforward fields) to the 12 configured moteus CAN IDs via MCP2518FD. Receives controller-reported position, velocity, and fault/status telemetry. Motor encoder sampling and motor PID remain inside the moteus controllers.
  * **Joint state publishing:** Relays measured moteus telemetry through `/joint_states` at a rate supported by bus capacity and application needs. Publish effort/current only when valid telemetry is available.
  * **BNO085 handling:** Reads configured fused rotation-vector/orientation and motion reports over SPI. The BNO085 performs sensor fusion; Teensy validates reports, handles sensor resets, and publishes `/imu/data` with correct units and frames.
  * **Robot-level safety enforcement:** Validates and bounds targets, monitors stale/missing CAN status and reported controller faults, and applies command-age and E-stop handling. A joint tracking-error guard may use moteus-reported position. The Teensy does not implement motor PID/FOC or motor thermal/current loops.

---

## 4. ROS 2 Topic Interface Architecture

| Topic Name | Message Type | Publisher Layer | Subscriber Layer | Rate (Hz) | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Teleop / Nav2 | Layer 4, 3A, 3B | 20–50 | Target 2D velocity ($v_x, v_y, \omega_z$). |
| `/cmd_pose` | `geometry_msgs/msg/Pose` | Teleop / User UI | Layer 4, Layer 2 | 20–50 | Target stance orientation ($\phi, \theta, \psi$) and height ($\Delta z$). |
| `/locomotion_mode` | `std_msgs/msg/UInt8` | Layer 4 (FSM) | Layer 3A, 3B, Teensy | On Change | Mode: `0: ALGORITHMIC`, `1: NEURAL_RL`. |
| `/foot_targets` | `geometry_msgs/msg/PoseArray` | Layer 3A (Gait) | Layer 2 (Teensy) | 50–100 | Cartesian foot targets $(x,y,z) \times 4$ relative to hips. |
| `/joint_targets` | `sensor_msgs/msg/JointState` | Layer 3B (RL) / Layer 2 | Layer 1 (Teensy) | 50–100 | Target joint angles ($\theta_1 \dots \theta_{12}$). |
| `/joint_states` | `sensor_msgs/msg/JointState` | Layer 1 (Teensy) | Layer 3B, Layer 4, RViz | 50–100 | Measured position/velocity and available effort telemetry reported by the moteus controllers over CAN-FD. |
| `/imu/data` | `sensor_msgs/msg/Imu` | Layer 1 (Teensy) | Layer 2, 3B, 4 | 100–200 | BNO085 fused orientation, gyro rates, and acceleration, with configured frames and timestamps. |
| `/robot_state` | `std_msgs/msg/String` | Layer 4 (FSM) | System-wide | On Change | Active robot state (`SIT`, `STAND`, `LOCOMOTION`, etc.). |
| `/e_stop` | `std_msgs/msg/Bool` | User / Layer 4 / Layer 1 | Layer 1 (Teensy) | Event | Immediate safety stop trigger. |

---

## 5. State Machine Control Logic

```
               +-----------------------------------+
               |               OFF                 |
               +-----------------+-----------------+
                                 |  [User Start]
                                 v
               +-----------------------------------+
               |               INIT                |
               |(Controller & Sensor Verification)|
               +-----------------+-----------------+
                                 |  [CAN controllers & BNO085 OK]
                                 v
               +-----------------------------------+
               |                SIT                |
               +--------+-----------------+--------+
                        |                 ^
         [Command Stand]|                 | [Command Sit]
                        v                 |
               +-----------------------------------+
               |               STAND               |
               |(Accepts Orientation Vector Inputs)|
               +--------+-----------------+--------+
                        |                 ^
        [cmd_vel > 0]   |                 | [cmd_vel == 0]
                        v                 |
               +-----------------------------------+
               |            LOCOMOTION             |
               |    (Accepts 2D Velocity Vector)   |
               +-----------------+-----------------+
                                 |  [IMU Roll/Pitch > Threshold OR CAN/Controller Fault]
                                 v
               +-----------------------------------+
               |             RECOVERY              |
               +-----------------------------------+
```

### State Processing Details

#### 1. `INIT` State
* **Active Input:** Hardware diagnostic check.
* **Processing:** Validates Micro-ROS, MCP2518FD/CAN-FD communication with all 12 moteus controllers, controller status and required joint references, and BNO085 report validity before enabling motion. Any calibration or zeroing procedure must follow the configured moteus and BNO085 setup.

#### 2. `STAND` State
* **Active Input:** `/cmd_pose` (`geometry_msgs/msg/Pose`).
* **Processing:** Velocities are set to zero. Target roll, pitch, yaw, and height ($\phi, \theta, \psi, \Delta z$) are routed to Layer 2 on the Teensy. The Teensy sends resulting joint targets to moteus, whose configured control loops track them; body pose balance uses BNO085 orientation feedback.

#### 3. `LOCOMOTION` State
* **Active Input:** `/cmd_vel` (`geometry_msgs/msg/Twist`).
* **Processing:** Depending on `/locomotion_mode`:
  * **If `ALGORITHMIC`:** Layer 3A calculates 3D Bézier foot trajectories based on $v_x, v_y, \omega_z$ and transmits `/foot_targets` to Layer 2 IK on the Teensy.
  * **If `NEURAL_RL`:** Layer 3B uses live moteus-reported joint positions ($\mathbf{q}$) and velocities ($\mathbf{\dot{q}}$) from `/joint_states` to populate the observation tensor $O_t$, evaluates the policy, and transmits `/joint_targets` directly to Layer 1.

---

## 6. Mode Switching Protocol & Safety Guarantees

### 6.1 Transition Sequence (`ALGORITHMIC` $\leftrightarrow$ `NEURAL_RL`)

To prevent dangerous joint jumps during pathway switching:

1. **Switch Request:** User or autonomy manager issues a mode switch service call.
2. **FSM Intercept:** Layer 4 validates that the robot is in a stable operational state (`STAND` or uniform `LOCOMOTION`).
3. **Joint Trajectory Blend ($0.3\text{ s}$ Window):**
   * Layer 4 freezes external velocity commands.
   * A cubic Hermite interpolation smoothly shifts active joint target commands from the current pathway's output to the target pathway's output using current moteus-reported position ($\mathbf{q}_{\text{actual}}$) as the initial anchor:
     $$\mathbf{q}_{\text{command}}(t) = (1 - \alpha(t)) \cdot \mathbf{q}_{\text{source}} + \alpha(t) \cdot \mathbf{q}_{\text{target}}, \quad \alpha(t) \in [0, 1]$$
4. **Teensy Pathway Switch:** Teensy receives the updated `/locomotion_mode` value and switches its internal command buffer routing.

### 6.2 Microcontroller Safety Interlocks

Regardless of whether control commands originate from classical IK or an AI policy:

* **Joint Tracking Error Guard:** If valid moteus-reported position differs from target by more than $\theta_{\text{threshold}}$ for longer than the configured interval, Layer 1 requests the configured safe motor state and raises a fault.
* **Joint Limits Saturation:** Teensy hard-clamps target joint commands to configured physical software limits and joint reference offsets.
* **Velocity/Acceleration Caps:** Filters out step discontinuities or `NaN` outputs produced by policy anomalies before targets are sent over CAN-FD to moteus.
* **Command and CAN Watchdogs:** On stale Micro-ROS commands, missing CAN responses, or stale controller status, request the configured safe state from the moteus controllers and report the fault. Timeout and safe-state behavior must be configured for the robot mechanics.
* **Hardware E-Stop Pin:** A dedicated digital GPIO wire directly triggers driver disabling, bypassing Linux/ROS software stacks.

---

## 7. Isaac Lab Model Deployment Workflow

To deploy an RL model trained in NVIDIA Isaac Lab into this architecture:

1. **Observation Space Matching:** Ensure Isaac Lab observation space includes joint positions ($\mathbf{q}$) and velocities ($\mathbf{\dot{q}}$) formatted to match the `/joint_states` telemetry relayed from moteus by Layer 1.
2. **Model Export:** Export the trained policy network from PyTorch to ONNX format inside Isaac Lab:
   ```python
   torch.onnx.export(
       policy_model, 
       dummy_observation_input, 
       "quadruped_policy.onnx",
       input_names=["observation"],
       output_names=["action"]
   )
   ```
3. **TensorRT Compilation (Jetson Orin Nano):** Compile the ONNX model into an optimized TensorRT engine on the Jetson to leverage FP16 Tensor Cores for fast execution ($< 2\text{ ms}$ per step):
   ```bash
   trtexec --onnx=quadruped_policy.onnx --saveEngine=quadruped_policy.engine --fp16
   ```
4. **ROS 2 Integration:** Load `quadruped_policy.engine` into the Layer 3B C++ node. The node constructs $O_t$ using live moteus-reported joint feedback from `/joint_states` and BNO085 data from `/imu/data`, runs inference, and publishes output actions to `/joint_targets`.
