# Quadruped Software Architecture Specification

**Platform:** NVIDIA Jetson Orin Nano + Teensy 4.1 Microcontroller  
**Control Paradigms:** Dual-Mode Architecture (Algorithmic Gait/IK & NVIDIA Isaac Lab RL Policy)  
**Middleware:** ROS 2 (Humble/Jazzy) + Micro-ROS over High-Speed USB Serial

---

## 1. System Overview

This document specifies the software architecture for a 12-DOF quadruped (robot dog). High-level autonomy, state estimation, gait planning, and neural network inference execute on an **NVIDIA Jetson Orin Nano**. Low-level motor PID/FOC control, encoder feedback processing, inverse kinematics, IMU filtering, and hardware safety checks execute on a **Teensy 4.1 microcontroller**. Communication between the two systems occurs via **Micro-ROS over USB Serial CDC**.

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
|  | Layer 1: Hardware I/O, PID Control & Safety (500 Hz - 1 kHz)                                |  |
|  |  - Motor Encoder Reading (Quadrature/SPI/CAN) -> Joint Position (q) & Velocity (q_dot)      |  |
|  |  - Closed-Loop Position/Velocity PID & Feedforward Torque Control                           |  |
|  |  - Fast IMU Sampling (Madgwick/Mahony Filter) & Encoder Tracking Safety Interlocks          |  |
|  +--------------------------------+------------------------------------------------------------+  |
|                                   |           ^              ^                                    |
+-----------------------------------|-----------|--------------|------------------------------------+
                                    |           |              |                                    
                                    v           |              |                                    
                                 [Motors]     [IMU]    [Motor Encoders]                             
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
  * Assembles state observation vector $O_t$ using actual joint encoder data ($\mathbf{q}_{\text{actual}}, \mathbf{\dot{q}}_{\text{actual}}$) received from Layer 1 via `/joint_states`:
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

### Layer 1: Hardware I/O, PID Control & Safety (Teensy 4.1)
* **Execution Environment:** Timer Interrupt Service Routine (ISR) / Hard Real-Time ($500\text{ Hz} - 1\text{ kHz}$).
* **Responsibilities:**
  * **Motor Encoder Processing:** 
    * Reads raw motor encoder hardware signals (Quadrature, SPI/SSI absolute magnetic encoders, or CAN telemetry).
    * Decodes actual joint angles $\mathbf{q}_{\text{actual}}$ and computes filtered angular velocities $\mathbf{\dot{q}}_{\text{actual}}$ via digital low-pass filtering.
    * Streams encoder positions, velocities, and estimated currents to the Jetson via `/joint_states` at $50 - 100\text{ Hz}$.
  * **Closed-Loop Joint Control:** Executes real-time PID/FOC loops using error vectors between target joint commands ($\mathbf{q}_{\text{target}}$) and measured encoder feedback ($\mathbf{q}_{\text{actual}}$).
  * **IMU Processing:** Samples body IMU via SPI/I2C at high rates and applies complementary/Madgwick orientation filtering.
  * **Hardware Safety Enforcement:** Encoder tracking error detection (detects mechanical stalls or motor decoupling), software joint limit enforcement, thermal/current limiting, and loss-of-signal E-stops.

---

## 4. ROS 2 Topic Interface Architecture

| Topic Name | Message Type | Publisher Layer | Subscriber Layer | Rate (Hz) | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Teleop / Nav2 | Layer 4, 3A, 3B | 20–50 | Target 2D velocity ($v_x, v_y, \omega_z$). |
| `/cmd_pose` | `geometry_msgs/msg/Pose` | Teleop / User UI | Layer 4, Layer 2 | 20–50 | Target stance orientation ($\phi, \theta, \psi$) and height ($\Delta z$). |
| `/locomotion_mode` | `std_msgs/msg/UInt8` | Layer 4 (FSM) | Layer 3A, 3B, Teensy | On Change | Mode: `0: ALGORITHMIC`, `1: NEURAL_RL`. |
| `/foot_targets` | `geometry_msgs/msg/PoseArray` | Layer 3A (Gait) | Layer 2 (Teensy) | 50–100 | Cartesian foot targets $(x,y,z) \times 4$ relative to hips. |
| `/joint_targets` | `sensor_msgs/msg/JointState` | Layer 3B (RL) / Layer 2 | Layer 1 (Teensy) | 50–100 | Target joint angles ($\theta_1 \dots \theta_{12}$). |
| `/joint_states` | `sensor_msgs/msg/JointState` | Layer 1 (Teensy) | Layer 3B, Layer 4, RViz | 50–100 | Measured motor encoder positions ($\mathbf{q}$), velocities ($\mathbf{\dot{q}}$), and effort ($\boldsymbol{\tau}$). |
| `/imu/data` | `sensor_msgs/msg/Imu` | Layer 1 (Teensy) | Layer 2, 3B, 4 | 100–200 | Filtered IMU quaternion, gyro rates, and acceleration. |
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
               |(Encoder Calib & Zero Verification)|
               +-----------------+-----------------+
                                 |  [Sensors & Encoders OK]
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
                                 |  [IMU Roll/Pitch > Threshold OR Encoder Fault]
                                 v
               +-----------------------------------+
               |             RECOVERY              |
               +-----------------------------------+
```

### State Processing Details

#### 1. `INIT` State
* **Active Input:** Hardware diagnostic check.
* **Processing:** Validates communication links, verifies zero references on all 12 motor encoders, checks encoder signal integrity, and zeros IMU sensor bias before enabling motor power.

#### 2. `STAND` State
* **Active Input:** `/cmd_pose` (`geometry_msgs/msg/Pose`).
* **Processing:** Velocities are set to zero. Target roll, pitch, yaw, and height ($\phi, \theta, \psi, \Delta z$) are routed to Layer 2 on the Teensy. Layer 1 uses closed-loop encoder PID control to maintain foot plant locations while holding the chassis pose.

#### 3. `LOCOMOTION` State
* **Active Input:** `/cmd_vel` (`geometry_msgs/msg/Twist`).
* **Processing:** Depending on `/locomotion_mode`:
  * **If `ALGORITHMIC`:** Layer 3A calculates 3D Bézier foot trajectories based on $v_x, v_y, \omega_z$ and transmits `/foot_targets` to Layer 2 IK on the Teensy.
  * **If `NEURAL_RL`:** Layer 3B uses live joint encoder positions ($\mathbf{q}$) and velocities ($\mathbf{\dot{q}}$) from `/joint_states` to populate the observation tensor $O_t$, evaluates the policy, and transmits `/joint_targets` directly to Layer 1.

---

## 6. Mode Switching Protocol & Safety Guarantees

### 6.1 Transition Sequence (`ALGORITHMIC` $\leftrightarrow$ `NEURAL_RL`)

To prevent dangerous joint jumps during pathway switching:

1. **Switch Request:** User or autonomy manager issues a mode switch service call.
2. **FSM Intercept:** Layer 4 validates that the robot is in a stable operational state (`STAND` or uniform `LOCOMOTION`).
3. **Joint Trajectory Blend ($0.3\text{ s}$ Window):**
   * Layer 4 freezes external velocity commands.
   * A cubic Hermite interpolation smoothly shifts active joint target commands from the current pathway's output to the target pathway's output using actual encoder feedback ($\mathbf{q}_{\text{actual}}$) as the initial anchor:
     $$\mathbf{q}_{\text{command}}(t) = (1 - \alpha(t)) \cdot \mathbf{q}_{\text{source}} + \alpha(t) \cdot \mathbf{q}_{\text{target}}, \quad \alpha(t) \in [0, 1]$$
4. **Teensy Pathway Switch:** Teensy receives the updated `/locomotion_mode` value and switches its internal command buffer routing.

### 6.2 Microcontroller Safety Interlocks

Regardless of whether control commands originate from classical IK or an AI policy:

* **Encoder Tracking Error Guard:** If $|\mathbf{q}_{\text{target}} - \mathbf{q}_{\text{actual}}| > \theta_{\text{threshold}}$ for longer than $50\text{ ms}$ (indicating a stall, leg snag, or encoder failure), Layer 1 immediately halts motion and triggers `ESTOP`.
* **Joint Limits Saturation:** Teensy hard-clamps target joint commands to physical software limits derived from encoder calibration values.
* **Velocity/Acceleration Caps:** Filters out step discontinuities or `NaN` outputs produced by policy anomalies before commands reach PID loops.
* **Serial Watchdog:** If Micro-ROS serial connection drops for $> 50\text{ ms}$, Teensy automatically defaults to an **Algorithmic Standing Hold** or **Controlled Sit**.
* **Hardware E-Stop Pin:** A dedicated digital GPIO wire directly triggers driver disabling, bypassing Linux/ROS software stacks.

---

## 7. Isaac Lab Model Deployment Workflow

To deploy an RL model trained in NVIDIA Isaac Lab into this architecture:

1. **Observation Space Matching:** Ensure Isaac Lab observation space includes joint positions ($\mathbf{q}$) and velocities ($\mathbf{\dot{q}}$) formatted to match the `/joint_states` array published by Layer 1.
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
4. **ROS 2 Integration:** Load `quadruped_policy.engine` into the Layer 3B C++ node. The node constructs $O_t$ using live encoder feedback from `/joint_states` and IMU data from `/imu/data`, runs inference, and publishes output actions to `/joint_targets`.