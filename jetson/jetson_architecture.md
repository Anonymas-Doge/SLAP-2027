# Jetson Orin Nano Software Architecture & ROS 2 Setup Guide

**Target Board:** NVIDIA Jetson Orin Nano  
**Operating System:** Ubuntu 24.04 LTS (Noble Numbat)  
**JetPack Version:** JetPack 7.2.1 (Jetson Linux 39.x)  
**ROS 2 Distribution:** ROS 2 Jazzy Jalisco (LTS for Ubuntu 24.04)  
**Interface Bridge:** Micro-ROS Agent over USB Serial CDC (`/dev/ttyACM0`)

---

## 1. Jetson Software Architecture

The Jetson Orin Nano handles high-level state decisions, path planning, user teleoperation routing, gait generation, and neural network inference (Isaac Lab RL policies).

```
+---------------------------------------------------------------------------------------------------+
| JETSON ORIN NANO SOFTWARE STACK                                                                   |
|                                                                                                   |
|  +---------------------------------------------------------------------------------------------+  |
|  | Layer 4: High-Level State Machine & Command Router (10 - 50 Hz)                             |  |
|  |  - FSM Modes: OFF, INIT, SIT, STAND, LOCOMOTION, RECOVERY, ESTOP                            |  |
|  |  - Teleop Arbitrator: Gamepad Joy / Nav2 Path Planner                                       |  |
|  |  - Locomotion Mode Selector: [ALGORITHMIC] <---> [NEURAL_RL]                                |  |
|  +---------------------------------------+-----------------------------------------------------+  |
|                                          |                                                        |
|                    +---------------------+---------------------+                                  |
|                    | (Mode: ALGORITHMIC)                       | (Mode: NEURAL_RL)                |
|                    v                                           v                                  |
|  +---------------------------------------+   +-------------------------------------------------+  |
|  | Layer 3A: Classical Gait Generator    |   | Layer 3B: Isaac Lab RL Policy Node              |  |
|  |           & Foot Trajectory Planner   |   |           (TensorRT / CUDA FP16 Engine)         |  |
|  |  - Phase Clock (Trot, Walk, Bound)    |   |  - Obs:[v_cmd, w_gyro, proj_g, q, q_dot, prev_a]|  |
|  |  - 3D Bézier Swing Foot Trajectories  |   |  - Inference: 50 Hz via TensorRT                |  |
|  |  - Output: Foot Targets (x,y,z)_4     |   |  - Output: Joint Targets (q_1..12)              |  |
|  +-------------------+-------------------+   +------------------------+------------------------+  |
|                      |                                                |                           |
|                      +-------------------+----------------------------+                           |
|                                          | Mode Multiplexer & Blending                            |
|                                          v                                                        |
|  +---------------------------------------------------------------------------------------------+  |
|  | Micro-ROS Agent Node (USB Serial Transport Bridge)                                          |  |
|  |  - Serial Port: /dev/ttyACM0 @ 3,000,000 Baud                                               |  |
|  |  - Publishes: /foot_targets OR /joint_targets, /locomotion_mode                             |  |
|  |  - Subscribes: /joint_states, /imu/data                                                     |  |
|  +---------------------------------------+-----------------------------------------------------+  |
|                                          |                                                        |
+------------------------------------------|--------------------------------------------------------+
                                           |
                              USB CDC Serial (Micro-ROS)
                                           |
                                           v
                            [ Teensy 4.1 Microcontroller ]
```

---

## 2. Component Breakdown

### 2.1 Layer 4: State Machine & Command Router (10 – 50 Hz)
* **FSM Control Node:** Manages system-wide lifecycle transitions (`OFF`, `INIT`, `SIT`, `STAND`, `LOCOMOTION`, `RECOVERY`, `ESTOP`).
* **Input Switching:**
  * In **`STAND` mode**, routes 4-DOF body stance requests ($\phi_{\text{roll}}, \theta_{\text{pitch}}, \psi_{\text{yaw}}, \Delta z_{\text{height}}$) via `/cmd_pose`.
  * In **`LOCOMOTION` mode**, routes target 2D velocity vectors ($v_x, v_y, \omega_z$) via `/cmd_vel`.
* **Locomotion Mode Switcher (Mux):** Smoothly interpolates joint states over a $0.3\text{-second}$ window during transitions between `ALGORITHMIC` IK mode and `NEURAL_RL` AI mode.

### 2.2 Layer 3A: Classical Gait Generator (50 – 100 Hz)
* **Phase Clock:** Tracks stance/swing timing for trot, walk, or crawl gaits.
* **Foot Trajectory Calculation:** Generates 3D Bézier swing-foot curves for ground clearance and linear ground-matching vectors for stance feet.
* **Output:** Publishes Cartesian foot targets relative to hip pivots ($[x, y, z] \times 4$) on `/foot_targets`.

### 2.3 Layer 3B: Isaac Lab RL Inference Engine (50 Hz)
* **Hardware Acceleration:** Uses CUDA & TensorRT FP16 precision on Orin Nano GPUs.
* **Observation Vector Construction:**
  $$O_t = [v_{\text{cmd}}, \boldsymbol{\omega}_{\text{gyro}}, \mathbf{g}_{\text{projected}}, \mathbf{q}_{\text{actual}}, \mathbf{\dot{q}}_{\text{actual}}, \mathbf{a}_{t-1}]$$
  Reads live joint position ($\mathbf{q}$) and velocity ($\mathbf{\dot{q}}$) feedback directly from the Teensy via `/joint_states`.
* **Output:** Publishes target joint angles directly on `/joint_targets` (bypassing Teenys IK).

### 2.4 Micro-ROS Agent Node
* Establishes low-latency serial transport over USB (`/dev/ttyACM0`) running at $3\text{ Mbps}$.
* Maps Micro-ROS client topics running on Teensy directly into the Jetson's local ROS 2 computational graph.

---

## 3. ROS 2 Installation Guide on Ubuntu 24.04 (JetPack 7.2.1)

JetPack 7.2.1 is built on **Ubuntu 24.04 LTS (Noble Numbat)**. The native, officially supported ROS 2 distribution for Ubuntu 24.04 is **ROS 2 Jazzy Jalisco**.

### Step 1: System Locale & Repository Preparation

Open a terminal on your Jetson Orin Nano and ensure your system supports UTF-8:

```bash
sudo apt update && sudo apt install -y locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Ensure the Ubuntu Universe repository is enabled:

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
```

### Step 2: Add ROS 2 Apt Repository

Add the ROS 2 GPG key:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 Jazzy repository to your sources list:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Packages & Development Tools

Update package index and upgrade existing system packages:

```bash
sudo apt update && sudo apt upgrade -y
```

Install **ROS 2 Jazzy Desktop** (includes ROS, RViz2, and core libraries) along with build tools:

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep
```

### Step 4: Environment Configuration

Source the ROS 2 environment setup script automatically in every new terminal session:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify the installation distro:

```bash
printenv ROS_DISTRO
# Output should be: jazzy
```

---

## 4. Setting Up the Micro-ROS Agent & Workspace

The Micro-ROS agent is required on the Jetson to communicate with the Teensy 4.1 over USB.

### Step 1: Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

### Step 2: Create a ROS 2 Workspace
```bash
mkdir -p ~/quadruped_ws/src
cd ~/quadruped_ws/src
```

### Step 3: Clone & Build the Micro-ROS Agent
```bash
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

cd ~/quadruped_ws
colcon build --symlink-install
source install/setup.bash

# Create and build the Micro-ROS Agent workspace
ros2 run micro_ros_setup create_agent_workspace.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash
```

---

## 5. Running the Micro-ROS Agent Bridge

Once your Teensy is flashed and plugged into the Jetson's USB port:

1. Identify the USB device port (typically `/dev/ttyACM0`):
   ```bash
   ls -l /dev/ttyACM*
   ```

2. Set read/write permissions for the serial device:
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```

3. Launch the Micro-ROS agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 3000000
   ```

---

## 6. Jetson Performance Tuning

To ensure low latency and deterministic performance for high-rate RL inference and ROS 2 execution on the Jetson Orin Nano:

### Maximize Power Profile
```bash
# Enable maximum performance mode (15W / MaxN depending on Orin Nano variant)
sudo nvpmodel -m 0

# Lock CPU and GPU clocks to maximum frequency
sudo jetson_clocks
```

### Real-Time Thread Priorities
Grant the user permission to assign real-time thread scheduling (`FIFO`/`RR`) to critical ROS 2 nodes:

```bash
sudo usermod -aG realtime $USER
```
*(Add `@realtime - rtprio 99` to `/etc/security/limits.conf` if configured).*