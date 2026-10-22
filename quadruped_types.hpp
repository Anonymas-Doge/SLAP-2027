/**
 * @file quadruped_types.hpp
 * @brief Common data structures, enumerations, and type definitions shared
 *        across software layers (Layers 1-4) and between Jetson Orin Nano
 *        and Teensy 4.1 platforms.
 * 
 * Designed to be lightweight, fixed-size (zero dynamic allocation), POD-compatible,
 * and portable between desktop/embedded C++ compilers (C++17).
 */

#pragma once

#include <cstdint>
#include <array>

namespace quadruped {

// ============================================================================
// CONSTANTS & DIMENSIONS
// ============================================================================

constexpr uint8_t NUM_LEGS = 4;
constexpr uint8_t JOINTS_PER_LEG = 3;
constexpr uint8_t NUM_JOINTS = NUM_LEGS * JOINTS_PER_LEG; // 12 joints total

// RL Observation dimensions (matching Isaac Lab policy setup)
constexpr size_t RL_OBSERVATION_DIM = 45; // 3 cmd + 3 gyro + 3 proj_g + 12 q + 12 qd + 12 prev_a

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @brief High-level Finite State Machine (FSM) states.
 */
enum class RobotState : uint8_t {
    OFF = 0,        ///< Unpowered / passive damping state
    INIT,           ///< Hardware self-check & encoder zeroing
    SIT,            ///< Crouched resting posture on ground
    STAND,          ///< Active upright standing with roll/pitch orientation control
    LOCOMOTION,     ///< Active walking/trotting state driven by velocity commands
    // RECOVERY,       ///< Fall detection and self-righting sequence  /*** not priority ***/
    ESTOP = 6       ///< Emergency hardware/software cutoff
};

/**
 * @brief Locomotion control pathways.
 */
enum class LocomotionMode : uint8_t {
    ALGORITHMIC = 0, ///< Jetson Layer 3A Gait Generator + Teensy Layer 2 IK
    NEURAL_RL = 1    ///< Jetson Layer 3B Isaac Lab RL Policy (Direct Joint Targets)
};

/**
 * @brief Supported algorithmic gaits.
 */
enum class GaitType : uint8_t {
    WALK = 0, ///< 3-leg stance crawl gait
    TROT      ///< Alternating diagonal pairs
};

/**
 * @brief Leg indexing convention.
 */
enum class LegIndex : uint8_t {
    FL = 0,  ///< Front Left
    FR = 1, ///< Front Right
    RL = 2,   ///< Rear Left
    RR = 3   ///< Rear Right
};

/**
 * @brief Joint indexing per leg.
 */
enum class JointIndex : uint8_t {
    HAA = 0, ///< Hip abduction/adduction
    HFE = 1, ///< Hip flexion/extension
    KFE = 2  ///< Knee flexion/extension
};

// ============================================================================
// MATH & KINEMATIC TYPES
// ============================================================================

/**
 * @brief 3D Cartesian vector float type.
 */
struct Vector3f {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

/**
 * @brief Unit quaternion for 3D body orientation representation.
 */
struct Quaternionf {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

/**
 * @brief Euler orientation angles in radians.
 */
struct EulerAngles {
    float roll{0.0f};  ///< Rotation around X axis (rad)
    float pitch{0.0f}; ///< Rotation around Y axis (rad)
    float yaw{0.0f};   ///< Rotation around Z axis (rad)
};

/**
 * @brief Target 2D body velocity vector.
 */
struct Twist2D {
    float vx{0.0f};      ///< Forward velocity (m/s)
    float vy{0.0f};      ///< Lateral velocity (m/s)
    float omega_z{0.0f}; ///< Yaw turning rate (rad/s)
};

// ============================================================================
// LAYER 4: HIGH-LEVEL COMMAND DATA
// ============================================================================

/**
 * @brief Commands passed from Layer 4 FSM to lower locomotion layers.
 */
struct HighLevelCommand {
    RobotState requested_state{RobotState::OFF};
    LocomotionMode requested_mode{LocomotionMode::ALGORITHMIC};
    Twist2D velocity_cmd;    ///< Target 2D velocity for LOCOMOTION state
    EulerAngles pose_cmd;    ///< Target body orientation for STAND state
    GaitType active_gait{GaitType::TROT};
    uint32_t timestamp_ms{0};
};

// ============================================================================
// LAYER 3A: ALGORITHMIC FOOT TARGETS
// ============================================================================

/**
 * @brief Foot position target for a single leg relative to hip frame.
 */
struct FootTarget {
    Vector3f position;   ///< Cartesian target (x, y, z) in meters relative to hip
    Vector3f velocity;   ///< Feedforward linear foot velocity (m/s)
    bool is_stance{true};///< true = stance phase (ground contact), false = swing phase
};

/**
 * @brief Container for all 4 foot targets produced by Layer 3A Gait Generator.
 */
struct FootTargetsArray {
    std::array<FootTarget, NUM_LEGS> feet;
    uint32_t timestamp_ms{0};
};

// ============================================================================
// LAYER 1 & 2 / 3B: JOINT COMMANDS & FEEDBACK
// ============================================================================

/**
 * @brief Target commands for all 12 motor joints.
 */
struct JointCommand {
    std::array<float, NUM_JOINTS> position_target{0.0f};      ///< q_target (rad)
    std::array<float, NUM_JOINTS> velocity_target{0.0f};      ///< qd_target (rad/s)
    std::array<float, NUM_JOINTS> kp{0.0f};                   ///< Proportional gain
    std::array<float, NUM_JOINTS> kd{0.0f};                   ///< Derivative gain
    std::array<float, NUM_JOINTS> ki{0.0f};                   ///< Integral gain
    // std::array<float, NUM_JOINTS> torque_feedforward{0.0f};   ///< Feedforward torque (N*m)
    /*** research PID vs PD+feedforward (feedforward may not be feasible) ***/
    uint32_t timestamp_ms{0};
};

/**
 * @brief Feedback measurement from motor encoders and current sensors.
 */
struct JointState {
    std::array<float, NUM_JOINTS> position{0.0f};     ///< Actual joint position q (rad)
    std::array<float, NUM_JOINTS> velocity{0.0f};     ///< Actual joint velocity qd (rad/s)
    std::array<float, NUM_JOINTS> effort{0.0f};       ///< Measured torque/current (N*m)
    std::array<bool, NUM_JOINTS> encoder_valid{false};///< Health check flag per joint /*** prevents runaway motors ***/
    uint32_t timestamp_ms{0};
};

// ============================================================================
// SENSOR & TELEMETRY DATA
// ============================================================================

/**
 * @brief Processed IMU sensor telemetry.
 */
struct ImuData {
    Quaternionf orientation;     ///< Filtered orientation quaternion
    EulerAngles euler;           ///< Roll, pitch, yaw (rad)
    Vector3f angular_velocity;   ///< Gyroscope values (rad/s)
    Vector3f linear_acceleration;///< Accelerometer values (m/s^2)
    uint32_t timestamp_ms{0};
};

// ============================================================================
// HARDWARE DIAGNOSTICS & STATUS
// ============================================================================

/**
 * @brief System hardware safety and monitoring flags.
 */
struct SystemDiagnostics {
    float battery_voltage{0.0f};            ///< Battery voltage (V)
    float battery_current{0.0f};            ///< Main power current draw (A)
    float board_temperature{0.0f};          ///< Teensy/Driver temp (°C)
    std::array<float, NUM_JOINTS> motor_temperatures{0.0f}; ///< Motor coil temp (°C)
    
    uint32_t error_flags{0};                ///< Bitfield for system fault codes
    bool estop_asserted{false};             ///< Physical or software E-stop status
    bool jetson_heartbeat_ok{true};         ///< True if serial connection is active
};

/**
 * @brief Overall telemetry packet emitted by Teensy.
 */
struct RobotTelemetry {
    RobotState current_state{RobotState::OFF};
    LocomotionMode current_mode{LocomotionMode::ALGORITHMIC};
    JointState joint_state;
    ImuData imu_data;
    SystemDiagnostics diagnostics;
    uint32_t timestamp_ms{0};
};

} // namespace quadruped
