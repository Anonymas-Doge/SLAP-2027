#pragma once

#include <cstdint>

#include "quadruped_types.hpp"

namespace quadruped::layer1::config {

// Hardware placeholders: verify every pin, bitrate, ID and mechanical limit
// against the assembled robot before enabling motor output.
constexpr uint32_t CAN_NOMINAL_BITRATE = 1000000;
constexpr uint8_t CAN_DATA_RATE_FACTOR = 4; // PLACEHOLDER: confirm bus supports 4 Mbit/s.
constexpr uint8_t MCP2518FD_CS_PIN = 10;
constexpr uint8_t MCP2518FD_INT_PIN = 9;
constexpr uint8_t BNO085_CS_PIN = 7;
constexpr uint8_t BNO085_INT_PIN = 6;
constexpr uint8_t BNO085_RESET_PIN = 255; // 255 means no reset line
constexpr uint8_t ESTOP_PIN = 5;
constexpr bool ESTOP_ACTIVE_LOW = true;
constexpr uint8_t DRIVER_ENABLE_PIN = 255; // optional hardware cutoff; 255 = absent
constexpr bool DRIVER_ENABLE_ACTIVE = true;

constexpr uint32_t CONTROL_PERIOD_US = 5000;
constexpr uint32_t COMMAND_TIMEOUT_MS = 250;
constexpr uint32_t STATUS_TIMEOUT_MS = 100;
constexpr uint32_t TRACKING_ERROR_TIME_MS = 50;
constexpr float TRACKING_ERROR_THRESHOLD_RAD = 0.35f;
constexpr float MAX_TARGET_VELOCITY_RAD_S = 12.0f;
constexpr uint16_t IMU_REPORT_PERIOD_US = 10000;
constexpr char JOINT_STATE_FRAME_ID[] = "base_link";
constexpr char IMU_FRAME_ID[] = "imu_link";

// Example IDs only. Configure these to match the physical moteus network.
constexpr uint8_t MOTEUS_CAN_IDS[NUM_JOINTS] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

// Seeded from Layer 2's generic per-leg limits; confirm per-joint offsets,
// directions and physical travel before use.
constexpr float JOINT_POSITION_MIN_RAD[NUM_JOINTS] = {
    -0.785f, -1.570f, -2.618f, -0.785f, -1.570f, -2.618f,
    -0.785f, -1.570f, -2.618f, -0.785f, -1.570f, -2.618f};
constexpr float JOINT_POSITION_MAX_RAD[NUM_JOINTS] = {
     0.785f,  3.141f, -0.523f,  0.785f,  3.141f, -0.523f,
     0.785f,  3.141f, -0.523f,  0.785f,  3.141f, -0.523f};

enum class SafeAction : uint8_t { STOP, BRAKE, HOLD_POSITION };
constexpr SafeAction SAFE_ACTION = SafeAction::STOP; // Confirm for robot mechanics.

} // namespace quadruped::layer1::config
