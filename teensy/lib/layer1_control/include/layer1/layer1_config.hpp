#pragma once

#include <cstdint>

#include "quadruped_types.hpp"

namespace quadruped::layer1::config {

// PLACEHOLDERS: replace these values with the wired hardware configuration
// and validate them against the physical robot before enabling motor output.
constexpr uint32_t CAN_NOMINAL_BITRATE = 1000000;
constexpr uint32_t CAN_DATA_BITRATE = 5000000;
constexpr uint8_t MCP2518FD_CS_PIN = 10;
constexpr uint8_t MCP2518FD_INT_PIN = 9;
constexpr uint8_t BNO085_CS_PIN = 7;
constexpr uint8_t BNO085_INT_PIN = 6;
constexpr uint8_t ESTOP_PIN = 5;
constexpr uint32_t CONTROL_PERIOD_US = 5000;       // 200 Hz; confirm bus capacity.
constexpr uint32_t COMMAND_TIMEOUT_MS = 250;       // PLACEHOLDER.
constexpr uint32_t STATUS_TIMEOUT_MS = 100;        // PLACEHOLDER.
constexpr uint32_t TRACKING_ERROR_TIME_MS = 50;
constexpr float TRACKING_ERROR_THRESHOLD_RAD = 0.35f; // PLACEHOLDER.
constexpr float MAX_TARGET_VELOCITY_RAD_S = 12.0f; // PLACEHOLDER.
constexpr char IMU_FRAME_ID[] = "imu_link";        // PLACEHOLDER.
constexpr uint16_t IMU_REPORT_PERIOD_MS = 10;      // 100 Hz; PLACEHOLDER.

// Example IDs only. Replace with the configured IDs of the 12 moteus devices.
constexpr uint8_t MOTEUS_CAN_IDS[NUM_JOINTS] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

// Example physical software bounds copied from the current Layer 2 leg limits.
// Confirm per-joint directions, offsets, and mechanics before use.
constexpr float JOINT_POSITION_MIN_RAD[NUM_JOINTS] = {
    -0.785f, -1.570f, -2.618f, -0.785f, -1.570f, -2.618f,
    -0.785f, -1.570f, -2.618f, -0.785f, -1.570f, -2.618f};
constexpr float JOINT_POSITION_MAX_RAD[NUM_JOINTS] = {
     0.785f,  3.141f, -0.523f,  0.785f,  3.141f, -0.523f,
     0.785f,  3.141f, -0.523f,  0.785f,  3.141f, -0.523f};

// This enum is an adapter-level request, not a claim about the resulting
// mechanical behavior. Validate the mapping for the installed moteus setup.
enum class SafeAction : uint8_t {
    DISABLE_OUTPUTS = 0,
    HOLD_POSITION = 1,
};
constexpr SafeAction SAFE_ACTION = SafeAction::DISABLE_OUTPUTS; // PLACEHOLDER.

} // namespace quadruped::layer1::config
