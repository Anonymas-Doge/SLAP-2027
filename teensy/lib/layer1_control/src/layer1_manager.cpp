#include "layer1/layer1_manager.hpp"

#include <cmath>

#include "layer1/layer1_config.hpp"

namespace quadruped::layer1 {
namespace {
bool elapsed(uint32_t now, uint32_t then, uint32_t duration) {
    return static_cast<uint32_t>(now - then) >= duration;
}
int jointIndexForCanId(uint8_t can_id) {
    for (uint8_t i = 0; i < NUM_JOINTS; ++i)
        if (config::MOTEUS_CAN_IDS[i] == can_id) return i;
    return -1;
}
}

Layer1Manager::Layer1Manager(CanInterface& can, ImuInterface& imu) : can_(can), imu_(imu) {}

bool Layer1Manager::init(uint32_t now_ms) {
    can_ready_ = can_.begin();
    imu_ready_ = imu_.begin();
    if (!can_ready_) faults_ |= FAULT_CAN_INIT;
    if (!imu_ready_) faults_ |= FAULT_IMU_INIT;
    initialized_at_ms_ = now_ms;
    command_received_at_ms_ = now_ms;
    return can_ready_ && imu_ready_;
}

void Layer1Manager::submitCommand(const JointCommand& command, uint32_t received_at_ms) {
    command_ = command;
    command_received_at_ms_ = received_at_ms;
    command_received_ = command_seen_once_ = true;
}

void Layer1Manager::onEstopInterrupt() { estop_interrupt_pending_ = true; }

void Layer1Manager::clearEstopLatch() {
    estop_latched_ = false;
    faults_ &= ~static_cast<uint32_t>(FAULT_ESTOP);
    if (faults_ == FAULT_NONE) { safe_stopped_ = false; safe_state_sent_ = false; }
}

void Layer1Manager::update(uint32_t now_ms) {
    if (can_ready_) {
        // Drain a bounded number of replies each cycle; each result carries a moteus ID.
        for (uint8_t n = 0; n < NUM_JOINTS; ++n) {
            ControllerStatus status{};
            if (!can_.pollStatus(status)) break;
            consumeStatus(status, now_ms);
        }
    }
    if (imu_ready_) {
        ImuReport report{};
        if (imu_.poll(report)) {
            if (report.orientation_valid) imu_data_.orientation = report.data.orientation;
            if (report.angular_velocity_valid) imu_data_.angular_velocity = report.data.angular_velocity;
            if (report.linear_acceleration_valid) imu_data_.linear_acceleration = report.data.linear_acceleration;
            if (report.orientation_valid || report.angular_velocity_valid || report.linear_acceleration_valid)
                imu_data_.timestamp_ms = report.received_at_ms;
        }
    }
    checkSafety(now_ms);
    if (!command_received_ || !can_ready_) return;
    if (faults_ != FAULT_NONE || safe_stopped_) { requestSafeState(); return; }

    JointCommand bounded = command_;
    if (!validateAndBoundCommand(bounded)) {
        faults_ |= FAULT_INVALID_COMMAND;
        checkSafety(now_ms);
        requestSafeState();
        return;
    }
    for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
        if (!can_.sendPosition(config::MOTEUS_CAN_IDS[i], bounded.position_target[i],
                               bounded.velocity_target[i], 0.0f)) {
            faults_ |= FAULT_CAN_TX;
            safe_stopped_ = true;
            requestSafeState();
            return;
        }
    }
}

void Layer1Manager::consumeStatus(const ControllerStatus& status, uint32_t now_ms) {
    const int index = jointIndexForCanId(status.can_id);
    if (index < 0 || static_cast<uint32_t>(now_ms - status.received_at_ms) > config::STATUS_TIMEOUT_MS) return;
    const auto i = static_cast<uint8_t>(index);
    if (status.controller_fault) faults_ |= FAULT_CONTROLLER;
    if (status.position_valid && std::isfinite(status.position)) {
        joint_state_.position[i] = status.position;
        joint_state_.encoder_valid[i] = true;
        joint_state_.timestamp_ms = status.received_at_ms;
    }
    velocity_valid_[i] = status.velocity_valid && std::isfinite(status.velocity);
    effort_valid_[i] = status.effort_valid && std::isfinite(status.effort);
    if (velocity_valid_[i]) joint_state_.velocity[i] = status.velocity;
    if (effort_valid_[i]) joint_state_.effort[i] = status.effort;
    joint_last_status_ms_[i] = status.received_at_ms;
}

void Layer1Manager::checkSafety(uint32_t now_ms) {
    if (estop_interrupt_pending_) { estop_interrupt_pending_ = false; estop_latched_ = true; }
    if (estop_latched_) faults_ |= FAULT_ESTOP;
    if (elapsed(now_ms, command_received_at_ms_, config::COMMAND_TIMEOUT_MS))
        faults_ |= FAULT_COMMAND_STALE;
    if (elapsed(now_ms, initialized_at_ms_, config::STATUS_TIMEOUT_MS)) {
        for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
            if (!joint_state_.encoder_valid[i] || elapsed(now_ms, joint_last_status_ms_[i], config::STATUS_TIMEOUT_MS)) {
                faults_ |= FAULT_STATUS_STALE; break;
            }
        }
    }
    if (command_received_) for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
        if (!joint_state_.encoder_valid[i] || !std::isfinite(command_.position_target[i])) continue;
        float target = fminf(fmaxf(command_.position_target[i], config::JOINT_POSITION_MIN_RAD[i]),
                             config::JOINT_POSITION_MAX_RAD[i]);
        if (fabsf(target - joint_state_.position[i]) > config::TRACKING_ERROR_THRESHOLD_RAD) {
            if (!tracking_error_active_[i]) { tracking_error_active_[i] = true; tracking_error_since_ms_[i] = now_ms; }
            else if (elapsed(now_ms, tracking_error_since_ms_[i], config::TRACKING_ERROR_TIME_MS)) faults_ |= FAULT_TRACKING_ERROR;
        } else tracking_error_active_[i] = false;
    }
    if (faults_ != FAULT_NONE) safe_stopped_ = true;
}

void Layer1Manager::requestSafeState() {
    if (!can_ready_ || safe_state_sent_) return;
    bool sent = true;
    for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
        float hold = joint_state_.encoder_valid[i] ? joint_state_.position[i] : 0.0f;
        sent = can_.requestSafeState(config::MOTEUS_CAN_IDS[i], config::SAFE_ACTION, hold) && sent;
    }
    safe_state_sent_ = sent;
}

bool Layer1Manager::validateAndBoundCommand(JointCommand& command) {
    for (uint8_t i = 0; i < NUM_JOINTS; ++i) {
        if (!std::isfinite(command.position_target[i]) || !std::isfinite(command.velocity_target[i]) ||
            config::JOINT_POSITION_MIN_RAD[i] > config::JOINT_POSITION_MAX_RAD[i]) return false;
        command.position_target[i] = fminf(fmaxf(command.position_target[i], config::JOINT_POSITION_MIN_RAD[i]),
                                           config::JOINT_POSITION_MAX_RAD[i]);
        command.velocity_target[i] = fminf(fmaxf(command.velocity_target[i], -config::MAX_TARGET_VELOCITY_RAD_S),
                                           config::MAX_TARGET_VELOCITY_RAD_S);
    }
    return true;
}
} // namespace quadruped::layer1
