#include "layer1/teensy_hardware.hpp"

#include <Arduino.h>
#include <SPI.h>
#include <MoteusAcan2517fd.h>
#include <cmath>

#include "layer1/layer1_config.hpp"

namespace quadruped::layer1 {
namespace {
ACAN2517FD can(config::MCP2518FD_CS_PIN, SPI, config::MCP2518FD_INT_PIN);
Moteus::Options optionsFor(uint8_t id) {
    Moteus::Options options;
    options.id = static_cast<int8_t>(id);
    options.disable_brs = false;
    options.min_rcv_wait_us = 1000;
    options.query_format.torque = Moteus::kFloat;
    return options;
}
Moteus m0(can, optionsFor(config::MOTEUS_CAN_IDS[0]));
Moteus m1(can, optionsFor(config::MOTEUS_CAN_IDS[1]));
Moteus m2(can, optionsFor(config::MOTEUS_CAN_IDS[2]));
Moteus m3(can, optionsFor(config::MOTEUS_CAN_IDS[3]));
Moteus m4(can, optionsFor(config::MOTEUS_CAN_IDS[4]));
Moteus m5(can, optionsFor(config::MOTEUS_CAN_IDS[5]));
Moteus m6(can, optionsFor(config::MOTEUS_CAN_IDS[6]));
Moteus m7(can, optionsFor(config::MOTEUS_CAN_IDS[7]));
Moteus m8(can, optionsFor(config::MOTEUS_CAN_IDS[8]));
Moteus m9(can, optionsFor(config::MOTEUS_CAN_IDS[9]));
Moteus m10(can, optionsFor(config::MOTEUS_CAN_IDS[10]));
Moteus m11(can, optionsFor(config::MOTEUS_CAN_IDS[11]));
Moteus* const motors[NUM_JOINTS] = {&m0,&m1,&m2,&m3,&m4,&m5,&m6,&m7,&m8,&m9,&m10,&m11};
int motorIndex(uint8_t id) {
    for (uint8_t i=0; i<NUM_JOINTS; ++i) if (config::MOTEUS_CAN_IDS[i] == id) return i;
    return -1;
}
bool sendFrame(const Moteus::CanFdFrame& frame) {
    CANFDMessage message;
    message.id = frame.arbitration_id;
    message.ext = true;
    message.len = frame.size;
    memcpy(message.data, frame.data, frame.size);
    const bool brs = frame.brs != mm::CanFdFrame::kForceOff;
    message.type = brs ? CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH :
                         CANFDMessage::CANFD_NO_BIT_RATE_SWITCH;
    mm::PadCan(&message);
    can.poll();
    return can.tryToSend(message);
}
}

bool TeensyCanInterface::begin() {
    SPI.begin();
    const DataBitRateFactor data_rate = static_cast<DataBitRateFactor>(config::CAN_DATA_RATE_FACTOR);
    ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_20MHz,
                                static_cast<uint32_t>(config::CAN_NOMINAL_BITRATE), data_rate);
    settings.mArbitrationSJW = 2;
    return can.begin(settings, [] { can.isr(); }) == 0;
}

bool TeensyCanInterface::sendPosition(uint8_t id, float position, float velocity, float torque) {
    const int index = motorIndex(id);
    if (index < 0) return false;
    Moteus::PositionMode::Command command;
    command.position = position;
    command.velocity = velocity;
    command.feedforward_torque = torque;
    return sendFrame(motors[index]->MakePosition(command));
}

bool TeensyCanInterface::pollStatus(ControllerStatus& status) {
    can.poll();
    for (uint8_t attempt = 0; attempt < 16; ++attempt) {
        CANFDMessage message;
        if (!can.receive(message)) return false;
        if (!message.ext) continue;
        const uint8_t can_id = static_cast<uint8_t>((message.id >> 8) & 0x7f);
        if (motorIndex(can_id) < 0 || (message.id & 0x7f) != 0 || (message.id >> 16) != 0) continue;
        const auto value = Moteus::Query::Parse(message.data, message.len);
        status.can_id = can_id;
        status.received_at_ms = millis();
        status.position = static_cast<float>(value.position);
        status.velocity = static_cast<float>(value.velocity);
        status.effort = static_cast<float>(value.torque);
        status.position_valid = std::isfinite(value.position);
        status.velocity_valid = std::isfinite(value.velocity);
        status.effort_valid = std::isfinite(value.torque);
        status.controller_fault = value.fault != 0;
        return true;
    }
    return false;
}

bool TeensyCanInterface::requestSafeState(uint8_t id, config::SafeAction action, float hold) {
    const int index = motorIndex(id);
    if (index < 0) return false;
    switch (action) {
    case config::SafeAction::STOP: return sendFrame(motors[index]->MakeStop());
    case config::SafeAction::BRAKE: return sendFrame(motors[index]->MakeBrake());
    case config::SafeAction::HOLD_POSITION: {
        Moteus::PositionMode::Command command;
        command.position = hold;
        command.velocity = 0.0;
        return sendFrame(motors[index]->MakePosition(command));
    }
    }
    return false;
}

Bno085SpiInterface::Bno085SpiInterface()
    : sensor_(config::BNO085_RESET_PIN == 255 ? -1 : config::BNO085_RESET_PIN) {}

bool Bno085SpiInterface::begin() {
    ready_ = sensor_.begin_SPI(config::BNO085_CS_PIN, config::BNO085_INT_PIN);
    reportsNeedReenable_ = ready_;
    return ready_ && enableReports();
}

bool Bno085SpiInterface::enableReports() {
    const bool ok = sensor_.enableReport(SH2_ROTATION_VECTOR, config::IMU_REPORT_PERIOD_US) &&
                    sensor_.enableReport(SH2_GYROSCOPE_CALIBRATED, config::IMU_REPORT_PERIOD_US) &&
                    sensor_.enableReport(SH2_LINEAR_ACCELERATION, config::IMU_REPORT_PERIOD_US);
    reportsNeedReenable_ = !ok;
    return ok;
}

bool Bno085SpiInterface::poll(ImuReport& report) {
    if (!ready_) return false;
    if (sensor_.wasReset()) reportsNeedReenable_ = true;
    if (reportsNeedReenable_ && !enableReports()) return false;
    sh2_SensorValue_t value;
    if (!sensor_.getSensorEvent(&value)) return false;
    report.received_at_ms = millis();
    switch (value.sensorId) {
    case SH2_ROTATION_VECTOR:
        report.data.orientation.w = value.un.rotationVector.real;
        report.data.orientation.x = value.un.rotationVector.i;
        report.data.orientation.y = value.un.rotationVector.j;
        report.data.orientation.z = value.un.rotationVector.k;
        report.orientation_valid = true;
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        report.data.angular_velocity = {value.un.gyroscope.x, value.un.gyroscope.y, value.un.gyroscope.z};
        report.angular_velocity_valid = true;
        break;
    case SH2_LINEAR_ACCELERATION:
        report.data.linear_acceleration = {value.un.linearAcceleration.x,
                                           value.un.linearAcceleration.y,
                                           value.un.linearAcceleration.z};
        report.linear_acceleration_valid = true;
        break;
    default: return false;
    }
    return true;
}
} // namespace quadruped::layer1
