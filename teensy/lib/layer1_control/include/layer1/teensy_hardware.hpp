#pragma once

#include <Adafruit_BNO08x.h>

#include "layer1/hardware_interfaces.hpp"

namespace quadruped::layer1 {

class TeensyCanInterface final : public CanInterface {
public:
    bool begin() override;
    bool sendPosition(uint8_t can_id, float position_rad, float velocity_rad_s,
                      float feedforward_torque) override;
    bool pollStatus(ControllerStatus& status) override;
    bool requestSafeState(uint8_t can_id, config::SafeAction action,
                          float hold_position_rad) override;
};

class Bno085SpiInterface final : public ImuInterface {
public:
    Bno085SpiInterface();
    bool begin() override;
    bool poll(ImuReport& report) override;
private:
    Adafruit_BNO08x sensor_;
    bool ready_{false};
    bool reportsNeedReenable_{false};
    bool enableReports();
};

} // namespace quadruped::layer1
