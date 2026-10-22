#pragma once

#include <string>
#include <memory>
#include "quadruped_fsm/quadruped_types.hpp"

namespace quadruped::fsm {

class Layer4Node; // Forward declaration

class FSMState {
public:
    explicit FSMState(RobotState state_enum, std::string state_name)
        : state_enum_(state_enum), state_name_(std::move(state_name)) {}
    
    virtual ~FSMState() = default;

    virtual void onEnter(Layer4Node& node) = 0;
    virtual void update(Layer4Node& node, float dt) = 0;
    virtual void onExit(Layer4Node& node) = 0;

    [[nodiscard]] RobotState getStateEnum() const { return state_enum_; }
    [[nodiscard]] const std::string& getStateName() const { return state_name_; }

protected:
    RobotState state_enum_;
    std::string state_name_;
};

} // namespace quadruped::fsm