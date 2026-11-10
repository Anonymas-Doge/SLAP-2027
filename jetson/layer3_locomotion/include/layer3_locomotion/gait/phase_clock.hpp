/**
 *  @file phase_clock.hpp
 *  @brief Controls the phase clock for the gait generation
 *         across the quadruped. liftOff and touchDown are
 *         only true on the tick where the switch
 *         happens.
 */

#ifndef BUILD_PHASE_CLOCK_HPP
#define BUILD_PHASE_CLOCK_HPP

#include <array>

#include "layer3_locomotion/gait/gait_params.hpp"
#include "layer3_locomotion/quadruped_types.hpp"

namespace quadruped::locomotion {


struct LegPhaseState {
    float progress = 0;
    bool inStance = true;
    bool liftOff = false;
    bool touchDown = false;
    float legPhase = 0;
};


struct PhaseState {
    float globalPhase = 0;
    std::array<bool, NUM_LEGS> wasInStance{true, true, true, true};
};


class PhaseClock {

public:

    explicit PhaseClock(const GaitParams& params);

    void advance(float deltaT);
    [[nodiscard]] LegPhaseState getLegState(LegIndex legIndex) const;

    void reset();

private:
    PhaseState state;
    GaitParams params;
    std::array<LegPhaseState, NUM_LEGS> outputs;

    void validateParams() const;
};

} // namespace quadruped::locomotion

#endif //BUILD_PHASE_CLOCK_HPP