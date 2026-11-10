#include <cmath>
#include <stdexcept>

#include "layer3_locomotion/gait/phase_clock.hpp"


// Computes frac(x) = x - floor(x)
static float frac(const float phase) {
    return phase - std::floor(phase);
}


namespace quadruped::locomotion {


PhaseClock::PhaseClock(const GaitParams& params) : params{params} {
    validateParams();
}


void PhaseClock::validateParams() const{

    if (params.period <= 0) { throw std::invalid_argument("Period must be positive"); }
    if (params.dutyFactor <= 0 || params.dutyFactor >= 1) { throw std::invalid_argument("Duty factor must be between 0 and 1");}

    for (size_t i = 0; i < NUM_LEGS; ++i) {
        if (params.offsets[i] < 0 || params.offsets[i] >= 1) { throw std::invalid_argument("Offset must be between 0 and 1"); }
    }
}


void PhaseClock::advance(const float deltaT) {

    if (deltaT < 0) { throw std::invalid_argument("deltaT must not be negative"); }

    // Advance global phase clock
    // ø_global = frac(ø_global + ∆t/T)
    state.globalPhase = frac(state.globalPhase + (deltaT/params.period));


    // Per leg phase calculation and updates
    //  ø_leg = frac(ø_global + leg_offset)
    for (size_t i = 0; i < NUM_LEGS; ++i) {

        auto& [progress, inStance, liftOff, touchDown, legPhase] = outputs[i];

        // Update leg phase
        legPhase = frac(state.globalPhase + params.offsets[i]);

        // Update stance status
        inStance = legPhase < params.dutyFactor;

        // Update progress
        if (inStance) { progress = legPhase / params.dutyFactor; }
        else { progress = (legPhase - params.dutyFactor) / (1 - params.dutyFactor); }

        // Update liftOff and touchDown
        liftOff = state.wasInStance[i] && !inStance;
        touchDown = !state.wasInStance[i] && inStance;

        // Update wasInStance
        state.wasInStance[i] = inStance;
    }
}


LegPhaseState PhaseClock::getLegState(LegIndex index) const {
    // Indexed by LegIndex enum
    return outputs[static_cast<size_t>(index)];
}


void PhaseClock::reset() {
    // Reset all outputs to default
    state = PhaseState{};
    outputs.fill(LegPhaseState{});
}


} // namespace quadruped::locomotion