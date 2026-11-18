#include <stdexcept>

#include "layer3_locomotion/gait/gait_params.hpp"

namespace quadruped::locomotion {


void validateGaitParams(const GaitParams& params) {

    if (params.period <= 0) { throw std::invalid_argument("Period must be positive"); }
    if (params.dutyFactor <= 0 || params.dutyFactor >= 1) {
        throw std::invalid_argument("Duty factor must be between 0 and 1");
    }

    for (size_t i = 0; i < NUM_LEGS; ++i) {
        if (params.offsets[i] < 0 || params.offsets[i] >= 1) {
            throw std::invalid_argument("Offset must be between 0 and 1");
        }
    }
}


} // namespace quadruped::locomotion