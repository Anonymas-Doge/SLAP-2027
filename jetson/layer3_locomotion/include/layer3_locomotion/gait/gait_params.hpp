/**
 *  @file gait_params.hpp
 *  @brief Simple struct to hold and pass gait
 *         parameters to different submodules.
 */


#pragma once

#include <array>

#include "layer3_locomotion/quadruped_types.hpp"


namespace quadruped::locomotion {


struct GaitParams {

    float period = 1.0f; // T
    float dutyFactor = 0.75f; // β
    float stepHeight = 0.05f; // Tunable clearance h

    std::array<float, NUM_LEGS> offsets{0.5f, 0.0f, 0.75f, 0.25f};
    std::array<Vector3f, NUM_LEGS> neutralPositions{Vector3f{0.0f, 0.0f, -0.20f},
                                                    Vector3f{0.0f, 0.0f, -0.20f},
                                                    Vector3f{0.0f, 0.0f, -0.20f},
                                                    Vector3f{0.0f, 0.0f, -0.20f}};
};


void validateGaitParams(const GaitParams& params);


} // namespace quadruped::locomotion