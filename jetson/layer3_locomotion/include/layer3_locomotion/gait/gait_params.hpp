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

    float period = 1.0f;
    float dutyFactor = 0.75f;

    std::array<float, NUM_LEGS> offsets{0.5f, 0.0f, 0.75f, 0.25f};
    std::array<Vector3f, NUM_LEGS> neutralPositions{Vector3f{0.0f, 0.0f, -0.20f},
                                                    Vector3f{0.0f, 0.0f, -0.20f},
                                                    Vector3f{0.0f, 0.0f, -0.20f},
                                                    Vector3f{0.0f, 0.0f, -0.20f}};
};


} // namespace quadruped::locomotion