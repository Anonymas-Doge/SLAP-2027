/**
 *  @file stance_trajectory.hpp
 *  @brief Uses leg progress and velocity to output
 *         the foot target for the leg in stance.
 */


#pragma once

#include "layer3_locomotion/quadruped_types.hpp"
#include "layer3_locomotion/gait/gait_params.hpp"


namespace quadruped::locomotion {

// Clamps progress values to [0, 1]
[[nodiscard]] FootTarget computeStanceTarget(const Twist2D& cmd,
                                             const GaitParams& params,
                                             LegIndex index,
                                             float progress);


} // namespace quadruped::locomotion