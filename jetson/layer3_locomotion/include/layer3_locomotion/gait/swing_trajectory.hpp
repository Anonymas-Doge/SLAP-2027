/**
 *  @file swing_trajectory.hpp
 *  @brief Calculates the foot target in swing using cubic
 *         Bézier curves
 */

#pragma once

#include "layer3_locomotion/quadruped_types.hpp"
#include "layer3_locomotion/gait/gait_params.hpp"

namespace quadruped::locomotion {


[[nodiscard]] FootTarget computeSwingTarget(const GaitParams& params,
                                            const Vector3f& p0, // p0 is starting foot positon from gait_generator
                                            const Vector3f& p3, // p3 is ending foot position from foot_placement
                                            float progress);


} // namespace quadruped::locomotion
