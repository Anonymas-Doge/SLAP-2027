#pragma once

#include "layer3_locomotion/quadruped_types.hpp"

namespace quadruped::locomotion {


[[nodiscard]] Twist2D computeLegVelocity(const Twist2D& cmd,
                                         const Vector3f& hip_position,
                                         const Vector3f& neutral_position);


} // namespace quadruped::locomotion