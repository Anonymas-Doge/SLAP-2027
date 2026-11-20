#include <algorithm>

#include "layer3_locomotion/gait/stance_trajectory.hpp"
#include "layer3_locomotion/gait/leg_velocity.hpp"


namespace quadruped::locomotion {


// Fetches and calculates positions for given leg
static Vector3f calculatePosition(const Twist2D& v_leg, const Vector3f& neutral_pos,
                                  const float T_stance, const float s) {

    const float x_pos = neutral_pos.x + v_leg.vx * T_stance * (0.5f - s);
    const float y_pos = neutral_pos.y + v_leg.vy * T_stance * (0.5f - s);
    const float z_pos = neutral_pos.z;
    return Vector3f{x_pos, y_pos, z_pos};
}


FootTarget computeStanceTarget(const Twist2D& cmd, const GaitParams& params, LegIndex index, const float progress) {

    const float s = std::clamp(progress, 0.0f, 1.0f); // Clamps to handle float values like 1.000000001
    const float T_stance = params.period * params.dutyFactor;

    const Vector3f& neutral_position{params.neutralPositions[static_cast<size_t>(index)]};
    const Vector3f& hip_position{params.hipPositions[static_cast<size_t>(index)]};
    const Twist2D v_leg{computeLegVelocity(cmd, hip_position, neutral_position)};

    return FootTarget{calculatePosition(v_leg, neutral_position, T_stance, s),
              Vector3f{-v_leg.vx, -v_leg.vy, 0.0f},
             true};
}


} // namespace quadruped::locomotion