#include <algorithm>

#include "layer3_locomotion/gait/stance_trajectory.hpp"


namespace quadruped::locomotion {


static Vector3f calculatePosition(const Twist2D& cmd, const Vector3f& neutral_pos, LegIndex index,
                                  const float T_stance, const float s) {

    const float x_pos = neutral_pos.x + cmd.vx * T_stance * (0.5f - s);
    const float y_pos = neutral_pos.y + cmd.vy * T_stance * (0.5f - s);
    const float z_pos = neutral_pos.z;
    return Vector3f{x_pos, y_pos, z_pos};
}

FootTarget computeStanceTarget(const Twist2D& cmd, const GaitParams& params, LegIndex index, float progress) {

    const float s = std::clamp(progress, 0.0f, 1.0f);
    const float T_stance = params.period * params.dutyFactor;
    const Vector3f neutral_position = params.neutralPositions[static_cast<uint8_t>(index)];

    return FootTarget{calculatePosition(cmd, neutral_position, index, T_stance, s),
              Vector3f{-cmd.vx, -cmd.vy, 0.0f},
             true};
}


} // namespace quadruped::locomotion