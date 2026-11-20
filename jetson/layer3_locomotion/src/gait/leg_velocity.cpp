#include "layer3_locomotion/gait/leg_velocity.hpp"

namespace quadruped::locomotion {


Twist2D computeLegVelocity(const Twist2D& cmd, const Vector3f& hip_position, const Vector3f& neutral_position) {

    const float r_x = hip_position.x + neutral_position.x;
    const float r_y = hip_position.y + neutral_position.y;

    return {cmd.vx - cmd.omega_z * r_y,
                   cmd.vy + cmd.omega_z * r_x,
                       cmd.omega_z};
}


} // namespace quadruped::locomotion
