#pragma once

#include "layer2/kinematics_config.hpp"
#include "quadruped_types.hpp"

namespace quadruped::layer2 {

class IKSolver {
public:
    IKSolver() = default;

    /**
     * @brief Solves analytical IK for a single 3-DOF leg.
     * @param target Foot position (x, y, z) relative to hip frame.
     * @param leg_idx Leg index (FL, FR, RL, RR) to apply side sign multipliers.
     * @param joint_angles Output array [q_hip, q_thigh, q_knee] in radians.
     * @return true if target was reachable without workspace saturation.
     */
    bool solveLegIK(const Vector3f& target, LegIndex leg_idx, std::array<float, 3>& joint_angles) const;

    /**
     * @brief Solves analytical IK for all 4 legs simultaneously.
     * @param foot_targets Input FootTargetsArray from Layer 3A.
     * @param joint_cmd Output JointCommand populated with target angles.
     * @return true if all 4 legs solved successfully.
     */
    bool solveBatchIK(const FootTargetsArray& foot_targets, JointCommand& joint_cmd) const;

private:
    LegDimensions config_;
};

} // namespace quadruped::layer2