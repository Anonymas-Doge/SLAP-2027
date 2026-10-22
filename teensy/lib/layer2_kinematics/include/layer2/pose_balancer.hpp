#pragma once

#include "quadruped_types.hpp"

namespace quadruped::layer2 {

struct BalanceGains {
    float kp_roll{0.5f};
    float kd_roll{0.05f};
    float kp_pitch{0.5f};
    float kd_pitch{0.05f};
};

class PoseBalancer {
public:
    PoseBalancer() = default;

    void setGains(const BalanceGains& gains) { gains_ = gains; }

    /**
     * @brief Adjusts foot targets using body stance pose and IMU feedback.
     * @param raw_targets Input Cartesian foot positions.
     * @param pose_cmd Target body stance orientation (roll, pitch, yaw).
     * @param imu Filtered IMU readings.
     * @param adjusted_targets Output foot positions with pose/balance offsets applied.
     */
    void applyPoseAndBalance(const FootTargetsArray& raw_targets,
                             const EulerAngles& pose_cmd,
                             const ImuData& imu,
                             FootTargetsArray& adjusted_targets) const;

private:
    BalanceGains gains_;
};

} // namespace quadruped::layer2