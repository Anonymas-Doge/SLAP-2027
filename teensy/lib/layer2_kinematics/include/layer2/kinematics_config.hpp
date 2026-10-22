#pragma once

namespace quadruped::layer2 {

struct LegDimensions {
    float l1_hip_offset{0.080f};  // Meters (hip roll to hip pitch)
    float l2_thigh_length{0.200f}; // Meters (hip pitch to knee)
    float l3_calf_length{0.200f};  // Meters (knee to foot)
};

struct JointLimits {
    float q_min[3]{-0.785f, -1.570f, -2.618f}; // Radians [Hip, Thigh, Knee]
    float q_max[3]{ 0.785f,  3.141f, -0.523f}; // Radians
    float max_velocity[3]{12.0f, 12.0f, 12.0f}; // Rad/s
};

} // namespace quadruped::layer2