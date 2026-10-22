#pragma once

#include "layer2/ik_solver.hpp"
#include "layer2/pose_balancer.hpp"
#include "quadruped_types.hpp"

namespace quadruped::layer2 {

class Layer2Manager {
public:
    Layer2Manager() = default;

    void init();

    /**
     * @brief Main processing step executed at 100-500 Hz.
     * @param mode Active locomotion mode (ALGORITHMIC vs NEURAL_RL).
     * @param foot_targets Incoming foot target buffer from Layer 3A.
     * @param rl_joint_cmd Incoming joint target buffer from Layer 3B.
     * @param pose_cmd Incoming stance orientation command.
     * @param imu Telemetry from Layer 1 IMU loop.
     * @param current_joint_state Actual motor encoder states from Layer 1.
     * @param out_cmd Processed safety-checked joint commands sent to Layer 1 PID.
     */
    void update(LocomotionMode mode,
                const FootTargetsArray& foot_targets,
                const JointCommand& rl_joint_cmd,
                const EulerAngles& pose_cmd,
                const ImuData& imu,
                const JointState& current_joint_state,
                JointCommand& out_cmd);

private:
    IKSolver ik_solver_;
    PoseBalancer pose_balancer_;
    
    LocomotionMode active_mode_{LocomotionMode::ALGORITHMIC};
    bool in_transition_{false};
    float transition_alpha_{0.0f}; // 0.0 = Algo, 1.0 = RL
    
    JointCommand prev_joint_cmd_;
    
    void applySafetyLimits(const JointState& current_state, JointCommand& cmd);
    void blendCommands(const JointCommand& cmd_algo, const JointCommand& cmd_rl, float alpha, JointCommand& out);
};

} // namespace quadruped::layer2