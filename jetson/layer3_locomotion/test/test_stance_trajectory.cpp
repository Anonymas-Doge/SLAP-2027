#include <gtest/gtest.h>

#include <cmath>

#include "layer3_locomotion/gait/stance_trajectory.hpp"

using namespace quadruped::locomotion;
using quadruped::FootTarget;
using quadruped::LegIndex;
using quadruped::NUM_LEGS;
using quadruped::Twist2D;
using quadruped::Vector3f;

// Expected values below are hand-calculated from:
//   T_stance = β × T
//   position = neutral + v × T_stance × (0.5 − s)
//   velocity = −v
static constexpr float TOL = 1e-6f;

// Trot from A3 Q1: T = 0.5, β = 0.5 → T_stance = 0.25 s
static GaitParams trotParams() {
    GaitParams params;
    params.period = 0.5f;
    params.dutyFactor = 0.5f;
    return params;
}

// Default walk: T = 1.0, β = 0.75 → T_stance = 0.75 s, neutral = (0, 0, −0.20)
static const GaitParams walkParams{};

static void expectVectorNear(const Vector3f& actual, const Vector3f& expected) {
    EXPECT_NEAR(actual.x, expected.x, TOL);
    EXPECT_NEAR(actual.y, expected.y, TOL);
    EXPECT_NEAR(actual.z, expected.z, TOL);
}


// ============================================================================
// FORWARD MOTION (A3 Q1: trot, vx = 0.2 → L = 0.2 × 0.25 = 0.05 m)
// ============================================================================

TEST(StanceTrajectoryTest, ForwardStartsAheadOfHip) {
    // s = 0: x = 0.05 × 0.5 = +0.025 (just touched down, ahead of hip)
    const FootTarget t = computeStanceTarget(Twist2D{0.2f, 0.0f, 0.0f}, trotParams(), LegIndex::FL, 0.0f);
    expectVectorNear(t.position, Vector3f{0.025f, 0.0f, -0.20f});
}

TEST(StanceTrajectoryTest, ForwardMidStanceUnderHip) {
    // s = 0.5: x = 0 (directly under the hip)
    const FootTarget t = computeStanceTarget(Twist2D{0.2f, 0.0f, 0.0f}, trotParams(), LegIndex::FL, 0.5f);
    expectVectorNear(t.position, Vector3f{0.0f, 0.0f, -0.20f});
}

TEST(StanceTrajectoryTest, ForwardEndsBehindHip) {
    // s = 1: x = 0.05 × (−0.5) = −0.025 (about to lift off, behind hip)
    const FootTarget t = computeStanceTarget(Twist2D{0.2f, 0.0f, 0.0f}, trotParams(), LegIndex::FL, 1.0f);
    expectVectorNear(t.position, Vector3f{-0.025f, 0.0f, -0.20f});
}

TEST(StanceTrajectoryTest, ForwardIsLinearInProgress) {
    // s = 0.25: x = 0.05 × 0.25 = +0.0125 (straight line, not a curve)
    const FootTarget t = computeStanceTarget(Twist2D{0.2f, 0.0f, 0.0f}, trotParams(), LegIndex::FL, 0.25f);
    expectVectorNear(t.position, Vector3f{0.0125f, 0.0f, -0.20f});
}

TEST(StanceTrajectoryTest, StepLengthMatchesVelocityTimesStanceTime) {
    // Total travel from s = 0 to s = 1 is L = v × T_stance = 0.05 m
    const Twist2D cmd{0.2f, 0.0f, 0.0f};
    const float start = computeStanceTarget(cmd, trotParams(), LegIndex::FL, 0.0f).position.x;
    const float end = computeStanceTarget(cmd, trotParams(), LegIndex::FL, 1.0f).position.x;
    EXPECT_NEAR(start - end, 0.05f, TOL);
}


// ============================================================================
// SIDEWAYS & COMBINED MOTION
// ============================================================================

TEST(StanceTrajectoryTest, SidewaysMovesFootTowardNegativeY) {
    // A3 Q3: vy = +0.1 (body moves left), walk T_stance = 0.75 → L = 0.075 m
    // Foot slides from y = +0.0375 to y = −0.0375 (toward −y, i.e. right); x unchanged
    const Twist2D cmd{0.0f, 0.1f, 0.0f};
    expectVectorNear(computeStanceTarget(cmd, walkParams, LegIndex::FL, 0.0f).position,
                     Vector3f{0.0f, 0.0375f, -0.20f});
    expectVectorNear(computeStanceTarget(cmd, walkParams, LegIndex::FL, 1.0f).position,
                     Vector3f{0.0f, -0.0375f, -0.20f});
}

TEST(StanceTrajectoryTest, DiagonalCombinesBothAxes) {
    // vx = 0.2, vy = −0.1, walk T_stance = 0.75, s = 0:
    //   x = 0.2 × 0.75 × 0.5 = +0.075,  y = −0.1 × 0.75 × 0.5 = −0.0375
    const FootTarget t = computeStanceTarget(Twist2D{0.2f, -0.1f, 0.0f}, walkParams, LegIndex::RR, 0.0f);
    expectVectorNear(t.position, Vector3f{0.075f, -0.0375f, -0.20f});
}

TEST(StanceTrajectoryTest, HeightStaysAtGroundLevel) {
    // z never changes during stance, for any s
    const Twist2D cmd{0.3f, 0.2f, 0.0f};
    for (const float s : {0.0f, 0.3f, 0.7f, 1.0f}) {
        EXPECT_NEAR(computeStanceTarget(cmd, walkParams, LegIndex::FL, s).position.z, -0.20f, TOL)
            << "s = " << s;
    }
}


// ============================================================================
// VELOCITY & STANCE FLAG
// ============================================================================

TEST(StanceTrajectoryTest, VelocityIsNegativeCommand) {
    // Planted foot moves backward relative to the hip at −v, no vertical motion
    const FootTarget t = computeStanceTarget(Twist2D{0.2f, -0.1f, 0.0f}, walkParams, LegIndex::FL, 0.4f);
    expectVectorNear(t.velocity, Vector3f{-0.2f, 0.1f, 0.0f});
}

TEST(StanceTrajectoryTest, MarksTargetAsStance) {
    const FootTarget t = computeStanceTarget(Twist2D{0.2f, 0.0f, 0.0f}, walkParams, LegIndex::FL, 0.5f);
    EXPECT_TRUE(t.is_stance);
}


// ============================================================================
// EDGE CASES
// ============================================================================

TEST(StanceTrajectoryTest, ZeroVelocityStaysAtNeutral) {
    const Twist2D still{0.0f, 0.0f, 0.0f};
    for (const float s : {0.0f, 0.5f, 1.0f}) {
        const FootTarget t = computeStanceTarget(still, walkParams, LegIndex::FL, s);
        expectVectorNear(t.position, Vector3f{0.0f, 0.0f, -0.20f});
        expectVectorNear(t.velocity, Vector3f{0.0f, 0.0f, 0.0f});
    }
}

TEST(StanceTrajectoryTest, ProgressIsClampedToZeroAndOne) {
    const Twist2D cmd{0.2f, 0.0f, 0.0f};
    expectVectorNear(computeStanceTarget(cmd, trotParams(), LegIndex::FL, -0.5f).position,
                     computeStanceTarget(cmd, trotParams(), LegIndex::FL, 0.0f).position);
    expectVectorNear(computeStanceTarget(cmd, trotParams(), LegIndex::FL, 1.5f).position,
                     computeStanceTarget(cmd, trotParams(), LegIndex::FL, 1.0f).position);
}

TEST(StanceTrajectoryTest, EachLegUsesItsOwnNeutral) {
    // Distinct placeholder neutrals per leg; zero velocity so position == neutral
    GaitParams params;
    params.neutralPositions = {Vector3f{0.01f, 0.02f, -0.20f},
                               Vector3f{0.03f, -0.04f, -0.21f},
                               Vector3f{-0.05f, 0.06f, -0.22f},
                               Vector3f{-0.07f, -0.08f, -0.23f}};

    for (std::size_t i = 0; i < NUM_LEGS; ++i) {
        const FootTarget t = computeStanceTarget(Twist2D{}, params, static_cast<LegIndex>(i), 0.5f);
        expectVectorNear(t.position, params.neutralPositions[i]);
    }
}

// ============================================================================
// TURNING (A4)
//   v_leg = (vx − ωz·ry, vy + ωz·rx),  r = hip + neutral (x, y)
//   Default hips: FL (+0.15, +0.08), FR (+0.15, −0.08), RL (−0.15, +0.08), RR (−0.15, −0.08)
// ============================================================================

TEST(StanceTrajectoryTest, TurnInPlaceFootVelocities) {
    // ωz = +0.5 (turning left), neutral x = y = 0 → r = hip
    //   FL: v_leg = (−0.5·0.08, 0.5·0.15) = (−0.04, +0.075) → foot velocity (+0.04, −0.075)
    //   FR: v_leg = (+0.04, +0.075) → (−0.04, −0.075)
    //   RL: v_leg = (−0.04, −0.075) → (+0.04, +0.075)
    //   RR: v_leg = (+0.04, −0.075) → (−0.04, +0.075)
    const Twist2D turnLeft{0.0f, 0.0f, 0.5f};
    expectVectorNear(computeStanceTarget(turnLeft, walkParams, LegIndex::FL, 0.5f).velocity,
                     Vector3f{0.04f, -0.075f, 0.0f});
    expectVectorNear(computeStanceTarget(turnLeft, walkParams, LegIndex::FR, 0.5f).velocity,
                     Vector3f{-0.04f, -0.075f, 0.0f});
    expectVectorNear(computeStanceTarget(turnLeft, walkParams, LegIndex::RL, 0.5f).velocity,
                     Vector3f{0.04f, 0.075f, 0.0f});
    expectVectorNear(computeStanceTarget(turnLeft, walkParams, LegIndex::RR, 0.5f).velocity,
                     Vector3f{-0.04f, 0.075f, 0.0f});
}

TEST(StanceTrajectoryTest, TurnInPlaceFootPath) {
    // FL, walk T_stance = 0.75: start = v_leg × 0.75 × 0.5 = (−0.015, +0.028125)
    // Foot slides from behind-left to ahead-right of the hip (forward and right)
    const Twist2D turnLeft{0.0f, 0.0f, 0.5f};
    expectVectorNear(computeStanceTarget(turnLeft, walkParams, LegIndex::FL, 0.0f).position,
                     Vector3f{-0.015f, 0.028125f, -0.20f});
    expectVectorNear(computeStanceTarget(turnLeft, walkParams, LegIndex::FL, 1.0f).position,
                     Vector3f{0.015f, -0.028125f, -0.20f});
}

TEST(StanceTrajectoryTest, TurnInPlaceAllFeetSameSpeed) {
    // Hips are equidistant from the center → |v_leg| = 0.5 × √(0.15² + 0.08²) = 0.085 for every leg
    const Twist2D turnLeft{0.0f, 0.0f, 0.5f};
    for (std::size_t i = 0; i < NUM_LEGS; ++i) {
        const Vector3f v = computeStanceTarget(turnLeft, walkParams, static_cast<LegIndex>(i), 0.5f).velocity;
        EXPECT_NEAR(std::hypot(v.x, v.y), 0.085f, TOL) << "leg index " << i;
    }
}

TEST(StanceTrajectoryTest, TurnRightMirrorsTurnLeft) {
    // Reversing ωz reverses every foot velocity
    for (std::size_t i = 0; i < NUM_LEGS; ++i) {
        const auto leg = static_cast<LegIndex>(i);
        const Vector3f left = computeStanceTarget(Twist2D{0.0f, 0.0f, 0.5f}, walkParams, leg, 0.5f).velocity;
        const Vector3f right = computeStanceTarget(Twist2D{0.0f, 0.0f, -0.5f}, walkParams, leg, 0.5f).velocity;
        expectVectorNear(right, Vector3f{-left.x, -left.y, 0.0f});
    }
}

TEST(StanceTrajectoryTest, ForwardPlusTurnCurvesLeft) {
    // vx = 0.2, ωz = +0.5:
    //   FR: v_leg = (0.2 + 0.04, 0.075) = (0.24, 0.075) → right feet slide back faster
    //   FL: v_leg = (0.2 − 0.04, 0.075) = (0.16, 0.075) → left feet slide back slower
    // Faster right side + slower left side = robot curves left
    const Twist2D curveLeft{0.2f, 0.0f, 0.5f};
    expectVectorNear(computeStanceTarget(curveLeft, walkParams, LegIndex::FR, 0.5f).velocity,
                     Vector3f{-0.24f, -0.075f, 0.0f});
    expectVectorNear(computeStanceTarget(curveLeft, walkParams, LegIndex::FL, 0.5f).velocity,
                     Vector3f{-0.16f, -0.075f, 0.0f});
}

TEST(StanceTrajectoryTest, TurningUsesHipPlusNeutral) {
    // Feet not directly under the hips (catches r = hip − neutral sign errors):
    //   FL hip (0.15, 0.08) + neutral (0.01, 0.02) → r = (0.16, 0.10)
    //   ωz = 0.5 → v_leg = (−0.05, +0.08) → foot velocity (+0.05, −0.08)
    GaitParams params;
    params.neutralPositions[static_cast<std::size_t>(LegIndex::FL)] = Vector3f{0.01f, 0.02f, -0.20f};
    const FootTarget t = computeStanceTarget(Twist2D{0.0f, 0.0f, 0.5f}, params, LegIndex::FL, 0.5f);
    expectVectorNear(t.velocity, Vector3f{0.05f, -0.08f, 0.0f});
    expectVectorNear(t.position, Vector3f{0.01f, 0.02f, -0.20f});  // s = 0.5 is still at neutral
}
