#include <gtest/gtest.h>

#include "layer3_locomotion/gait/swing_trajectory.hpp"

using namespace quadruped::locomotion;
using quadruped::FootTarget;
using quadruped::Vector3f;

// Expected values below are hand-calculated from:
//   H  = (4/3) × h,  P1 = P0 + (0, 0, H),  P2 = P3 + (0, 0, H)
//   B(s)  = (1−s)³·P0 + 3s(1−s)²·P1 + 3s²(1−s)·P2 + s³·P3
//   B'(s) = 3(1−s)²·(P1 − P0) + 6s(1−s)·(P2 − P1) + 3s²·(P3 − P2)
//   velocity = B'(s) / T_swing,  T_swing = (1 − β) × T
//   flat-ground shortcut: z(s) = ground + H × 3s(1 − s)
static constexpr float TOL = 1e-6f;

// B6 example: 8 cm step centered under the hip, foot 0.20 m below the hip
static const Vector3f P0{-0.04f, 0.0f, -0.20f};
static const Vector3f P3{0.04f, 0.0f, -0.20f};

// Default walk (T = 1.0, β = 0.75 → T_swing = 0.25 s) with h = 0.045 → H = 0.06
static GaitParams swingParams() {
    GaitParams params;
    params.stepHeight = 0.045f;
    return params;
}

static void expectVectorNear(const Vector3f& actual, const Vector3f& expected) {
    EXPECT_NEAR(actual.x, expected.x, TOL);
    EXPECT_NEAR(actual.y, expected.y, TOL);
    EXPECT_NEAR(actual.z, expected.z, TOL);
}


// ============================================================================
// POSITION
// ============================================================================

TEST(SwingTrajectoryTest, StartsAtLiftOffPoint) {
    expectVectorNear(computeSwingTarget(swingParams(), P0, P3, 0.0f).position, P0);
}

TEST(SwingTrajectoryTest, EndsAtTouchDownPoint) {
    expectVectorNear(computeSwingTarget(swingParams(), P0, P3, 1.0f).position, P3);
}

TEST(SwingTrajectoryTest, PeakClearsExactlyStepHeight) {
    // s = 0.5: x midway (0), z = −0.20 + 0.75 × 0.06 = −0.155 → clears h = 0.045
    const FootTarget t = computeSwingTarget(swingParams(), P0, P3, 0.5f);
    expectVectorNear(t.position, Vector3f{0.0f, 0.0f, -0.155f});
}

TEST(SwingTrajectoryTest, QuarterSwingMatchesHandCalculation) {
    // B6 Q2, s = 0.25 with weights (0.421875, 0.421875, 0.140625, 0.015625):
    //   x = 0.421875(−0.04) + 0.421875(−0.04) + 0.140625(0.04) + 0.015625(0.04) = −0.0275
    //   z = −0.20 + 0.06 × 3(0.25)(0.75) = −0.16625   (shortcut, independent of B(s) code)
    const FootTarget t = computeSwingTarget(swingParams(), P0, P3, 0.25f);
    expectVectorNear(t.position, Vector3f{-0.0275f, 0.0f, -0.16625f});
}

TEST(SwingTrajectoryTest, HeightIsSymmetricOnFlatGround) {
    // z(s) = z(1 − s) because 3s(1 − s) is symmetric
    for (const float s : {0.1f, 0.2f, 0.35f}) {
        EXPECT_NEAR(computeSwingTarget(swingParams(), P0, P3, s).position.z,
                    computeSwingTarget(swingParams(), P0, P3, 1.0f - s).position.z, TOL)
            << "s = " << s;
    }
}

TEST(SwingTrajectoryTest, NeverDipsBelowGround) {
    for (int i = 0; i <= 20; ++i) {
        const float s = static_cast<float>(i) / 20.0f;
        EXPECT_GE(computeSwingTarget(swingParams(), P0, P3, s).position.z, -0.20f - TOL) << "s = " << s;
    }
}

TEST(SwingTrajectoryTest, HandlesSidewaysAndUnevenEndpoints) {
    // General Bézier: endpoints still hit exactly even with y motion and different heights
    const Vector3f liftOff{-0.03f, 0.01f, -0.20f};
    const Vector3f touchDown{0.05f, -0.02f, -0.18f};
    expectVectorNear(computeSwingTarget(swingParams(), liftOff, touchDown, 0.0f).position, liftOff);
    expectVectorNear(computeSwingTarget(swingParams(), liftOff, touchDown, 1.0f).position, touchDown);

    // s = 0.5: x and y are the plain average (both control points share their endpoint's x, y)
    const Vector3f mid = computeSwingTarget(swingParams(), liftOff, touchDown, 0.5f).position;
    EXPECT_NEAR(mid.x, 0.01f, TOL);
    EXPECT_NEAR(mid.y, -0.005f, TOL);
}


// ============================================================================
// VELOCITY (T_swing = 0.25 s)
// ============================================================================

TEST(SwingTrajectoryTest, LiftsOffStraightUp) {
    // B'(0) = 3(P1 − P0) = (0, 0, 0.18) → / 0.25 = (0, 0, 0.72)
    expectVectorNear(computeSwingTarget(swingParams(), P0, P3, 0.0f).velocity, Vector3f{0.0f, 0.0f, 0.72f});
}

TEST(SwingTrajectoryTest, TouchesDownStraightDown) {
    // B'(1) = 3(P3 − P2) = (0, 0, −0.18) → / 0.25 = (0, 0, −0.72)
    expectVectorNear(computeSwingTarget(swingParams(), P0, P3, 1.0f).velocity, Vector3f{0.0f, 0.0f, -0.72f});
}

TEST(SwingTrajectoryTest, MovesHorizontallyAtPeak) {
    // B'(0.5) = 0.75(P1 − P0) + 1.5(P2 − P1) + 0.75(P3 − P2)
    //         = (0, 0, 0.045) + (0.12, 0, 0) + (0, 0, −0.045) = (0.12, 0, 0) → / 0.25 = (0.48, 0, 0)
    // Peak forward speed is 1.5× the average 0.08 m / 0.25 s = 0.32 m/s
    expectVectorNear(computeSwingTarget(swingParams(), P0, P3, 0.5f).velocity, Vector3f{0.48f, 0.0f, 0.0f});
}

TEST(SwingTrajectoryTest, VelocityScalesWithSwingTime) {
    // Halving T halves T_swing (0.125 s), doubling the velocity
    GaitParams fast = swingParams();
    fast.period = 0.5f;
    expectVectorNear(computeSwingTarget(fast, P0, P3, 0.5f).velocity, Vector3f{0.96f, 0.0f, 0.0f});
}


// ============================================================================
// FLAGS, CLAMPING & VALIDATION
// ============================================================================

TEST(SwingTrajectoryTest, MarksTargetAsSwing) {
    EXPECT_FALSE(computeSwingTarget(swingParams(), P0, P3, 0.5f).is_stance);
}

TEST(SwingTrajectoryTest, ProgressIsClampedToZeroAndOne) {
    expectVectorNear(computeSwingTarget(swingParams(), P0, P3, -0.5f).position, P0);
    expectVectorNear(computeSwingTarget(swingParams(), P0, P3, 1.5f).position, P3);
}

TEST(SwingTrajectoryTest, NonPositiveStepHeightIsRejected) {
    for (const float h : {0.0f, -0.01f}) {
        GaitParams params = swingParams();
        params.stepHeight = h;
        EXPECT_THROW(validateGaitParams(params), std::invalid_argument) << "h = " << h;
    }
    EXPECT_NO_THROW(validateGaitParams(swingParams()));
}
