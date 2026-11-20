#include <gtest/gtest.h>

#include "layer3_locomotion/gait/leg_velocity.hpp"

using namespace quadruped::locomotion;
using quadruped::Twist2D;
using quadruped::Vector3f;

// Expected values below are hand-calculated from:
//   r     = hip + neutral (x, y)
//   v_leg = (vx − ωz·ry, vy + ωz·rx)
static constexpr float TOL = 1e-6f;

static const Vector3f FL_HIP{0.15f, 0.08f, 0.0f};
static const Vector3f UNDER_HIP{0.0f, 0.0f, -0.20f};

static void expectPlanarNear(const Twist2D& actual, float vx, float vy) {
    EXPECT_NEAR(actual.vx, vx, TOL);
    EXPECT_NEAR(actual.vy, vy, TOL);
}


TEST(LegVelocityTest, NoTurnPassesBodyVelocityThrough) {
    // ωz = 0 → every point on the body moves with the body
    expectPlanarNear(computeLegVelocity(Twist2D{0.2f, -0.1f, 0.0f}, FL_HIP, UNDER_HIP), 0.2f, -0.1f);
}

TEST(LegVelocityTest, PureTurnIsPerpendicularToRadius) {
    // r = (0.15, 0.08), ωz = 0.5 → (−0.04, +0.075)
    const Twist2D v = computeLegVelocity(Twist2D{0.0f, 0.0f, 0.5f}, FL_HIP, UNDER_HIP);
    expectPlanarNear(v, -0.04f, 0.075f);

    // Rotation velocity is perpendicular to r: r · v = 0
    EXPECT_NEAR(0.15f * v.vx + 0.08f * v.vy, 0.0f, TOL);
}

TEST(LegVelocityTest, SpeedScalesWithDistanceFromCenter) {
    // Doubling the radius doubles the rotational speed
    const Vector3f farHip{0.30f, 0.16f, 0.0f};
    const Twist2D near = computeLegVelocity(Twist2D{0.0f, 0.0f, 0.5f}, FL_HIP, UNDER_HIP);
    const Twist2D far = computeLegVelocity(Twist2D{0.0f, 0.0f, 0.5f}, farHip, UNDER_HIP);
    expectPlanarNear(far, 2.0f * near.vx, 2.0f * near.vy);
}

TEST(LegVelocityTest, PointAtCenterDoesNotRotate) {
    // r = 0 → turning contributes nothing
    const Vector3f center{0.0f, 0.0f, 0.0f};
    expectPlanarNear(computeLegVelocity(Twist2D{0.1f, 0.05f, 2.0f}, center, UNDER_HIP), 0.1f, 0.05f);
}

TEST(LegVelocityTest, NeutralOffsetAddsToHipPosition) {
    // hip (0.15, 0.08) + neutral (0.01, 0.02) → r = (0.16, 0.10)
    // ωz = 0.5 → (−0.05, +0.08)
    const Vector3f outward{0.01f, 0.02f, -0.20f};
    expectPlanarNear(computeLegVelocity(Twist2D{0.0f, 0.0f, 0.5f}, FL_HIP, outward), -0.05f, 0.08f);
}

TEST(LegVelocityTest, TranslationAndRotationAdd) {
    // vx = 0.2, vy = 0.1, ωz = 0.5 at r = (0.15, 0.08):
    //   (0.2 − 0.04, 0.1 + 0.075) = (0.16, 0.175)
    expectPlanarNear(computeLegVelocity(Twist2D{0.2f, 0.1f, 0.5f}, FL_HIP, UNDER_HIP), 0.16f, 0.175f);
}

TEST(LegVelocityTest, IgnoresHeight) {
    // z of hip or neutral never affects planar velocity
    const Vector3f highHip{0.15f, 0.08f, 0.5f};
    const Vector3f deepFoot{0.0f, 0.0f, -1.0f};
    expectPlanarNear(computeLegVelocity(Twist2D{0.0f, 0.0f, 0.5f}, highHip, deepFoot), -0.04f, 0.075f);
}
