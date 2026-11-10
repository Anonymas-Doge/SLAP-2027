#include <gtest/gtest.h>
#include <array>

#include "layer3_locomotion/gait/phase_clock.hpp"

using namespace quadruped::locomotion;
using quadruped::LegIndex;
using quadruped::NUM_LEGS;

// Walk gait: T = 1.0 s, β = 0.75, lift-off order RL → FL → RR → FR
static const GaitParams defaultParams{1.0f, 0.75f, {0.5f, 0.0f, 0.75f, 0.25f}};

// Expected values below are hand-calculated from the phase clock formulas
static constexpr float TOL = 1e-5f;


// ============================================================================
// CONSTRUCTION & VALIDATION
// ============================================================================

TEST(PhaseClockTest, PhaseClockConstruction) {
    EXPECT_NO_THROW(PhaseClock clock{defaultParams});
}

TEST(PhaseClockTest, PhaseClockInvalidPeriod) {
    GaitParams zero = defaultParams;
    zero.period = 0.0f;
    EXPECT_THROW(PhaseClock clock{zero}, std::invalid_argument);

    GaitParams negative = defaultParams;
    negative.period = -1.0f;
    EXPECT_THROW(PhaseClock clock{negative}, std::invalid_argument);
}

TEST(PhaseClockTest, PhaseClockInvalidDutyFactor) {
    for (const float beta : {0.0f, 1.0f, -0.5f, 2.0f}) {
        GaitParams params = defaultParams;
        params.dutyFactor = beta;
        EXPECT_THROW(PhaseClock clock{params}, std::invalid_argument) << "beta = " << beta;
    }
}

TEST(PhaseClockTest, PhaseClockInvalidOffsets) {
    for (const float offset : {1.0f, -0.1f, 3.1f}) {
        GaitParams params = defaultParams;
        params.offsets[static_cast<std::size_t>(LegIndex::FL)] = offset;
        EXPECT_THROW(PhaseClock clock{params}, std::invalid_argument) << "offset = " << offset;
    }
}

TEST(PhaseClockTest, PhaseClockZeroOffsetIsValid) {
    // Lower boundary of [0, 1) is allowed (FR uses it in the walk)
    GaitParams params = defaultParams;
    params.offsets.fill(0.0f);
    EXPECT_NO_THROW(PhaseClock clock{params});
}

TEST(PhaseClockTest, PhaseClockNegativeDeltaThrows) {
    PhaseClock clock{defaultParams};
    EXPECT_THROW(clock.advance(-0.01f), std::invalid_argument);
}


// ============================================================================
// FIRST TICK FROM RESET (Δt = 0.01 → φ_global = 0.01)
// ============================================================================

TEST(PhaseClockTest, PhaseClockAdvanceFrontLegs) {
    PhaseClock clock{defaultParams};
    clock.advance(0.01f);

    // Front Left: φ = 0.51 → stance, progress = 0.51 / 0.75
    const LegPhaseState fl = clock.getLegState(LegIndex::FL);
    EXPECT_NEAR(fl.progress, 0.68f, TOL);
    EXPECT_TRUE(fl.inStance);
    EXPECT_FALSE(fl.liftOff);
    EXPECT_FALSE(fl.touchDown);

    // Front Right: φ = 0.01 → stance, progress = 0.01 / 0.75
    const LegPhaseState fr = clock.getLegState(LegIndex::FR);
    EXPECT_NEAR(fr.progress, 0.01f / 0.75f, TOL);
    EXPECT_TRUE(fr.inStance);
    EXPECT_FALSE(fr.liftOff);
    EXPECT_FALSE(fr.touchDown);
}

TEST(PhaseClockTest, PhaseClockAdvanceRearLegs) {
    PhaseClock clock{defaultParams};
    clock.advance(0.01f);

    // Rear Left: φ = 0.76 → swing, progress = (0.76 - 0.75) / 0.25; just lifted off
    const LegPhaseState rl = clock.getLegState(LegIndex::RL);
    EXPECT_NEAR(rl.progress, 0.04f, TOL);
    EXPECT_FALSE(rl.inStance);
    EXPECT_TRUE(rl.liftOff);
    EXPECT_FALSE(rl.touchDown);

    // Rear Right: φ = 0.26 → stance, progress = 0.26 / 0.75
    const LegPhaseState rr = clock.getLegState(LegIndex::RR);
    EXPECT_NEAR(rr.progress, 0.26f / 0.75f, TOL);
    EXPECT_TRUE(rr.inStance);
    EXPECT_FALSE(rr.liftOff);
    EXPECT_FALSE(rr.touchDown);
}


// ============================================================================
// EVENTS & TIMING
// ============================================================================

TEST(PhaseClockTest, PhaseClockAdvanceTwice) {
    // liftOff is only true on the tick the switch happens
    PhaseClock clock{defaultParams};
    clock.advance(0.01f);
    clock.advance(0.01f);

    const LegPhaseState rl = clock.getLegState(LegIndex::RL);
    EXPECT_FALSE(rl.inStance);
    EXPECT_FALSE(rl.liftOff);
}

TEST(PhaseClockTest, PhaseClockLiftOffOrderOverOneCycle) {
    // Lift-off happens when φ_global = frac(β - offset):
    //   RL 0.00, FL 0.25, RR 0.50, FR 0.75
    // 99 ticks of 0.01 covers one cycle without reaching RL's next lift-off at 1.0
    constexpr float dt = 0.01f;
    constexpr int ticks = 99;

    PhaseClock clock{defaultParams};
    std::array<int, NUM_LEGS> liftOffCount{};
    std::array<float, NUM_LEGS> liftOffTime{};

    for (int tick = 1; tick <= ticks; ++tick) {
        clock.advance(dt);
        for (std::size_t i = 0; i < NUM_LEGS; ++i) {
            if (clock.getLegState(static_cast<LegIndex>(i)).liftOff) {
                ++liftOffCount[i];
                liftOffTime[i] = tick * dt;
            }
        }
    }

    // Each leg lifts off exactly once per cycle
    for (std::size_t i = 0; i < NUM_LEGS; ++i) {
        EXPECT_EQ(liftOffCount[i], 1) << "leg index " << i;
    }

    // Detected on the first tick at or after the expected phase (±1 tick for float rounding)
    const auto at = [&](LegIndex leg) { return liftOffTime[static_cast<std::size_t>(leg)]; };
    EXPECT_NEAR(at(LegIndex::RL), 0.00f + dt, dt + TOL);
    EXPECT_NEAR(at(LegIndex::FL), 0.25f + dt, dt + TOL);
    EXPECT_NEAR(at(LegIndex::RR), 0.50f + dt, dt + TOL);
    EXPECT_NEAR(at(LegIndex::FR), 0.75f + dt, dt + TOL);

    EXPECT_LT(at(LegIndex::RL), at(LegIndex::FL));
    EXPECT_LT(at(LegIndex::FL), at(LegIndex::RR));
    EXPECT_LT(at(LegIndex::RR), at(LegIndex::FR));
}

TEST(PhaseClockTest, PhaseClockTouchDownAfterSwing) {
    // RL lifts at φ_global = 0 and swings for (1 - β) = 0.25 of the cycle,
    // so it touches down once, at φ_global ≈ 0.25
    constexpr float dt = 0.01f;

    PhaseClock clock{defaultParams};
    int touchDownCount = 0;
    float touchDownTime = -1.0f;

    for (int tick = 1; tick <= 99; ++tick) {
        clock.advance(dt);
        const LegPhaseState rl = clock.getLegState(LegIndex::RL);
        if (rl.touchDown) {
            ++touchDownCount;
            touchDownTime = tick * dt;
            EXPECT_TRUE(rl.inStance);
        }
    }

    EXPECT_EQ(touchDownCount, 1);
    EXPECT_NEAR(touchDownTime, 0.25f + dt, dt + TOL);
}

TEST(PhaseClockTest, PhaseClockWrapsPastOneCycle) {
    // 0.6 + 0.6 = 1.2 → φ_global wraps to 0.2
    PhaseClock clock{defaultParams};
    clock.advance(0.6f);
    clock.advance(0.6f);

    // FR has offset 0, so its phase equals the global phase
    EXPECT_NEAR(clock.getLegState(LegIndex::FR).legPhase, 0.2f, TOL);

    for (std::size_t i = 0; i < NUM_LEGS; ++i) {
        const float phase = clock.getLegState(static_cast<LegIndex>(i)).legPhase;
        EXPECT_GE(phase, 0.0f) << "leg index " << i;
        EXPECT_LT(phase, 1.0f) << "leg index " << i;
    }
}

TEST(PhaseClockTest, PhaseClockPeriodScalesTime) {
    // T = 2.0: Δt = 0.5 is a quarter cycle → FR (offset 0) at φ = 0.25
    GaitParams params = defaultParams;
    params.period = 2.0f;
    PhaseClock clock{params};
    clock.advance(0.5f);

    EXPECT_NEAR(clock.getLegState(LegIndex::FR).legPhase, 0.25f, TOL);
}


// ============================================================================
// RESET
// ============================================================================

TEST(PhaseClockTest, PhaseClockResetRestoresDefaults) {
    PhaseClock clock{defaultParams};
    clock.advance(0.3f);
    clock.reset();

    for (std::size_t i = 0; i < NUM_LEGS; ++i) {
        const LegPhaseState leg = clock.getLegState(static_cast<LegIndex>(i));
        EXPECT_TRUE(leg.inStance) << "leg index " << i;
        EXPECT_FALSE(leg.liftOff) << "leg index " << i;
        EXPECT_FALSE(leg.touchDown) << "leg index " << i;
        EXPECT_FLOAT_EQ(leg.progress, 0.0f) << "leg index " << i;
        EXPECT_FLOAT_EQ(leg.legPhase, 0.0f) << "leg index " << i;
    }
}

TEST(PhaseClockTest, PhaseClockResetBehavesLikeNewClock) {
    // After reset, the first tick matches a freshly constructed clock: RL lifts off
    PhaseClock clock{defaultParams};
    clock.advance(0.3f);
    clock.reset();
    clock.advance(0.01f);

    const LegPhaseState rl = clock.getLegState(LegIndex::RL);
    EXPECT_TRUE(rl.liftOff);
    EXPECT_NEAR(rl.progress, 0.04f, TOL);
}
