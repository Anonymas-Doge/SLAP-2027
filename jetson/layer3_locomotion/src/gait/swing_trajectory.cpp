#include <array>
#include <algorithm>

#include "layer3_locomotion/gait/swing_trajectory.hpp"

static constexpr int NUM_BEZIER_CONTROL_POINTS = 4;


namespace quadruped::locomotion {

struct BezierTargets {
    quadruped::Vector3f positon;
    quadruped::Vector3f derivative;
};


static float computeSingleBezierPoint(const std::array<float, NUM_BEZIER_CONTROL_POINTS>& points, const float s) {
    const float u = 1-s;

    return u*u*u*points[0] +
           3*s*u*u*points[1] +
           3*s*s*u*points[2] +
           s*s*s*points[3];
}


static float computeSingleBezierDerivative(const std::array<float, NUM_BEZIER_CONTROL_POINTS>& points, const float s) {
    const float u = 1-s;

    return 3*u*u*(points[1]-points[0]) +
           6*s*u*(points[2]-points[1]) +
           3*s*s*(points[3]-points[2]);
}


// Computes B(s) = (1−s)³·P0 + 3s(1−s)²·P1 + 3s²(1−s)·P2 + s³·P3 AND
//          B'(s) = 3(1−s)²·(P1 − P0) + 6s(1−s)·(P2 − P1) + 3s²·(P3 − P2)
static BezierTargets computeBezierTarget(const std::array<Vector3f, NUM_BEZIER_CONTROL_POINTS>& positions, const float s) {


    const Vector3f position{computeSingleBezierPoint({positions[0].x,
                                          positions[1].x,
                                          positions[2].x,
                          positions[3].x}, s),
                            computeSingleBezierPoint({positions[0].y,
                                          positions[1].y,
                                          positions[2].y,
                                          positions[3].y}, s),
                            computeSingleBezierPoint({positions[0].z,
                                          positions[1].z,
                                          positions[2].z,
                                          positions[3].z}, s)};

    const Vector3f derivative{computeSingleBezierDerivative({positions[0].x,
                                                                positions[1].x,
                                                                positions[2].x,
                                                                positions[3].x}, s),
                              computeSingleBezierDerivative({positions[0].y,
                                                                positions[1].y,
                                                                positions[2].y,
                                                                positions[3].y}, s),
                              computeSingleBezierDerivative({positions[0].z,
                                                                positions[1].z,
                                                                positions[2].z,
                                                                positions[3].z}, s)};

    return {position, derivative};
}


FootTarget computeSwingTarget(const GaitParams &params, const Vector3f &p0, const Vector3f &p3, const float progress) {

    const float s = std::clamp(progress, 0.0f, 1.0f); // Clamps to handle float values like 1.000000001
    const float controlPointHeight = params.stepHeight * (4.0f / 3.0f); // H = (4/3) * h
    const float T_swing = (1 - params.dutyFactor) * params.period;

    const Vector3f p1{p0.x, p0.y, p0.z + controlPointHeight};
    const Vector3f p2{p3.x, p3.y, p3.z + controlPointHeight};

    auto [positon, derivative]{computeBezierTarget({p0, p1, p2, p3}, s)};
    const Vector3f velocity{derivative.x/T_swing, derivative.y/T_swing, derivative.z/T_swing};

    return {positon, velocity, false};
}


} // namespace quadruped::locomotion