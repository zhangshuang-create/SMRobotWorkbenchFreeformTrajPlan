#pragma once
#include <Eigen/Dense>

namespace ModelTrajPlan {
// Axis codes: X+, X-, Y+, Y-, Z+, Z-.
inline bool validTrajectoryAxes(int normal, int forward) {
    return normal >= 0 && normal < 6 && forward >= 0 && forward < 6 && normal / 2 != forward / 2;
}
inline Eigen::Matrix3d trajectoryFrameMapping(int normal, int forward) {
    Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
    result.col(normal / 2) = (normal % 2 ? -1.0 : 1.0) * Eigen::Vector3d::UnitZ();
    result.col(forward / 2) = (forward % 2 ? -1.0 : 1.0) * Eigen::Vector3d::UnitX();
    const int remaining = 3 - normal / 2 - forward / 2;
    result.col(remaining) = result.col((remaining + 1) % 3).cross(result.col((remaining + 2) % 3));
    return result;
}
}
