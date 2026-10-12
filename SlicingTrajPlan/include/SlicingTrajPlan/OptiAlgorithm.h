#pragma once

#include <Eigen/Dense>

#include <vector>

namespace ModelTrajPlan
{
    struct Triangle
    {
        Eigen::Vector3d v0;
        Eigen::Vector3d v1;
        Eigen::Vector3d v2;
        Eigen::Vector3d normal;

        Triangle(const Eigen::Vector3d& first, const Eigen::Vector3d& second,
            const Eigen::Vector3d& third);
    };

    struct RRTNode
    {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d orientationRadians = Eigen::Vector3d::Zero();
        int nodeId = -1;

        RRTNode() = default;
        RRTNode(const Eigen::Vector3d& nodePosition,
            const Eigen::Vector3d& nodeOrientationRadians, int id = -1);

        Eigen::Matrix4d toMatrix4d() const;
    };

    // Pure pose-selection algorithms. Collision checks, robot loading and scene updates
    // are platform concerns and are intentionally not represented in this document module.
    class OptiAlgorithm
    {
    public:
        static bool selectContinuousSolution(
            const std::vector<Eigen::Matrix4d>& solutions,
            const Eigen::Matrix4d& originalPose,
            const Eigen::Matrix4d* previousPose,
            Eigen::Matrix4d& selectedPose);
    };
}
