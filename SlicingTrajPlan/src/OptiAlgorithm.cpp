#include <SlicingTrajPlan/OptiAlgorithm.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ModelTrajPlan
{
    Triangle::Triangle(const Eigen::Vector3d& first, const Eigen::Vector3d& second,
        const Eigen::Vector3d& third)
        : v0(first)
        , v1(second)
        , v2(third)
    {
        const Eigen::Vector3d cross = (v1 - v0).cross(v2 - v0);
        normal = cross.squaredNorm() > 0.0 ? cross.normalized() : Eigen::Vector3d::Zero();
    }

    RRTNode::RRTNode(const Eigen::Vector3d& nodePosition,
        const Eigen::Vector3d& nodeOrientationRadians, int id)
        : position(nodePosition)
        , orientationRadians(nodeOrientationRadians)
        , nodeId(id)
    {
    }

    Eigen::Matrix4d RRTNode::toMatrix4d() const
    {
        const Eigen::AngleAxisd roll(orientationRadians.x(), Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd pitch(orientationRadians.y(), Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd yaw(orientationRadians.z(), Eigen::Vector3d::UnitZ());

        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
        pose.block<3, 1>(0, 3) = position;
        return pose;
    }

    bool OptiAlgorithm::selectContinuousSolution(
        const std::vector<Eigen::Matrix4d>& solutions,
        const Eigen::Matrix4d& originalPose,
        const Eigen::Matrix4d* previousPose,
        Eigen::Matrix4d& selectedPose)
    {
        if(solutions.empty()) {
            return false;
        }

        const Eigen::Matrix4d& reference = previousPose != nullptr ? *previousPose : originalPose;
        double bestCost = std::numeric_limits<double>::infinity();
        const Eigen::Matrix4d* best = nullptr;
        for(const Eigen::Matrix4d& candidate : solutions) {
            const double translationCost = (candidate.block<3, 1>(0, 3) -
                reference.block<3, 1>(0, 3)).squaredNorm();
            const Eigen::Matrix3d rotationDelta = reference.block<3, 3>(0, 0).transpose() *
                candidate.block<3, 3>(0, 0);
            const double cosine = std::clamp((rotationDelta.trace() - 1.0) * 0.5, -1.0, 1.0);
            const double rotationCost = std::acos(cosine);
            const double cost = translationCost + rotationCost * rotationCost;
            if(cost < bestCost) {
                bestCost = cost;
                best = &candidate;
            }
        }

        selectedPose = *best;
        return true;
    }
}
