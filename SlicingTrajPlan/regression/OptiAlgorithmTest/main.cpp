#include <SlicingTrajPlan/OptiAlgorithm.h>

#include <cmath>
#include <iostream>

namespace
{
    bool nearlyEqual(double left, double right, double tolerance = 1.0e-12)
    {
        return std::abs(left - right) <= tolerance;
    }
}

int main()
{
    using namespace ModelTrajPlan;

    const Triangle triangle({ 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 });
    if(!nearlyEqual(triangle.normal.z(), 1.0)) {
        std::cerr << "Triangle normal calculation failed.\n";
        return 1;
    }

    const RRTNode node({ 2.0, -1.0, 3.0 }, { 0.0, 0.0, 0.0 }, 17);
    const Eigen::Matrix4d nodePose = node.toMatrix4d();
    if(!nearlyEqual(nodePose(0, 3), 2.0) || !nearlyEqual(nodePose(1, 3), -1.0) ||
        !nearlyEqual(nodePose(2, 3), 3.0) || !nearlyEqual(nodePose(0, 0), 1.0)) {
        std::cerr << "RRT node pose conversion failed.\n";
        return 2;
    }

    Eigen::Matrix4d original = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d nearCandidate = Eigen::Matrix4d::Identity();
    nearCandidate(0, 3) = 0.1;
    Eigen::Matrix4d farCandidate = Eigen::Matrix4d::Identity();
    farCandidate(0, 3) = 10.0;
    Eigen::Matrix4d selected;
    if(!OptiAlgorithm::selectContinuousSolution({ farCandidate, nearCandidate }, original, nullptr, selected) ||
        !nearlyEqual(selected(0, 3), 0.1)) {
        std::cerr << "Continuous pose selection failed.\n";
        return 3;
    }
    return 0;
}
