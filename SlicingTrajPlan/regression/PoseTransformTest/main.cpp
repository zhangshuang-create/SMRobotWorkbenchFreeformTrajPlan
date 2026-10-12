#include <SlicingTrajPlan/PoseTransform.h>

#include <cmath>
#include <iostream>

namespace
{
    bool nearlyEqual(double left, double right, double tolerance = 1.0e-9)
    {
        return std::abs(left - right) <= tolerance;
    }
}

int main()
{
    using namespace ModelTrajPlan;

    PoseMeshData source;
    source.vertices = { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } };
    PoseMeshData target;
    target.vertices = { { 2.0, -3.0, 4.0 }, { 3.0, -3.0, 4.0 }, { 2.0, -2.0, 4.0 } };

    PoseTransform transform;
    const Eigen::Matrix4d registration = transform.computeRigidTransformation(source, target);
    const PoseMeshData transformedMesh = transform.transformMesh(source, registration);
    if(transformedMesh.vertices.size() != target.vertices.size() ||
        !nearlyEqual(transformedMesh.vertices[2].x, 2.0) ||
        !nearlyEqual(transformedMesh.vertices[2].y, -2.0) ||
        !nearlyEqual(transformedMesh.vertices[2].z, 4.0)) {
        std::cerr << "Rigid mesh registration failed.\n";
        return 1;
    }

    PoseTransformTrajectory trajectory("pose-transform-regression");
    trajectory.addPoint(Eigen::Matrix4d::Identity(), 1.25);
    const PoseTransformTrajectory transformedTrajectory = transform.transformTrajectory(trajectory, registration);
    if(transformedTrajectory.size() != 1 || !nearlyEqual(transformedTrajectory.timestamps[0], 1.25) ||
        !nearlyEqual(transformedTrajectory.poses[0](0, 3), 2.0) ||
        !nearlyEqual(transformedTrajectory.poses[0](1, 3), -3.0) ||
        !nearlyEqual(transformedTrajectory.poses[0](2, 3), 4.0)) {
        std::cerr << "Trajectory pose transformation failed.\n";
        return 2;
    }
    return 0;
}
