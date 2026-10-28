#pragma once
#include <SlicingTrajPlan/BoundaryDetector.h>
#include <array>
namespace ModelTrajPlan {
// Geometry is workpiece-local, in metres. Display extent never limits collision extent.
struct VirtualBaffle {
    bool enabled=false;
    double thickness=0.001, rmsResidual=0, maxResidual=0, correspondenceRms=0;
    Eigen::Vector3d center=Eigen::Vector3d::Zero(), normal=Eigen::Vector3d::UnitZ();
    Eigen::Vector3d axisX=Eigen::Vector3d::UnitX(), axisY=Eigen::Vector3d::UnitY();
    std::vector<std::vector<Eigen::Vector2d>> contours;
    std::string diagnostic;
    BoundaryTopologyDiagnostics topology;
    using Triangle=std::array<Eigen::Vector3d,3>;
    // All closed contours use even/odd fill, supporting disconnected sections and nested holes.
    bool apertureContains(const Eigen::Vector2d& p) const;
    bool intersectsTriangle(const Triangle& t, double clearance=0) const;
    std::vector<Triangle> displayTriangles(double extent) const;
};
VirtualBaffle buildVirtualBaffle(const ModelData& modelInMetres,
    double thickness=0.001, double maxFitResidual=0.005);
}
