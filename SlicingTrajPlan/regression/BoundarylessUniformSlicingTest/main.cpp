#include <SlicingTrajPlan/UniformSlicer.h>

#include <cmath>
#include <iostream>

namespace
{
    ModelTrajPlan::ModelData makeClosedBox()
    {
        ModelTrajPlan::ModelData model;
        auto& mesh = model.mainModelData_;
        mesh.m_vertices = {
            -1.0F, -1.0F, 0.0F,  1.0F, -1.0F, 0.0F,  1.0F, 1.0F, 0.0F, -1.0F, 1.0F, 0.0F,
            -1.0F, -1.0F, 6.0F,  1.0F, -1.0F, 6.0F,  1.0F, 1.0F, 6.0F, -1.0F, 1.0F, 6.0F };
        mesh.m_indices = {
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
            0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5,
            2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 };
        return model;
    }
}

int main()
{
    const ModelTrajPlan::ModelData model = makeClosedBox();
    ModelTrajPlan::UniformSlicer slicer;
    ModelTrajPlan::SlicingParams params;
    params.mode = ModelTrajPlan::SlicingMode::AA;
    params.minStep = 1.0;
    params.maxSliceCount = 3;
    ModelTrajPlan::SlicingLocation locations;
    ModelTrajPlan::Epiphyseal unusedSkeleton;
    ModelTrajPlan::SliceResult result;
    std::vector<ModelTrajPlan::CuttingPlane> planes;
    if(!slicer.performSlicing(model, {}, {}, params, locations, unusedSkeleton, result, &planes)) {
        std::cerr << "Closed-mesh boundaryless slicing failed.\n";
        return 1;
    }
    if(!result.isBoundarylessFallback || locations.size() != 3 || planes.size() != 3 ||
        result.internalBoundaries.size() != 3) {
        std::cerr << "Boundaryless slicing did not honor the requested slice count.\n";
        return 2;
    }
    for(const auto& location : locations) {
        if(std::abs(location.normal.z() - 1.0) > 1.0e-12 ||
            location.point.z() <= 0.0 || location.point.z() >= 6.0) {
            std::cerr << "Boundaryless slice is not an internal horizontal plane.\n";
            return 3;
        }
    }

    // The same limit must apply when Parallel slicing has two detected
    // boundaries.  The mesh height and 1 mm spacing would normally yield five
    // internal planes; the result must contain only the requested three.
    ModelTrajPlan::BoundaryLine lowerBoundary;
    lowerBoundary.points.push_back({ Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, 0U });
    ModelTrajPlan::BoundaryLine upperBoundary;
    upperBoundary.points.push_back({ Eigen::Vector3d(0.0, 0.0, 6.0), 0.0, 0U });
    params.mode = ModelTrajPlan::SlicingMode::Parallel;
    locations.clear();
    result = {};
    planes.clear();
    if(!slicer.performSlicing(model, upperBoundary, lowerBoundary, params,
        locations, unusedSkeleton, result, &planes)) {
        std::cerr << "Parallel slicing with boundaries failed.\n";
        return 4;
    }
    if(locations.size() != 3 || planes.size() != 3 ||
        std::abs(locations.front().point.z() - 1.5) > 1.0e-12 ||
        std::abs(locations.back().point.z() - 4.5) > 1.0e-12) {
        std::cerr << "Parallel slicing did not honor the requested slice count.\n";
        return 5;
    }
    return 0;
}
