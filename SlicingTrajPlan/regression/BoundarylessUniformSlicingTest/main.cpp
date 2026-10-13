#include <SlicingTrajPlan/UniformSlicer.h>

#include <cmath>
#include <iostream>
#include "CoplanarContourChecks.h"

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

int main(int argc, char** argv)
{
    try {
    const ModelTrajPlan::ModelData model = makeClosedBox();
    ModelTrajPlan::UniformSlicer slicer;
    ModelTrajPlan::SlicingParams params;
    params.mode = ModelTrajPlan::SlicingMode::AA;
    params.minStep = 2.0;
    params.maxSliceCount = 0;
    params.trajectorySpacing = 2.0;
    ModelTrajPlan::SlicingLocation locations;
    ModelTrajPlan::Epiphyseal unusedSkeleton;
    ModelTrajPlan::SliceResult result;
    std::vector<ModelTrajPlan::CuttingPlane> planes;
    if(!slicer.performSlicing(model, {}, {}, params, locations, unusedSkeleton, result, &planes)) {
        std::cerr << "Closed-mesh boundaryless slicing failed.\n";
        return 1;
    }
    if(!result.isBoundarylessFallback || locations.size() != 4 || planes.size() != 4 ||
        result.internalBoundaries.size() != 4) {
        std::cerr << "Boundaryless slicing did not honor the requested Z pitch.\n";
        return 2;
    }
    for(const auto& location : locations) {
        if(std::abs(location.normal.z() - 1.0) > 1.0e-12 ||
            location.point.z() < 0.0 || location.point.z() > 6.0) {
            std::cerr << "Boundaryless slice is not a horizontal display plane.\n";
            return 3;
        }
    }

    coplanar_checks::require(locations.front().point.z() == 0.0 && locations.back().point.z() == 6.0,
        "Display planes lost model extrema.");
    params.minStep = 2.5;
    params.trajectorySpacing = 100.0;
    params.numTrajectories = 7;
    coplanar_checks::require(slicer.performSlicing(model, {}, {}, params, locations, unusedSkeleton, result, &planes),
        "Non-divisible Z pitch failed.");
    coplanar_checks::require(locations.size() == 4 && locations[2].point.z() == 5.0 && locations[3].point.z() == 6.0,
        "Final partial interval was redistributed.");
    coplanar_checks::require(result.internalBoundaries.front().points.front().position.z() > 0.0 &&
        result.internalBoundaries.back().points.front().position.z() < 6.0,
        "Extreme display layers were not queried inside the mesh.");
    coplanar_checks::require(ModelTrajPlan::UniformSlicer::convertToCuttingPlanes(result, &planes, &model) &&
        result.internalBoundaries.front().points.front().position.z() > 0.0,
        "Rebuilding planes lost the inward extrema queries.");
    params.numTrajectories = 11;
    params.trajectorySpacing = 0.5;
    params.maxSliceCount = 0;
    coplanar_checks::require(slicer.performSlicing(model, {}, {}, params, locations, unusedSkeleton, result, &planes) &&
        locations.size() == 4 && locations[2].point.z() == 5.0,
        "Contour spacing or column count changed the Z pitch.");
    params.maxSliceCount = 3;
    params.minStep = 0.0; // Disabled spacing must not invalidate explicit count.
    coplanar_checks::require(slicer.performSlicing(model, {}, {}, params, locations, unusedSkeleton, result, &planes) &&
        locations.size() == 3 && locations[1].point.z() == 3.0,
        "Explicit slice count did not override minimum spacing.");
    params.maxSliceCount = 1;
    coplanar_checks::require(slicer.performSlicing(model, {}, {}, params, locations, unusedSkeleton, result, &planes) &&
        locations.size() == 1 && locations[0].point.z() == 3.0,
        "Single slice was not placed at the height midpoint.");
    params.maxSliceCount = 0;
    params.minStep = 2.5;
    auto openModel = model;
    openModel.mainModelData_.m_indices = {0, 1, 5, 0, 5, 4};
    ModelTrajPlan::BoundaryLine singleBoundary{};
    singleBoundary.points.push_back({Eigen::Vector3d(0, 0, 0), 0, 0});
    coplanar_checks::require(slicer.performSlicing(openModel, {}, singleBoundary, params,
        locations, unusedSkeleton, result, &planes) && result.isBoundarylessFallback,
        "A single boundary did not select fallback slicing.");
    for (const auto& line : result.internalBoundaries)
        coplanar_checks::require((line.points.front().position - line.points.back().position).norm() > 1.0,
            "Open STL contour was forced closed.");
    params.trajectorySpacing = 20.0;
    coplanar_checks::require(slicer.performSlicing(openModel, {}, {}, params,
        locations, unusedSkeleton, result, &planes), "Display-size regression slicing failed.");
    for (const auto& plane : planes) {
        coplanar_checks::require((plane.vertices[1] - plane.vertices[0]).norm() >= 20.0 - 1e-9 &&
            (plane.vertices[3] - plane.vertices[0]).norm() >= 20.0 - 1e-9,
            "Initial display plane collapsed below the requested spacing.");
    }
    const auto contourBefore = result.internalBoundaries[1].points;
    bool updated = false;
    coplanar_checks::require(ModelTrajPlan::UniformSlicer::updateSingleCuttingPlaneSize(
        planes[1], openModel, planes[1].center, updated, 20.0), "Display plane resize failed.");
    coplanar_checks::require((planes[1].vertices[1] - planes[1].vertices[0]).norm() >= 20.0 - 1e-9 &&
        (planes[1].vertices[3] - planes[1].vertices[0]).norm() >= 20.0 - 1e-9,
        "Adjusted display plane collapsed below the requested spacing.");
    for (size_t i = 0; i < contourBefore.size(); ++i)
        coplanar_checks::require((contourBefore[i].position - result.internalBoundaries[1].points[i].position).norm() == 0.0,
            "Display resizing changed the physical contour.");
    params.minStep = 1.0;
    params.maxSliceCount = 3;

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
    try {
        coplanar_checks::checkModel(coplanar_checks::cylinder());
        coplanar_checks::checkCoplanarFace(model);
        coplanar_checks::checkFallbackSampling(model);
        if (argc > 1) {
            ModelTrajPlan::ModelData input;
            coplanar_checks::require(input.loadFromFile(argv[1]), "Could not load STL regression input.");
            coplanar_checks::checkModel(input);
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 6;
    }
    return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 7;
    }
}
