#pragma once

#include <SlicingTrajPlan/BurTrajPlan.h>
#include <SlicingTrajPlan/TrajectoryGenerator.h>
#include <algorithm>
#include <stdexcept>

namespace coplanar_checks
{
    using namespace ModelTrajPlan;

    inline void require(bool ok, const char* message)
    {
        if (!ok) throw std::runtime_error(message);
    }

    inline ModelData cylinder()
    {
        ModelData model;
        auto& mesh = model.mainModelData_;
        constexpr unsigned count = 64;
        for (int layer = 0; layer < 3; ++layer)
            for (unsigned i = 0; i < count; ++i) {
                const double angle = 2.0 * 3.14159265358979323846 * i / count;
                mesh.m_vertices.insert(mesh.m_vertices.end(), {
                    static_cast<float>(300.0 + 300.0 * std::cos(angle)),
                    static_cast<float>(300.0 + 300.0 * std::sin(angle)),
                    static_cast<float>(layer * 500.0)});
            }
        for (unsigned layer = 0; layer < 2; ++layer)
            for (unsigned i = 0; i < count; ++i) {
                const unsigned a = layer * count + i, b = layer * count + (i + 1) % count;
                mesh.m_indices.insert(mesh.m_indices.end(), {a, b, b + count, a, b + count, a + count});
            }
        return model;
    }

    inline Eigen::Vector3d span(const BoundaryLine& line)
    {
        require(line.points.size() > 20, "A full circular contour was reduced to a fragment.");
        require((line.points.front().position - line.points.back().position).norm() < 1e-6,
            "Contour is not explicitly closed.");
        Eigen::Vector3d low = line.points.front().position, high = low;
        for (std::size_t i = 0; i < line.points.size(); ++i) {
            low = low.cwiseMin(line.points[i].position);
            high = high.cwiseMax(line.points[i].position);
            if (i > 0) require((line.points[i].position - line.points[i - 1].position).norm() > 1e-6,
                "Adjacent contour records contain the same point.");
        }
        return high - low;
    }

    inline void checkModel(const ModelData& model)
    {
        UniformSlicer slicer;
        SlicingParams params;
        params.maxSliceCount = 1;
        params.minStep = 10.0;
        SlicingLocation locations;
        Epiphyseal skeleton;
        SliceResult result;
        std::vector<CuttingPlane> planes;
        require(slicer.performSlicing(model, {}, {}, params, locations, skeleton, result, &planes),
            "Initial slicing at the mesh ring failed.");
        require(planes.size() == 1 && std::abs(planes[0].center.z() - 500.0) < 1e-6,
            "Expected the initial plane at Z=500.");
        const Eigen::Vector3d extent = span(result.internalBoundaries.front());
        require(extent.x() > 599.0 && extent.y() > 599.0, "Initial contour does not span the full cylinder.");
        require((planes[0].vertices[2] - planes[0].vertices[0]).norm() > 900.0,
            "Initial slice surface collapsed to a narrow strip.");
        std::cout << "Z=500 contour: " << result.internalBoundaries.front().points.size()
                  << " points, span " << extent.transpose() << '\n';

        GeneralData document;
        document.modelDataManager_.mainModelData_ = model.mainModelData_;
        document.processingState_.isSTLProcessed = true;
        document.slicingData_.cuttingPlanes_ = planes;
        document.slicingData_.currentSliceResult_ = result;
        BurTrajPlan plan;
        for (double shift : {-10.0, 0.0, 1e-7, 0.0}) {
            // Simulate trajectory sampling having populated the contour cache.
            document.slicingData_.sliceLines_ = document.slicingData_.currentSliceResult_.internalBoundaries;
            document.slicingData_.cuttingPlanes_[0].translation_z = shift;
            require(plan.updateSingleCuttingPlaneSize(document, 0), "Slice adjustment failed.");
            require(document.slicingData_.sliceLines_.empty(), "Stale trajectory contour cache survived adjustment.");
            const auto& contour = document.slicingData_.currentSliceResult_.internalBoundaries.front();
            const auto movedSpan = span(contour);
            require(movedSpan.x() > 599.0 && movedSpan.y() > 599.0, "Adjusted contour is incomplete.");
            for (const auto& point : contour.points)
                require(std::abs(point.position.z() - (500.0 + shift)) < 2e-6,
                    "Moving back to zero retained the previous contour height.");
        }
        // An oblique plane must still close, and resetting rotation must restore Z=500.
        document.slicingData_.cuttingPlanes_[0].rotation_x = 12.0;
        require(plan.updateSingleCuttingPlaneSize(document, 0), "Oblique slice failed.");
        span(document.slicingData_.currentSliceResult_.internalBoundaries.front());
        document.slicingData_.cuttingPlanes_[0].rotation_x = 0.0;
        require(plan.updateSingleCuttingPlaneSize(document, 0), "Rotation reset failed.");
        for (const auto& point : document.slicingData_.currentSliceResult_.internalBoundaries.front().points)
            require(std::abs(point.position.z() - 500.0) < 1e-6, "Rotation reset left a tilted contour.");

        // Check the user-visible outcome: end rings and middle contour must
        // produce columns along the wall, not converge to one tiny segment.
        std::vector<CuttingPlane> endPlanes(2, planes.front());
        endPlanes[0].center.z() = 0.0;
        endPlanes[1].center.z() = 1000.0;
        SliceResult ends;
        require(UniformSlicer::convertToCuttingPlanes(ends, &endPlanes, &model), "End ring slicing failed.");
        result.isBoundarylessFallback = false;
        result.usedParams.numTrajectories = 32;
        result.usedParams.trajectorySpacing = 30.0;
        TrajectoryGenerator generator;
        TrajectoryGenerationParams generation;
        generation.z1Offset = generation.z2Offset = generation.offsetDistance = 0.0;
        BoundaryData boundaries;
        LinePoint seamStart{}, seamEnd{};
        seamStart.position = Eigen::Vector3d(600.0, 300.0, 0.0);
        seamEnd.position = Eigen::Vector3d(600.0, 300.0, 1000.0);
        boundaries.phaseReferenceLine_ = {seamStart, seamEnd};
        boundaries.phaseReferencePlaneNormal_ = Eigen::Vector3d::UnitY();
        SlicingData slicing;
        TrajData trajectories;
        TrajectoryGroupInfo groups;
        std::vector<TrajectoryPoint> output;
        require(generator.generateMATLABTrajectory(ends.internalBoundaries[1], ends.internalBoundaries[0],
            result, generation, &model, output, groups, boundaries, slicing, trajectories),
            "Trajectory generation from the repaired contours failed.");
        Eigen::Vector3d low = Eigen::Vector3d::Constant(1e9), high = -low;
        for (const auto& column : trajectories.sampledBoundaryPoints_) {
            require(column.size() == 3, "Trajectory column lost a slicing layer.");
            low = low.cwiseMin(column[1]); high = high.cwiseMax(column[1]);
            for (std::size_t i = 1; i < column.size(); ++i)
                require((column[i].head<2>() - column[i - 1].head<2>()).norm() < 40.0,
                    "Trajectory columns twist across the cylinder instead of following its wall.");
        }
        require((high - low).x() > 590.0 && (high - low).y() > 590.0,
            "Middle trajectory samples collapsed into a small contour fragment.");
        std::cout << "Trajectory check: " << groups.numTrajectories << " columns remain on the cylinder wall.\n";
    }

    inline void checkCoplanarFace(const ModelData& box)
    {
        SliceResult result;
        std::vector<CuttingPlane> planes(1);
        planes[0].center = Eigen::Vector3d(0, 0, 0);
        planes[0].normal = Eigen::Vector3d::UnitZ();
        require(UniformSlicer::convertToCuttingPlanes(result, &planes, &box), "Coplanar box face failed.");
        require(result.internalBoundaries.front().points.size() == 5,
            "Coplanar face retained an internal diagonal or lost a boundary edge.");
        planes[0].translation_z = 10.0;
        require(!UniformSlicer::convertToCuttingPlanes(result, &planes, &box) && result.internalBoundaries.empty(),
            "An out-of-model plane retained a stale contour.");
    }
}
