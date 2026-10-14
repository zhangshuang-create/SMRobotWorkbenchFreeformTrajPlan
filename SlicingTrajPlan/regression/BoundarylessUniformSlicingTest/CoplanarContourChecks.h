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
        planes.resize(1);
        planes[0].center = Eigen::Vector3d(0, 0, 500);
        require(UniformSlicer::convertToCuttingPlanes(result, &planes, &model),
            "Initial slicing at the mesh ring failed.");
        const auto& contourPoints = result.internalBoundaries.front().points;
        Eigen::Vector3d low = contourPoints.front().position, high = low;
        for (const auto& point : contourPoints) {
            low = low.cwiseMin(point.position); high = high.cwiseMax(point.position);
        }
        planes[0].vertices = {low, Eigen::Vector3d(high.x(), low.y(), 500),
            high, Eigen::Vector3d(low.x(), high.y(), 500)};
        const Eigen::Vector3d extent = span(result.internalBoundaries.front());
        require(extent.x() > 599.0 && extent.y() > 599.0, "Initial contour does not span the full cylinder.");
        require((planes[0].vertices[2] - planes[0].vertices[0]).norm() > 800.0,
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
        low = Eigen::Vector3d::Constant(1e9); high = -low;
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
        // Boundary labels are not a distance-to-origin ordering. Reverse them
        // so the nearest seam endpoint belongs to the supplied upper boundary.
        require(generator.generateMATLABTrajectory(ends.internalBoundaries[0], ends.internalBoundaries[1],
            result, generation, &model, output, groups, boundaries, slicing, trajectories),
            "Generation with the nearest endpoint on the upper boundary failed.");
        const auto& firstColumn = trajectories.sampledBoundaryPoints_.front();
        require(firstColumn.back().squaredNorm() < firstColumn.front().squaredNorm(),
            "Regression setup did not reverse boundary distances.");
        require((output.front().pose.block<3,1>(0,3) - firstColumn.back()).norm() < 1e-6,
            "The first seam trajectory started at the farther boundary.");
        require((output[firstColumn.size()].pose.block<3,1>(0,3) -
            trajectories.sampledBoundaryPoints_[1].front()).norm() < 1e-6,
            "Changing the initial boundary broke serpentine traversal.");
        generation.z2Offset = 7.0;
        require(generator.generateMATLABTrajectory(ends.internalBoundaries[0], ends.internalBoundaries[1],
            result, generation, &model, output, groups, boundaries, slicing, trajectories),
            "Upper-boundary empty-run generation failed.");
        const Eigen::Matrix4d firstPose = output.front().pose;
        generation.previewFirstPoint = true;
        require(generator.generateMATLABTrajectory(ends.internalBoundaries[0], ends.internalBoundaries[1],
            result, generation, &model, output, groups, boundaries, slicing, trajectories),
            "Reversed first-point preview failed.");
        require(output.size() == 1 && (output.front().pose - firstPose).norm() < 1e-6,
            "First-point preview disagrees with the exported initial pose.");
    }


    inline void checkFallbackSampling(const ModelData& model)
    {
        SliceResult result;
        result.isBoundarylessFallback = true;
        result.usedParams.numTrajectories = 5;
        for (int layer = 0; layer < 3; ++layer) {
            BoundaryLine line{};
            line.normal = Eigen::Vector3d::UnitZ();
            line.z_level = layer;
            for (double x : {-2.0, 0.0, 2.0})
                line.points.push_back({Eigen::Vector3d(x, 0, layer), 0, 0});
            if (layer == 1) std::reverse(line.points.begin(), line.points.end());
            result.internalBoundaries.push_back(line);
        }
        TrajectoryGenerator generator;
        TrajectoryGenerationParams generation;
        generation.z1Offset = generation.z2Offset = generation.offsetDistance = 0.0;
        BoundaryData boundaries;
        SlicingData slicing;
        TrajData data;
        TrajectoryGroupInfo groups;
        std::vector<TrajectoryPoint> output;
        require(generator.generateMATLABTrajectory({}, {}, result, generation, &model,
            output, groups, boundaries, slicing, data), "Open fallback generation failed.");
        require(groups.numTrajectories == 5, "Requested fallback count was changed.");
        for (int c = 0; c < 5; ++c) {
            require(data.sampledBoundaryPoints_[c].size() == 3, "Fallback lost a layer.");
            for (const auto& p : data.sampledBoundaryPoints_[c])
                require(std::abs(p.x() - (c - 2)) < 1e-9, "Open contour was closed or reversed across layers.");
        }
        for (int count : {2, 4, 30, 100}) {
            result.usedParams.numTrajectories = count;
            slicing.sliceLines_.clear();
            require(generator.generateMATLABTrajectory({}, {}, result, generation, &model,
                output, groups, boundaries, slicing, data), "Even open-contour sampling failed.");
            require(groups.numTrajectories == count, "Even trajectory count was changed.");
            for (int c = 0; c < count; ++c) {
                const double expected = -2.0 + 4.0 * c / (count - 1);
                for (const auto& p : data.sampledBoundaryPoints_[c])
                    require(std::abs(p.x() - expected) < 1e-9,
                        "Even sampling lost equal arc lengths or direction continuity.");
            }
            for (size_t layer = 0; layer < 3; ++layer) {
                const auto& left = data.sampledBoundaryPoints_[count / 2 - 1][layer];
                const auto& right = data.sampledBoundaryPoints_[count / 2][layer];
                require(std::abs((left.x() + right.x()) / 2.0) < 1e-9,
                    "Middle pair does not straddle the arc-length midpoint.");
            }
        }
        result.usedParams.numTrajectories = 1;
        slicing.sliceLines_.clear();
        require(!generator.generateMATLABTrajectory({}, {}, result, generation, &model,
            output, groups, boundaries, slicing, data), "A single fallback column was accepted.");
        require(data.sampledBoundaryPoints_.empty(), "Failed generation retained stale samples.");

        result.usedParams.numTrajectories = 0;
        result.usedParams.trajectorySpacing = 1.5;
        // Vary contour length while preserving a common column count.
        for (auto& p : result.internalBoundaries[1].points) p.position.x() *= 0.5;
        slicing.sliceLines_.clear();
        require(generator.generateMATLABTrajectory({}, {}, result, generation, &model,
            output, groups, boundaries, slicing, data) && groups.numTrajectories == 4,
            "Open auto count did not use longest arc length and include both ends.");
        for (const auto& column : data.sampledBoundaryPoints_)
            require(column.size() == 3, "Automatic sampling produced unequal layer counts.");
        result.usedParams.numTrajectories = 6;
        result.usedParams.trajectorySpacing = 0.0;
        slicing.sliceLines_.clear();
        require(generator.generateMATLABTrajectory({}, {}, result, generation, &model,
            output, groups, boundaries, slicing, data) && groups.numTrajectories == 6,
            "Explicit trajectory count did not override contour spacing.");

        result.internalBoundaries.clear();
        result.usedParams.numTrajectories = 4;
        for (int layer = 0; layer < 3; ++layer) {
            BoundaryLine line{};
            line.normal = Eigen::Vector3d::UnitZ();
            line.z_level = layer;
            const std::vector<Eigen::Vector3d> ring = {
                {-1, -1, double(layer)}, {1, -1, double(layer)},
                {1, 1, double(layer)}, {-1, 1, double(layer)}};
            for (int i = 0; i <= 4; ++i) {
                const int index = layer == 1 ? (5 - i) % 4 : i % 4;
                line.points.push_back({ring[index], 0, 0});
            }
            result.internalBoundaries.push_back(line);
        }
        slicing.sliceLines_.clear();
        require(generator.generateMATLABTrajectory({}, {}, result, generation, &model,
            output, groups, boundaries, slicing, data), "Closed fallback generation failed.");
        require(groups.numTrajectories == 4, "Closed ring count was changed.");
        for (const auto& column : data.sampledBoundaryPoints_)
            for (size_t i = 1; i < column.size(); ++i)
                require((column[i].head<2>() - column[i - 1].head<2>()).norm() < 1e-9,
                    "Closed contour direction or cyclic phase changed across layers.");
        result.usedParams.numTrajectories = 0;
        result.usedParams.trajectorySpacing = 2.0;
        slicing.sliceLines_.clear();
        require(generator.generateMATLABTrajectory({}, {}, result, generation, &model,
            output, groups, boundaries, slicing, data) && groups.numTrajectories == 4,
            "Closed fallback count was not derived from contour spacing.");
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
