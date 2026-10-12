#include <SlicingTrajPlan/TrajectoryGenerator.h>
#include <SlicingTrajPlan/GeneralData.h>
#include <SlicingTrajPlan/TrajectoryFrame.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <map>
#include <SlicingTrajPlan/BoundaryDetector.h>
#include <SlicingTrajPlan/STLProcessor.h>
#include <SlicingTrajPlan/UniformSlicer.h>

// If M_PI is not defined, use the standard value
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace ModelTrajPlan
{
    TrajectoryGenerator::TrajectoryGenerator()
    {
        // [DESIGN] Stateless algorithm class
    }

    TrajectoryGenerator::~TrajectoryGenerator()
    {
        // No cleanup needed - stateless class
    }

    // Helper: ensure contour winding is consistent with its own slice plane.
    // AA contours can be strongly tilted, so XY projected area is not a
    // reliable orientation test and can arbitrarily reverse near-vertical
    // contours from one layer to the next.
    void TrajectoryGenerator::normalizeBoundaryCCW(BoundaryLine& boundary)
    {
        if (boundary.points.size() < 3 || boundary.normal.norm() <= 1e-8) {
            return;
        }

        Eigen::Vector3d polygonNormal = Eigen::Vector3d::Zero();
        const size_t n = boundary.points.size();
        for (size_t i = 0; i < n; ++i) {
            const auto& p1 = boundary.points[i].position;
            const auto& p2 = boundary.points[(i + 1) % n].position;
            polygonNormal += p1.cross(p2);
        }

        // Reverse only when the contour's geometric normal opposes its own
        // cutting-plane normal. This keeps winding stable for tilted planes.
        if (polygonNormal.norm() > 1e-10 && polygonNormal.dot(boundary.normal) < 0.0) {
            std::reverse(boundary.points.begin(), boundary.points.end());
        }
    }


    // Helper function
    Eigen::Vector3d TrajectoryGenerator::getPointAtLength(
        const std::vector<BasicPoint>& points,
        const std::vector<double>& cumLengths,
        double totalLength,
        double targetLen)
    {
        // Handle closed loop (wrap around)
        targetLen = std::fmod(targetLen, totalLength);
        if (targetLen < 0) targetLen += totalLength;

        // Iterate and find segment
        for (size_t i = 0; i < cumLengths.size(); ++i) {
            double nextLength = (i + 1 < cumLengths.size()) ? cumLengths[i + 1] : totalLength;

            if (targetLen >= cumLengths[i] && targetLen <= nextLength) {
                double segmentLen = nextLength - cumLengths[i];

                // Prevent division by zero
                double t = (segmentLen > 1e-9) ? (targetLen - cumLengths[i]) / segmentLen : 0.0;

                // Get positions (points is BasicPoint, so use .position)
                const Eigen::Vector3d& p1 = points[i].position;
                // Handle closure: the next point after last is the first
                const Eigen::Vector3d& p2 = (i + 1 < points.size()) ? points[i + 1].position : points[0].position;

                // Linear interpolation
                return p1 + t * (p2 - p1);
            }
        }

        // Fallback: return first point on failure
        if (!points.empty()) {
            return points[0].position;
        }
        return Eigen::Vector3d::Zero();
    }


    // [Core Addition] Main MATLAB trajectory generation method - [Refactor] parameterized interface
    // [Refactor] Main trajectory generation function - explicit input/output, stateless design
    bool TrajectoryGenerator::generateMATLABTrajectory(const BoundaryLine& upperBoundary,
        const BoundaryLine& lowerBoundary,
        const SliceResult& sliceResult,
        const TrajectoryGenerationParams& trajParams,
        const ModelData* modelData,
        std::vector<TrajectoryPoint>& trajectoryPoints,
        TrajectoryGroupInfo& groupInfo,
        BoundaryData& boundaryData,
        SlicingData& slicingData,
        TrajData& trajData
    )
    {
        try {
            // Clear output data
            trajectoryPoints.clear();
            trajData.trajectoryPoints_.clear();

            // 1. [Refactor] Generate sampled points and store in trajData.sampledBoundaryPoints_
            if (!generateReferenceLinesFromSliceResult(upperBoundary, lowerBoundary, sliceResult, trajParams, boundaryData, slicingData, trajData)) {
                return false;
            }

            // 2. [Refactor] Generate trajectory points directly from sampled points, skipping referenceLines intermediate step
            std::vector<Eigen::Vector3d> cuttingPlaneCenters;
            cuttingPlaneCenters.reserve(boundaryData.boundaryLines_.size());
            for (const auto& boundary : boundaryData.boundaryLines_) {
                // Derive the center from the actual contour points. Some input
                // boundary records do not initialize their cached center.
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                if (!boundary.points.empty()) {
                    center.setZero();
                    for (const auto& boundaryPoint : boundary.points) {
                        center += boundaryPoint.position;
                    }
                    center /= static_cast<double>(boundary.points.size());
                } else if (boundary.center.allFinite()) {
                    center = boundary.center;
                }
                cuttingPlaneCenters.push_back(center);
            }

            if (!generateTrajectoryPointsFromSampledPoints(
                    trajData.sampledBoundaryPoints_, cuttingPlaneCenters,
                    modelData, trajParams, trajData.trajectoryPoints_)) {
                return false;
            }

            if (trajData.trajectoryPoints_.empty()) {
                return false;
            }

            // Sync to output parameter (keep interface compatibility)
            trajectoryPoints = trajData.trajectoryPoints_;

            // 3. Output trajectory group info from the actual sampled columns.
            // A requested count of zero means "derive it from spacing", so it
            // must never be used as a divisor after sampling.
            groupInfo.numTrajectories = static_cast<int>(trajData.sampledBoundaryPoints_.size());
            if(groupInfo.numTrajectories <= 0) {
                trajectoryPoints.clear();
                trajData.trajectoryPoints_.clear();
                return false;
            }
            groupInfo.pointsPerTrajectory = !trajData.trajectoryPoints_.empty() ?
                static_cast<int>(trajData.trajectoryPoints_.size() / groupInfo.numTrajectories) : 0;
            groupInfo.isConsistent = true;

            return true;
        }
        catch (const std::exception& e) {
            trajectoryPoints.clear();
            return false;
        }
    }


    // [Refactor Addition] Generate reference lines from slicing result - new interface auto-copies boundaries based on offset params
    // [Addition] Also place sliceLines_ into boundaryLines_ in order
    bool TrajectoryGenerator::generateReferenceLinesFromSliceResult(const BoundaryLine& upperBoundary,
        const BoundaryLine& lowerBoundary,
        const SliceResult& sliceResult,
        const TrajectoryGenerationParams& trajParams,
        BoundaryData& boundaryData,
        SlicingData& slicingData,
        TrajData& trajData)
    {

        try {
            const bool boundarylessFallback = sliceResult.isBoundarylessFallback;
            // [Addition] Place sliceLines_ into boundaryLines_ in order
            // The boundary offsets are empty-run extensions, not independent
            // Z-shifted contours.  Sample the physical contours first, then
            // derive each extension from its adjacent slice trajectory below.
            // Order here is always lower boundary, slices, upper boundary.
            boundaryData.boundaryLines_.clear();

            // Discard legacy offset contours. They were produced by shifting
            // Z and therefore cannot represent the local empty-run direction.
            boundaryData.lowerBoundaryOffset_.points.clear();
            boundaryData.upperBoundaryOffset_.points.clear();

            // 1. Add lower boundary
            if (!boundarylessFallback && !lowerBoundary.points.empty()) {
                boundaryData.boundaryLines_.push_back(lowerBoundary);
            }

            // 2. Add sliceLines_ (bottom to top)
            // Prefer slicingData.sliceLines_; if empty, copy from sliceResult.internalBoundaries
            if (slicingData.sliceLines_.empty() && !sliceResult.internalBoundaries.empty()) {
                // If sliceLines_ is empty, copy from internalBoundaries
                slicingData.sliceLines_ = sliceResult.internalBoundaries;
            }

            // Use sliceLines_ (already ensured non-empty or copied from internalBoundaries)
            if (!slicingData.sliceLines_.empty()) {
                std::vector<BoundaryLine> orderedSliceLines = slicingData.sliceLines_;

                if (sliceResult.usedParams.mode == SlicingMode::Parallel || boundarylessFallback) {
                    // Parallel planes all share the global Z direction, so Z
                    // is a valid bottom-to-top ordering key.
                    std::sort(orderedSliceLines.begin(), orderedSliceLines.end(),
                        [](const BoundaryLine& a, const BoundaryLine& b) {
                            return a.z_level < b.z_level;
                        });
                } else if (orderedSliceLines.size() > 1) {
                    // AA planes are generated in adaptive-node order. Their
                    // z_level is normal.dot(center), not a height, and sorting
                    // by it reorders tilted planes and creates top-to-bottom
                    // jumps in the bow trajectory. Keep that generation order,
                    // reversing it only when its endpoints are closer to the
                    // opposite model boundaries.
                    const auto contourCenter = [](const BoundaryLine& line) -> Eigen::Vector3d {
                        if (line.points.empty()) return line.center;
                        Eigen::Vector3d center = Eigen::Vector3d::Zero();
                        for (const auto& point : line.points) center += point.position;
                        return Eigen::Vector3d(center / static_cast<double>(line.points.size()));
                    };
                    const Eigen::Vector3d lowerCenter = contourCenter(lowerBoundary);
                    const Eigen::Vector3d upperCenter = contourCenter(upperBoundary);
                    const Eigen::Vector3d firstCenter = contourCenter(orderedSliceLines.front());
                    const Eigen::Vector3d lastCenter = contourCenter(orderedSliceLines.back());
                    const double forwardCost = (firstCenter - lowerCenter).norm() +
                        (lastCenter - upperCenter).norm();
                    const double reverseCost = (lastCenter - lowerCenter).norm() +
                        (firstCenter - upperCenter).norm();
                    if (reverseCost < forwardCost) {
                        std::reverse(orderedSliceLines.begin(), orderedSliceLines.end());
                    }
                }

                // Append to boundaryLines_
                for (const auto& sliceLine : orderedSliceLines) {
                    boundaryData.boundaryLines_.push_back(sliceLine);
                }
            }

            // 3. Add upper boundary
            if (!boundarylessFallback && !upperBoundary.points.empty()) {
                boundaryData.boundaryLines_.push_back(upperBoundary);
            }

            // [Addition] Normalize all boundaries to CCW to avoid inconsistent sampling directions
            for (auto& bl : boundaryData.boundaryLines_) {
                normalizeBoundaryCCW(bl);
            }

            // 4. Derive the common contour sample count from spacing d.
            // Use the longest and shortest polyline lengths in the sliced contours.
            const double spacing = sliceResult.usedParams.trajectorySpacing;
            if (!std::isfinite(spacing) || spacing <= 0.0) {
                std::cout << "TrajectoryGenerator: Error - invalid trajectory spacing d: " << spacing << std::endl;
                return false;
            }
            double longestLength = 0.0;
            double shortestLength = std::numeric_limits<double>::max();
            for (const auto& contour : boundaryData.boundaryLines_) {
                double length = 0.0;
                for (size_t i = 1; i < contour.points.size(); ++i) {
                    length += (contour.points[i].position - contour.points[i - 1].position).norm();
                }
                if (length > 0.0) {
                    longestLength = std::max(longestLength, length);
                    shortestLength = std::min(shortestLength, length);
                }
            }
            if (longestLength <= 0.0 || shortestLength == std::numeric_limits<double>::max()) {
                std::cout << "TrajectoryGenerator: Error - no valid contour polylines" << std::endl;
                return false;
            }
            int trajectoryCount = sliceResult.usedParams.numTrajectories;
            if (trajectoryCount <= 0) {
                const int longestCount = std::max(1, static_cast<int>(std::floor(longestLength / spacing)));
                const int shortestCount = std::max(1, static_cast<int>(std::floor(shortestLength / spacing)));
                trajectoryCount = std::max(2, (longestCount + shortestCount) / 2);
            }
            trajectoryCount = std::max(2, trajectoryCount);
            if (trajectoryCount % 2 != 0) {
                ++trajectoryCount;
            }


            // 5. Uniformly sample the physical contours first.
            trajData.sampledBoundaryPoints_ =
                samplePointsFromBoundaryLines(boundaryData.boundaryLines_, trajectoryCount,
                    boundarylessFallback ? std::vector<LinePoint>{} : boundaryData.phaseReferenceLine_,
                    boundarylessFallback ? Eigen::Vector3d::Zero() : boundaryData.phaseReferencePlaneNormal_);

            if (trajData.sampledBoundaryPoints_.empty()) {
                return false;
            }

            // 6. Add the two empty-run layers.  Each extension is computed
            // per trajectory column, so tilted/adaptive slices retain their
            // real local direction instead of being forced along global Z.
            const auto buildOffsetBoundary = [](const std::vector<Eigen::Vector3d>& points,
                const BoundaryLine& source, BoundaryLineType type) {
                BoundaryLine result;
                result.points.reserve(points.size());
                result.center = Eigen::Vector3d::Zero();
                for (size_t index = 0; index < points.size(); ++index) {
                    BasicPoint point;
                    point.position = points[index];
                    point.index = static_cast<int>(index);
                    result.points.push_back(point);
                    result.center += point.position;
                }
                if (!points.empty()) {
                    result.center /= static_cast<double>(points.size());
                }
                result.z_level = source.normal.dot(result.center);
                result.normal = source.normal;
                result.color = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
                result.Type = type;
                result.name = source.name + " empty-run offset";
                return result;
            };

            std::vector<Eigen::Vector3d> lowerOffsetPoints;
            std::vector<Eigen::Vector3d> upperOffsetPoints;
            lowerOffsetPoints.reserve(trajData.sampledBoundaryPoints_.size());
            upperOffsetPoints.reserve(trajData.sampledBoundaryPoints_.size());

            // UI semantics: z1 is the large/lower-end offset, z2 is the
            // small/upper-end offset.  Each is a translation away from the
            // adjacent physical slice toward its own boundary.
            const bool requiresEmptyRun = trajParams.z1Offset != 0.0 || trajParams.z2Offset != 0.0;
            const size_t minimumLayersForOffset = boundarylessFallback ? 2 : 3;
            for (auto& trajectory : trajData.sampledBoundaryPoints_) {
                if (requiresEmptyRun && trajectory.size() < minimumLayersForOffset) {
                    std::cerr << "Boundary empty-run requires a lower boundary, an adjacent slice, and an upper boundary." << std::endl;
                    return false;
                }
                if (!requiresEmptyRun) {
                    continue;
                }

                const Eigen::Vector3d& lowerBoundaryPoint = trajectory.front();
                const Eigen::Vector3d& lowerAdjacentSlicePoint = trajectory[1];
                const Eigen::Vector3d& upperBoundaryPoint = trajectory.back();
                const Eigen::Vector3d& upperAdjacentSlicePoint = trajectory[trajectory.size() - 2];

                Eigen::Vector3d lowerDirection = lowerBoundaryPoint - lowerAdjacentSlicePoint;
                Eigen::Vector3d upperDirection = upperBoundaryPoint - upperAdjacentSlicePoint;
                if (lowerDirection.norm() <= 1e-8 || upperDirection.norm() <= 1e-8) {
                    std::cerr << "Cannot derive empty-run direction from coincident boundary and slice points." << std::endl;
                    return false;
                }

                lowerOffsetPoints.push_back(lowerBoundaryPoint +
                    lowerDirection.normalized() * trajParams.z1Offset);
                upperOffsetPoints.push_back(upperBoundaryPoint +
                    upperDirection.normalized() * trajParams.z2Offset);
            }

            if (trajParams.z1Offset != 0.0) {
                if (!boundarylessFallback) {
                    boundaryData.lowerBoundaryOffset_ = buildOffsetBoundary(
                        lowerOffsetPoints, lowerBoundary, BoundaryLineType::LowerBoundaryOffset);
                    boundaryData.boundaryLines_.insert(boundaryData.boundaryLines_.begin(),
                        boundaryData.lowerBoundaryOffset_);
                }
                for (size_t index = 0; index < trajData.sampledBoundaryPoints_.size(); ++index) {
                    trajData.sampledBoundaryPoints_[index].insert(
                        trajData.sampledBoundaryPoints_[index].begin(), lowerOffsetPoints[index]);
                }
            }

            if (trajParams.z2Offset != 0.0) {
                if (!boundarylessFallback) {
                    boundaryData.upperBoundaryOffset_ = buildOffsetBoundary(
                        upperOffsetPoints, upperBoundary, BoundaryLineType::UpperBoundaryOffset);
                    boundaryData.boundaryLines_.push_back(boundaryData.upperBoundaryOffset_);
                }
                for (size_t index = 0; index < trajData.sampledBoundaryPoints_.size(); ++index) {
                    trajData.sampledBoundaryPoints_[index].push_back(upperOffsetPoints[index]);
                }
            }

            return true;  // Sampled points are stored in trajData.sampledBoundaryPoints_ and used directly later

        }
        catch (const std::exception& e) {
            std::cout << "TrajectoryGenerator: Failed to generate reference lines - " << e.what() << std::endl;
            return false;
        }
    }


    // [Addition] Generate trajectory points directly from sampled points (skip referenceLines intermediate step)
    // [Addition] Generate trajectory points directly from sampled points (skip referenceLines intermediate step)
    bool TrajectoryGenerator::generateTrajectoryPointsFromSampledPoints(
        const std::vector<std::vector<Eigen::Vector3d>>& sampledPoints,
        const std::vector<Eigen::Vector3d>& cuttingPlaneCenters,
        const ModelData* modelData,
        const TrajectoryGenerationParams& trajParams,
        std::vector<TrajectoryPoint>& trajectoryPoints) const
    {
        trajectoryPoints.clear();
        if (sampledPoints.empty()) {
            return false;
        }
        if (!modelData || STLProcessor::getTriangleCount(modelData->mainModelData_) == 0) {
            std::cerr << "Trajectory generation requires valid STL triangles to calculate surface normals." << std::endl;
            return false;
        }
        if (!validTrajectoryAxes(trajParams.normalAxis, trajParams.forwardAxis)) return false;

        // A slice center is a good sign reference for side walls, but it is
        // nearly tangent to the normal on caps (and can be ambiguous at
        // concave corners).  Keep the model center as a stable fallback.
        Eigen::Vector3d modelMin, modelMax;
        STLProcessor::calculateBounds(modelData->mainModelData_, modelMin, modelMax);
        const Eigen::Vector3d modelCenter = 0.5 * (modelMin + modelMax);

        const int numTrajectories = static_cast<int>(sampledPoints.size());

        // Derive centers from the sampled grid itself.  The grid can contain
        // lower/upper empty-run points which are not present in
        // boundaryLines_, so its layer indices must not be paired directly
        // with the older boundary-center list.
        size_t layerCount = 0;
        for (const auto& trajectory : sampledPoints) {
            layerCount = std::max(layerCount, trajectory.size());
        }
        std::vector<Eigen::Vector3d> layerCenters(layerCount, Eigen::Vector3d::Zero());
        std::vector<size_t> layerCenterCounts(layerCount, 0);
        for (const auto& trajectory : sampledPoints) {
            for (size_t layerIndex = 0; layerIndex < trajectory.size(); ++layerIndex) {
                if (trajectory[layerIndex].allFinite()) {
                    layerCenters[layerIndex] += trajectory[layerIndex];
                    ++layerCenterCounts[layerIndex];
                }
            }
        }
        for (size_t layerIndex = 0; layerIndex < layerCount; ++layerIndex) {
            if (layerCenterCounts[layerIndex] > 0) {
                layerCenters[layerIndex] /= static_cast<double>(layerCenterCounts[layerIndex]);
            } else if (layerIndex < cuttingPlaneCenters.size()) {
                layerCenters[layerIndex] = cuttingPlaneCenters[layerIndex];
            }
        }

        // First establish an absolute orientation for every sampled point.
        // The layer fit below is deliberately based on this pre-continuity
        // field: a bad first point must not dictate an entire serpentine path.
        std::vector<std::vector<Eigen::Vector3d>> baseNormals(numTrajectories);
        std::vector<std::vector<Eigen::Vector3d>> layerReferences(numTrajectories);
        std::vector<double> layerSignedEvidence(layerCount, 0.0);
        std::vector<double> layerAbsoluteEvidence(layerCount, 0.0);
        constexpr double referenceStrength = 0.15;

        for (int trajectoryIndex = 0; trajectoryIndex < numTrajectories; ++trajectoryIndex) {
            const auto& layerPoints = sampledPoints[trajectoryIndex];
            baseNormals[trajectoryIndex].resize(layerPoints.size());
            layerReferences[trajectoryIndex].assign(layerPoints.size(), Eigen::Vector3d::Zero());

            for (size_t layerIndex = 0; layerIndex < layerPoints.size(); ++layerIndex) {
                const auto& point = layerPoints[layerIndex];
                const TrajVector3D normalTemp = calculateNormalAtPointAccurate(
                    TrajVector3D(point.x(), point.y(), point.z()), modelData);
                Eigen::Vector3d surfaceNormal(normalTemp.x, normalTemp.y, normalTemp.z);
                if (!surfaceNormal.allFinite() || surfaceNormal.norm() <= 1e-8) {
                    surfaceNormal = Eigen::Vector3d::UnitZ();
                } else {
                    surfaceNormal.normalize();
                }

                Eigen::Vector3d layerReference = point - layerCenters[layerIndex];
                if (trajParams.offsetDistance < 0.0) layerReference = -layerReference;
                const bool hasLayerReference = layerReference.allFinite() &&
                    layerReference.norm() > 1e-8;
                double referenceDot = hasLayerReference
                    ? surfaceNormal.dot(layerReference.normalized()) : 0.0;

                // Caps and other nearly tangential locations retain the old
                // global fallback.  They do not contribute to the layer fit.
                if (!hasLayerReference || std::abs(referenceDot) < referenceStrength) {
                    Eigen::Vector3d globalReference = point - modelCenter;
                    if (trajParams.offsetDistance < 0.0) globalReference = -globalReference;
                    if (globalReference.allFinite() && globalReference.norm() > 1e-8) {
                        referenceDot = surfaceNormal.dot(globalReference.normalized());
                    }
                }
                if (referenceDot < 0.0) surfaceNormal = -surfaceNormal;
                baseNormals[trajectoryIndex][layerIndex] = surfaceNormal;

                if (hasLayerReference) {
                    layerReferences[trajectoryIndex][layerIndex] = layerReference.normalized();
                    const double fittedDot = surfaceNormal.dot(layerReferences[trajectoryIndex][layerIndex]);
                    if (std::abs(fittedDot) >= referenceStrength) {
                        layerSignedEvidence[layerIndex] += fittedDot;
                        layerAbsoluteEvidence[layerIndex] += std::abs(fittedDot);
                    }
                }
            }
        }

        // A cylindrical layer has changing normal vectors around its contour,
        // so do not average vectors.  Instead fit their common radial sign.
        std::vector<int> layerTrendSigns(layerCount, 0);
        for (size_t layerIndex = 0; layerIndex < layerCount; ++layerIndex) {
            if (layerAbsoluteEvidence[layerIndex] > 1e-8 &&
                std::abs(layerSignedEvidence[layerIndex]) /
                    layerAbsoluteEvidence[layerIndex] >= 0.20) {
                layerTrendSigns[layerIndex] = layerSignedEvidence[layerIndex] >= 0.0 ? 1 : -1;
            }
        }

        // Generate points for each trajectory
        for (int trajectoryIndex = 0; trajectoryIndex < numTrajectories; ++trajectoryIndex) {
            if (trajectoryIndex >= static_cast<int>(sampledPoints.size())) {
                continue;
            }

            const std::vector<Eigen::Vector3d>& layerPoints = sampledPoints[trajectoryIndex];
            if (trajParams.previewFirstPoint && trajectoryIndex > 0) break;

            // [Key] Determine whether to reverse: odd indices (1st, 3rd, 5th...) run top to bottom
            bool reverseDirection = (trajectoryIndex % 2 == 1);

            // Keep the sign continuous along this trajectory.  The radial
            // center test below is useful for selecting the initial side, but
            // it is not reliable at concave corners or where contour samples
            // rotate between layers.  Without this state a single ambiguous
            // STL face can produce an isolated 180-degree pose flip.
            Eigen::Vector3d previousNormal = Eigen::Vector3d::Zero();

            for (size_t i = 0; i < layerPoints.size(); ++i) {
                // A first offset point also needs the physical boundary pose.
                if (trajParams.previewFirstPoint && i > (trajParams.z1Offset != 0.0 ? 1u : 0u)) break;
                // [Serpentine] Select point index based on direction
                size_t pointIndex = reverseDirection ? (layerPoints.size() - 1 - i) : i;
                const auto& point = layerPoints[pointIndex];

                // Empty-run layers are pure translations of their physical
                // boundary. Keep that boundary's orientation; only their
                // position changes along the boundary-to-adjacent-slice line.
                size_t poseSourceIndex = pointIndex;
                if (layerPoints.size() >= 3) {
                    if (pointIndex == 0 && trajParams.z1Offset != 0.0) {
                        poseSourceIndex = 1;
                    } else if (pointIndex + 1 == layerPoints.size() && trajParams.z2Offset != 0.0) {
                        poseSourceIndex = layerPoints.size() - 2;
                    }
                }

                // 1. Start with the independently oriented normal.  A layer
                // consensus is applied after path continuity below.
                Eigen::Vector3d surfaceNormal = baseNormals[trajectoryIndex][poseSourceIndex];

                // Enforce local continuity after the absolute side has been
                // selected.  Use the traversal order (including serpentine
                // reversal), so neighboring generated poses cannot disagree
                // solely because their nearest STL faces have opposite
                // winding or because the radial reference is ill-conditioned.
                if (previousNormal.squaredNorm() > 1e-12 &&
                    surfaceNormal.dot(previousNormal) < -0.85) {
                    surfaceNormal = -surfaceNormal;
                }

                // Correct a trajectory-local continuity flip with the fitted
                // radial sign of all trajectories in this same layer.  Weak
                // cap references are intentionally left to local continuity.
                const Eigen::Vector3d& layerReference =
                    layerReferences[trajectoryIndex][poseSourceIndex];
                if (poseSourceIndex < layerTrendSigns.size() &&
                    layerTrendSigns[poseSourceIndex] != 0 &&
                    layerReference.squaredNorm() > 1e-12) {
                    const double fittedDot = surfaceNormal.dot(layerReference);
                    if (std::abs(fittedDot) >= referenceStrength &&
                        fittedDot * layerTrendSigns[poseSourceIndex] < 0.0) {
                        surfaceNormal = -surfaceNormal;
                    }
                }
                previousNormal = surfaceNormal;

                // 2. X uses the fixed lower-to-upper layer direction. The
                // return stroke traverses points in reverse order, but its X
                // axis intentionally remains aligned with the outward stroke.
                Eigen::Vector3d xAxis = Eigen::Vector3d::UnitX();
                if(pointIndex + 1 < layerPoints.size()) {
                    xAxis = layerPoints[pointIndex + 1] - point;
                } else if(pointIndex > 0) {
                    xAxis = point - layerPoints[pointIndex - 1];
                }
                // A pose frame must be orthogonal: preserve the part of the
                // travel direction lying in the tangent plane.
                xAxis -= surfaceNormal * xAxis.dot(surfaceNormal);
                if(xAxis.norm() <= 1e-8) {
                    const Eigen::Vector3d referenceAxis =
                        std::abs(surfaceNormal.z()) < 0.9
                        ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
                    xAxis = referenceAxis.cross(surfaceNormal);
                }
                xAxis.normalize();
                Eigen::Vector3d yAxis = surfaceNormal.cross(xAxis).normalized();

                // 5. The tool position is offset opposite to its pose normal.
                // The distance sign has already selected the normal direction
                // above, so use only its magnitude here.
                Eigen::Vector3d offsetPos = point - surfaceNormal *
                    std::abs(trajParams.offsetDistance);
                Eigen::Vector3d zAxis = surfaceNormal;

                // 6. Build the right-handed frame: X follows travel, Z is
                // the spray normal, and Y is derived from them.
                Eigen::Matrix3d rotationMatrix;
                rotationMatrix.col(0) = xAxis;
                rotationMatrix.col(1) = yAxis;
                rotationMatrix.col(2) = zAxis;

                // 8. Create and fill TrajectoryPoint
                TrajectoryPoint trajPoint;
                trajPoint.pose = Eigen::Matrix4d::Identity();

                // Fill rotation block
                trajPoint.pose.block<3, 3>(0, 0) = rotationMatrix;

                // Fill translation block
                trajPoint.pose(0, 3) = offsetPos.x();
                trajPoint.pose(1, 3) = offsetPos.y();
                trajPoint.pose(2, 3) = offsetPos.z();

                trajPoint.velocity = trajParams.normalVelocity;
                trajPoint.acceleration = trajParams.acceleration;
                trajPoint.point_type = 0;
                // spray_distance is a physical nozzle-to-surface length. The
                // offset sign selects the pose direction, never a negative
                // propagation distance for the thickness model.
                trajPoint.spray_distance = std::abs(trajParams.offsetDistance);

                // Write indices
                trajPoint.trajectory_index = trajectoryIndex;
                trajPoint.layer_index = static_cast<int>(pointIndex);

                trajectoryPoints.push_back(trajPoint);
            }
        }

        // Empty-run points are translations of the adjacent physical boundary
        // layer. Reconstructing X/Y from an offset point can rotate its frame
        // at an end corner, so copy the completed boundary pose verbatim and
        // retain only the offset point's translation.
        if((trajParams.z1Offset != 0.0 || trajParams.z2Offset != 0.0) &&
            !trajectoryPoints.empty()) {
            std::map<std::pair<int, int>, Eigen::Matrix4d> boundaryPoses;
            for(const TrajectoryPoint& point : trajectoryPoints) {
                boundaryPoses[{ point.trajectory_index, point.layer_index }] = point.pose;
            }
            for(TrajectoryPoint& point : trajectoryPoints) {
                if(point.trajectory_index < 0 ||
                    point.trajectory_index >= static_cast<int>(sampledPoints.size())) {
                    continue;
                }
                const int layerCount = static_cast<int>(sampledPoints[
                    static_cast<std::size_t>(point.trajectory_index)].size());
                int sourceLayer = -1;
                if(trajParams.z1Offset != 0.0 && point.layer_index == 0) {
                    sourceLayer = 1;
                } else if(trajParams.z2Offset != 0.0 && point.layer_index == layerCount - 1) {
                    sourceLayer = layerCount - 2;
                }
                const auto source = boundaryPoses.find({ point.trajectory_index, sourceLayer });
                if(source == boundaryPoses.end()) {
                    continue;
                }
                const Eigen::Vector3d position = point.pose.block<3, 1>(0, 3);
                point.pose = source->second;
                point.pose.block<3, 1>(0, 3) = position;
            }
        }
        if (trajParams.previewFirstPoint && trajectoryPoints.size() > 1) trajectoryPoints.resize(1);
        const Eigen::Matrix3d mapping = trajectoryFrameMapping(trajParams.normalAxis, trajParams.forwardAxis);
        for (auto& point : trajectoryPoints) {
            point.pose.block<3, 3>(0, 0) = (point.pose.block<3, 3>(0, 0) * mapping).eval();
        }
        return !trajectoryPoints.empty();
    }

    // [Addition] Uniformly sample points on section lines
    std::vector<std::vector<Eigen::Vector3d>> TrajectoryGenerator::samplePointsFromBoundaryLines(
        const std::vector<BoundaryLine>& boundaryLines,
        int numTrajectories,
        const std::vector<LinePoint>& phaseReferenceLine,
        const Eigen::Vector3d& phaseReferencePlaneNormal)
    {
        std::vector<std::vector<Eigen::Vector3d>> result(numTrajectories);
        if (boundaryLines.empty() || numTrajectories <= 0) return result;

        const auto pointToSegment = [](const Eigen::Vector3d& point,
            const Eigen::Vector3d& start, const Eigen::Vector3d& end, double& fraction) {
            const Eigen::Vector3d segment = end - start;
            const double segmentLengthSq = segment.squaredNorm();
            fraction = segmentLengthSq > 1e-12
                ? std::clamp((point - start).dot(segment) / segmentLengthSq, 0.0, 1.0)
                : 0.0;
            return (point - (start + fraction * segment)).squaredNorm();
        };

        for (size_t bIdx = 0; bIdx < boundaryLines.size(); ++bIdx) {
            const auto& currLine = boundaryLines[bIdx];
            if (currLine.points.empty()) continue;

            // --- 1. Precompute arc-length info for current layer ---
            std::vector<double> cumLengths;
            cumLengths.push_back(0.0);
            double totalLength = 0.0;
            for (size_t i = 1; i < currLine.points.size(); ++i) {
                totalLength += (currLine.points[i].position - currLine.points[i - 1].position).norm();
                cumLengths.push_back(totalLength);
            }
            totalLength += (currLine.points[0].position - currLine.points.back().position).norm(); // Close the loop

            if (totalLength < 1e-9) continue;

            // --- 2. Locate the global skeleton seam on this contour. ---
            // The reference plane crosses each slicing plane at one surface
            // point. Use that point as P(0), so c0 lies directly on the
            // cutting-line reference instead of half a pitch beside it.
            double seamLength = 0.0;
            double bestDistanceSq = std::numeric_limits<double>::infinity();
            if (phaseReferenceLine.size() >= 2 && currLine.normal.norm() > 1e-8) {
                const Eigen::Vector3d planeNormal = currLine.normal.normalized();
                // BoundaryLine::center is a cached construction value and is
                // not reliable for every transformed first AA slice. The
                // centroid of this contour lies on its actual cutting plane,
                // so use it as the plane equation's reference point.
                Eigen::Vector3d planePoint = Eigen::Vector3d::Zero();
                for (const auto& contourPoint : currLine.points) {
                    planePoint += contourPoint.position;
                }
                planePoint /= static_cast<double>(currLine.points.size());
                std::vector<Eigen::Vector3d> seamCandidates;
                Eigen::Vector3d nearestReferencePoint = phaseReferenceLine.front().position;
                double nearestPlaneDistance = std::numeric_limits<double>::infinity();
                for (size_t referenceIndex = 1; referenceIndex < phaseReferenceLine.size(); ++referenceIndex) {
                    const Eigen::Vector3d& r0 = phaseReferenceLine[referenceIndex - 1].position;
                    const Eigen::Vector3d& r1 = phaseReferenceLine[referenceIndex].position;
                    const double d0 = planeNormal.dot(r0 - planePoint);
                    const double d1 = planeNormal.dot(r1 - planePoint);
                    if (std::abs(d0) < nearestPlaneDistance) {
                        nearestPlaneDistance = std::abs(d0);
                        nearestReferencePoint = r0;
                    }
                    if (std::abs(d1) < nearestPlaneDistance) {
                        nearestPlaneDistance = std::abs(d1);
                        nearestReferencePoint = r1;
                    }
                    constexpr double planeTolerance = 1e-8;
                    if (std::abs(d0) <= planeTolerance) {
                        seamCandidates.push_back(r0);
                    }
                    if (referenceIndex + 1 == phaseReferenceLine.size() &&
                        std::abs(d1) <= planeTolerance) {
                        seamCandidates.push_back(r1);
                    }
                    if (d0 * d1 < 0.0) {
                        const double fraction = d0 / (d0 - d1);
                        seamCandidates.push_back(r0 + fraction * (r1 - r0));
                    }
                }

                // A transformed contour should always intersect the global
                // reference curve. If numerical tolerance prevents detecting
                // the crossing, use the reference point closest to this plane
                // rather than silently resetting the seam to contour point 0.
                if (seamCandidates.empty()) {
                    std::cerr << "Phase reference did not intersect contour plane; using nearest reference point."
                        << "layer" << static_cast<int>(bIdx)
                        << "plane distance" << nearestPlaneDistance << std::endl;
                    seamCandidates.push_back(nearestReferencePoint);
                }

                // Intersecting a plane with the reference polyline and then
                // projecting to the closed contour is O(reference + contour),
                // rather than testing every reference segment against every
                // contour segment.
                for (const auto& candidate : seamCandidates) {
                    for (size_t contourIndex = 0; contourIndex < currLine.points.size(); ++contourIndex) {
                        const size_t nextIndex = (contourIndex + 1) % currLine.points.size();
                        double contourFraction = 0.0;
                        const double distanceSq = pointToSegment(candidate,
                            currLine.points[contourIndex].position,
                            currLine.points[nextIndex].position, contourFraction);
                        if (distanceSq < bestDistanceSq) {
                            bestDistanceSq = distanceSq;
                            seamLength = cumLengths[contourIndex] + contourFraction *
                                (currLine.points[nextIndex].position - currLine.points[contourIndex].position).norm();
                        }
                    }
                }
            }

            // --- 3. Select a consistent direction from the skeleton plane. ---
            // Boundary contours can be stored in the opposite winding from
            // internal slice contours. Do not mutate their points or normals;
            // only choose the output parameter direction so c0 is consistently
            // on the positive side of the one global phase-reference plane.
            bool reverseSampling = false;
            if (phaseReferencePlaneNormal.norm() > 1e-8) {
                const Eigen::Vector3d seamPoint =
                    getPointAtLength(currLine.points, cumLengths, totalLength, seamLength);
                const Eigen::Vector3d forwardC1 = getPointAtLength(
                    currLine.points, cumLengths, totalLength,
                    seamLength + totalLength / numTrajectories);
                reverseSampling = phaseReferencePlaneNormal.dot(forwardC1 - seamPoint) < 0.0;
            }

            // --- 4. Generate evenly spaced trajectory points from the cutting line. ---
            for (int trajIdx = 0; trajIdx < numTrajectories; ++trajIdx) {
                const double targetLength = reverseSampling
                    ? seamLength - (totalLength * trajIdx) / numTrajectories
                    : seamLength + (totalLength * trajIdx) / numTrajectories;

                Eigen::Vector3d pt = getPointAtLength(currLine.points, cumLengths, totalLength, targetLength);
                result[trajIdx].push_back(pt);
            }
        }

        return result;
    }


    // [Enhanced] Compute point normal using STL triangles (corresponds to MATLAB addNormalsToReferenceLines)
    // [Important] point is sampled point position (from trajData.sampledBoundaryPoints_)
    // Find nearby triangles around sampled point and compute averaged normal
    TrajVector3D TrajectoryGenerator::calculateNormalAtPoint(const TrajVector3D& point, const ModelData* modelData) const
    {
        if (!modelData || STLProcessor::getTriangleCount(modelData->mainModelData_) == 0) {
            // If model data is unavailable, use simplified radial normal
            TrajVector3D normal;
            double length = sqrt(point.x * point.x + point.y * point.y);
            if (length > 1e-8) {
                normal.x = point.x / length;
                normal.y = point.y / length;
                normal.z = 0.0;
            }
            else {
                normal = TrajVector3D(0, 0, 1);
            }
            return normal;
        }

        // Compute normal from model triangles (corresponding MATLAB behavior)
        const double threshold = 50.0;  // Distance threshold, MATLAB threshold = 50
        std::vector<TrajVector3D> nearbyNormals;

        // Compute distance from point to each triangle
        for (size_t triangleIndex = 0; triangleIndex < STLProcessor::getTriangleCount(modelData->mainModelData_); ++triangleIndex) {
            // Get triangle vertices
            Eigen::Vector3d vertices[3];
            STLProcessor::getTriangleVertices(modelData->mainModelData_, triangleIndex, vertices);

            // Compute triangle center
            TrajVector3D triangleCenter;
            triangleCenter.x = (vertices[0].x() + vertices[1].x() + vertices[2].x()) / 3.0;
            triangleCenter.y = (vertices[0].y() + vertices[1].y() + vertices[2].y()) / 3.0;
            triangleCenter.z = (vertices[0].z() + vertices[1].z() + vertices[2].z()) / 3.0;

            // Compute distance
            double dx = triangleCenter.x - point.x;
            double dy = triangleCenter.y - point.y;
            double dz = triangleCenter.z - point.z;
            double distance = sqrt(dx * dx + dy * dy + dz * dz);

            // If within threshold, add normal
            if (distance < threshold) {
                Eigen::Vector3d triangleNormal = STLProcessor::getTriangleNormal(modelData->mainModelData_, triangleIndex);
                TrajVector3D normal;
                normal.x = triangleNormal.x();
                normal.y = triangleNormal.y();
                normal.z = triangleNormal.z();
                nearbyNormals.push_back(normal);
            }
        }

        // If no nearby triangles found, expand search range
        if (nearbyNormals.empty()) {
            const double expandedThreshold = 130.0;  // MATLAB threshold = 130
            for (size_t triangleIndex = 0; triangleIndex < STLProcessor::getTriangleCount(modelData->mainModelData_); ++triangleIndex) {
                // Get triangle vertices
                Eigen::Vector3d vertices[3];
                STLProcessor::getTriangleVertices(modelData->mainModelData_, triangleIndex, vertices);

                TrajVector3D triangleCenter;
                triangleCenter.x = (vertices[0].x() + vertices[1].x() + vertices[2].x()) / 3.0;
                triangleCenter.y = (vertices[0].y() + vertices[1].y() + vertices[2].y()) / 3.0;
                triangleCenter.z = (vertices[0].z() + vertices[1].z() + vertices[2].z()) / 3.0;

                double dx = triangleCenter.x - point.x;
                double dy = triangleCenter.y - point.y;
                double dz = triangleCenter.z - point.z;
                double distance = sqrt(dx * dx + dy * dy + dz * dz);

                if (distance < expandedThreshold) {
                    Eigen::Vector3d triangleNormal = STLProcessor::getTriangleNormal(modelData->mainModelData_, triangleIndex);
                    TrajVector3D normal;
                    normal.x = triangleNormal.x();
                    normal.y = triangleNormal.y();
                    normal.z = triangleNormal.z();
                    nearbyNormals.push_back(normal);
                }
            }
        }

        // Compute average normal (corresponds to MATLAB mean(nearbyNormals, 1))
        if (!nearbyNormals.empty()) {
            TrajVector3D avgNormal(0, 0, 0);
            for (const auto& normal : nearbyNormals) {
                avgNormal.x += normal.x;
                avgNormal.y += normal.y;
                avgNormal.z += normal.z;
            }
            avgNormal.x /= nearbyNormals.size();
            avgNormal.y /= nearbyNormals.size();
            avgNormal.z /= nearbyNormals.size();

            // Normalize normal vector
            double length = sqrt(avgNormal.x * avgNormal.x + avgNormal.y * avgNormal.y + avgNormal.z * avgNormal.z);
            if (length > 1e-8) {
                avgNormal.x /= length;
                avgNormal.y /= length;
                avgNormal.z /= length;
            }

            return avgNormal;
        }

        // If still not found, return default upward normal
        return TrajVector3D(0, 0, 1);
    }

    // [Addition] Accurately compute sampled-point normal (using point-to-triangle-plane distance and weighted averaging)
    // This function computes normals directly from sampled point position and STL model, independent of boundary-line normals
    TrajVector3D TrajectoryGenerator::calculateNormalAtPointAccurate(const TrajVector3D& point, const ModelData* modelData) const
    {
        if (!modelData || STLProcessor::getTriangleCount(modelData->mainModelData_) == 0) {
            // The caller rejects this case. Never fabricate a radial normal:
            // it is unrelated to the actual model surface.
            return TrajVector3D(0, 0, 1);
        }

        // Use the distance to the finite triangle, not to its infinite plane.
        // An infinite plane can pass near a point even when the corresponding
        // mesh face is far away, which makes the average look radial/global.
        const double threshold = 50.0;
        std::vector<std::pair<TrajVector3D, double>> weightedNormals;
        Eigen::Vector3d nearestNormal = Eigen::Vector3d::UnitZ();
        double nearestDistance = std::numeric_limits<double>::max();

        Eigen::Vector3d pointVec(point.x, point.y, point.z);
        for (size_t triangleIndex = 0; triangleIndex < STLProcessor::getTriangleCount(modelData->mainModelData_); ++triangleIndex) {
            Eigen::Vector3d vertices[3];
            STLProcessor::getTriangleVertices(modelData->mainModelData_, triangleIndex, vertices);

            Eigen::Vector3d edge1 = vertices[1] - vertices[0];
            Eigen::Vector3d edge2 = vertices[2] - vertices[0];
            Eigen::Vector3d triangleNormal = edge1.cross(edge2);
            double triangleArea = triangleNormal.norm() * 0.5;
            if (triangleArea < 1e-10) {
                continue;
            }
            triangleNormal.normalize();

            const Eigen::Vector3d ab = vertices[1] - vertices[0];
            const Eigen::Vector3d ac = vertices[2] - vertices[0];
            const Eigen::Vector3d ap = pointVec - vertices[0];
            const double d1 = ab.dot(ap);
            const double d2 = ac.dot(ap);
            Eigen::Vector3d closestPoint;
            if (d1 <= 0.0 && d2 <= 0.0) {
                closestPoint = vertices[0];
            } else {
                const Eigen::Vector3d bp = pointVec - vertices[1];
                const double d3 = ab.dot(bp);
                const double d4 = ac.dot(bp);
                if (d3 >= 0.0 && d4 <= d3) {
                    closestPoint = vertices[1];
                } else {
                    const double vc = d1 * d4 - d3 * d2;
                    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
                        closestPoint = vertices[0] + ab * (d1 / (d1 - d3));
                    } else {
                        const Eigen::Vector3d cp = pointVec - vertices[2];
                        const double d5 = ab.dot(cp);
                        const double d6 = ac.dot(cp);
                        if (d6 >= 0.0 && d5 <= d6) {
                            closestPoint = vertices[2];
                        } else {
                            const double vb = d5 * d2 - d1 * d6;
                            if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
                                closestPoint = vertices[0] + ac * (d2 / (d2 - d6));
                            } else {
                                const double va = d3 * d6 - d5 * d4;
                                if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
                                    const Eigen::Vector3d bc = vertices[2] - vertices[1];
                                    closestPoint = vertices[1] + bc * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
                                } else {
                                    const double invDenom = 1.0 / (va + vb + vc);
                                    closestPoint = vertices[0] + ab * (vb * invDenom) + ac * (vc * invDenom);
                                }
                            }
                        }
                    }
                }
            }

            const double distance = (pointVec - closestPoint).norm();
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestNormal = triangleNormal;
            }
            if (distance <= threshold) {
                weightedNormals.push_back({
                    TrajVector3D(triangleNormal.x(), triangleNormal.y(), triangleNormal.z()),
                    triangleArea / (1.0 + distance * distance)
                });
            }
        }

        // Points outside the local threshold still retain the true closest
        // face normal; no global plane-based expansion is permitted.
        if (weightedNormals.empty() && nearestDistance < std::numeric_limits<double>::max()) {
            return TrajVector3D(nearestNormal.x(), nearestNormal.y(), nearestNormal.z());
        }

        // Compute weighted average normal
        if (!weightedNormals.empty()) {
            TrajVector3D weightedSum(0, 0, 0);
            double totalWeight = 0.0;

            for (const auto& pair : weightedNormals) {
                const TrajVector3D& normal = pair.first;
                double weight = pair.second;
                Eigen::Vector3d alignedNormal(normal.x, normal.y, normal.z);

                // STL files can contain locally reversed triangle winding.
                // Align every contributing face to the closest face before
                // averaging so opposite normals never cancel or alternate.
                if (alignedNormal.dot(nearestNormal) < 0.0) {
                    alignedNormal = -alignedNormal;
                }

                weightedSum.x += alignedNormal.x() * weight;
                weightedSum.y += alignedNormal.y() * weight;
                weightedSum.z += alignedNormal.z() * weight;
                totalWeight += weight;
            }

            if (totalWeight > 1e-10) {
                weightedSum.x /= totalWeight;
                weightedSum.y /= totalWeight;
                weightedSum.z /= totalWeight;
            }

            // Normalize normal vector
            double length = sqrt(weightedSum.x * weightedSum.x + weightedSum.y * weightedSum.y + weightedSum.z * weightedSum.z);
            if (length > 1e-8) {
                weightedSum.x /= length;
                weightedSum.y /= length;
                weightedSum.z /= length;
            }
            else {
                // If normalization fails, return default upward normal
                return TrajVector3D(0, 0, 1);
            }

            return weightedSum;
        }

        // If still not found, return default upward normal
        return TrajVector3D(0, 0, 1);
    }


    // Format number for output
    std::string TrajectoryGenerator::formatNumber(double value, int precision) const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        return oss.str();
    }

}
