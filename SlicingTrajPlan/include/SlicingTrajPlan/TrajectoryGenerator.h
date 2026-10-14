#pragma once

#include "TrajectoryGenerator_global.h"
#include <SlicingTrajPlan/GeneralData.h>
#include <vector>
#include <string>
#include <memory>
// Define PI constant
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace ModelTrajPlan
{
    // Forward declarations
    struct BoundaryLine;
    struct SliceResult;
    struct ModelData;
    struct EdgeExtractionResult;

    struct SectionProps {
        Eigen::Vector3d center;
        Eigen::Vector3d normal;
    };

    // Trajectory generator class - [Refactor] converted to a stateless pure algorithm class
    class TrajectoryGenerator_SHAREDLIB TrajectoryGenerator {
    public:
        TrajectoryGenerator();
        ~TrajectoryGenerator();

        // === Main generation workflow ===

        // [ALGORITHM] Main MATLAB trajectory generation method
        // Input: upperBoundary, lowerBoundary - boundary data
        //        sliceResult - slicing result
        //        trajParams - trajectory generation parameters (offset parameters)
        //        modelData - STL model data for normal calculation (optional, can be nullptr)
        // Output: referenceLines - generated reference lines (output via reference parameter)
        //         trajectoryPoints - generated trajectory points (output via reference parameter)
        //         groupInfo - trajectory group information (output via reference parameter)
        // Return: true if successful, false otherwise
        bool generateMATLABTrajectory(const BoundaryLine& upperBoundary,
            const BoundaryLine& lowerBoundary,
            const SliceResult& sliceResult,
            const TrajectoryGenerationParams& trajParams,
            const ModelData* modelData,
            std::vector<TrajectoryPoint>& trajectoryPoints,
            TrajectoryGroupInfo& groupInfo,
            BoundaryData& boundaryData,
            SlicingData& slicingData,
            TrajData& trajData);
        // [ALGORITHM] Generate trajectory from edge points
        // Input: edgeResults - edge extraction results
        //        params - generation parameters
        // Output: trajectoryPoints - generated trajectory points

    private:
        // [ALGORITHM] Internal helper methods (all methods take explicit parameters and do not depend on member state)
        // Automatically copy boundaries and generate reference lines from offset parameters
        // [Addition] Also place sliceLines_ into boundaryLines_ in order
        bool generateReferenceLinesFromSliceResult(
            const BoundaryLine& upperBoundary,
            const BoundaryLine& lowerBoundary,
            const SliceResult& sliceResult,
            const TrajectoryGenerationParams& trajParams,
            BoundaryData& boundaryData,
            SlicingData& slicingData,
            TrajData& trajData);

        // Helper methods (static or const, independent of member state)
        TrajVector3D calculateNormalAtPoint(const TrajVector3D& point, const ModelData* modelData) const;

        // [Addition] Accurately compute sampled-point normal (using point-to-triangle-plane distance and weighted averaging)
        // This function computes normals directly from sampled-point positions using the STL model, independent of boundary-line normals
        TrajVector3D calculateNormalAtPointAccurate(const TrajVector3D& point, const ModelData* modelData) const;
        std::string formatNumber(double value, int precision = 3) const;

        // [Addition] Uniformly sample points on section lines
        // Input: boundaryLines - set of section lines
        //        numTrajectories - number of points to sample (trajectory count)
        // Output: 2D array [trajectoryIndex][layerIndex] -> point of trajectoryIndex at boundary layerIndex
        static std::vector<std::vector<Eigen::Vector3d>> samplePointsFromBoundaryLines(
            const std::vector<BoundaryLine>& boundaryLines,
            int numTrajectories,
            const std::vector<LinePoint>& phaseReferenceLine,
            const Eigen::Vector3d& phaseReferencePlaneNormal);

        // [Addition] Ensure boundary points are CCW (use signed area on XY plane; reverse when area < 0)
        static void normalizeBoundaryCCW(BoundaryLine& boundary);

        // [Addition] Generate trajectory points directly from sampled points (skip the referenceLines intermediate step)
        // Input: sampledPoints - sampled points [trajectoryIndex][layerIndex]
        //        modelData - STL model data (used for normal computation)
        //        trajParams - trajectory generation parameters
        // Output: trajectoryPoints - generated trajectory points, stored directly in trajData.trajectoryPoints_
        bool generateTrajectoryPointsFromSampledPoints(
            const std::vector<std::vector<Eigen::Vector3d>>& sampledPoints,
            const std::vector<Eigen::Vector3d>& cuttingPlaneCenters,
            const ModelData* modelData,
            const TrajectoryGenerationParams& trajParams,
            std::vector<TrajectoryPoint>& trajectoryPoints, bool startFromUpper) const;
        static Eigen::Vector3d getPointAtLength(const std::vector<BasicPoint>& points,
            const std::vector<double>& cumLengths,
            double totalLength,
            double targetLen);
    };

}
