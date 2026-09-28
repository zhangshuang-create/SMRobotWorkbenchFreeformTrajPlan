#pragma once

#include "TrajectoryConnector_global.h"
#include <GeneralData/GeneralData.h>
#include <RenderData/RenderData.h>
#include <ModelTangQing/imodel.h>
#include <ModelTangQing/MeshData.h>
#include <ModelTangQing/iscene.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
namespace ModelTrajPlan
{
    // Forward declaration
    class TrajectoryGenerator;

    // Trajectory connector class - Pure algorithm, no data storage
    class TrajectoryConnector_SHAREDLIB TrajectoryConnector
    {
    public:
        TrajectoryConnector();
        ~TrajectoryConnector();

        // === Core connection functionality (pure algorithm) ===

        // [ALGORITHM] Connect grouped trajectory points (bow-path strategy)
        // Input: groupedTrajectories - grouped trajectory data
        //        strategy - connection strategy
        // Output: trajData.connectionData_ - connected trajectory and segments
        static bool connectTrajectoryPoints(
            const std::vector<std::vector<TrajectoryPoint>>& groupedTrajectories,
            ConnectionStrategy strategy,
            TrajData& trajData);

        bool initializeRenderer(ConnectionRenderData& renderData);

        // [ALGORITHM] Convert trajectory points to grouped format using sampled boundary points structure
        // Input: trajectoryPoints - flat trajectory points vector (ordered by trajectory, then by layer)
        //        sampledBoundaryPoints - structure that defines grouping [trajectoryIndex][layerIndex]
        // Output: vector<vector<TrajectoryPoint>> - grouped trajectory data
        static std::vector<std::vector<TrajectoryPoint>> convertTrajectoryPointsToGroupedFormat(
            const std::vector<TrajectoryPoint>& trajectoryPoints,
            const std::vector<std::vector<Eigen::Vector3d>>& sampledBoundaryPoints);

        // === Visualization functionality (pure algorithm) ===


        // [RENDER] Draw connection lines
        // Input: trajData.connectionData_ - connection data
        // Output: renderData - rendering calls
        static bool draw(ConnectionRenderData& renderData, const TrajData& trajData);

        // Set connection line visibility
        static void setConnectionVisibility(ConnectionRenderData& renderData, bool visible);

        // === State query (read from data) ===

        static bool hasValidData(const TrajData& trajData);
        static bool isConnected(const TrajData& trajData);
        static bool hasRenderableData(const TrajData& trajData);

    private:
        // === No internal data storage ===

        // [ALGORITHM] Private algorithm methods (all static)

        // [ALGORITHM] Bow-path connection algorithm
        // Input: groupedTrajectories - grouped trajectory data
        // Output: connectionData - connected trajectory and segments
        static bool connectBowPath(
            const std::vector<std::vector<TrajectoryPoint>>& groupedTrajectories,
            ConnectionData& connectionData);

        // [RENDER] Prepare render data
        // Input: connectionSegments - connection segment data
        // Output: renderData - updated rendering data
        static void prepareRenderData(ConnectionRenderData& renderData,
            const std::vector<ConnectionSegment>& connectionSegments);

        // [ALGORITHM] Data conversion
        static MeshData convertConnectionSegmentsToModelData(
            const std::vector<ConnectionSegment>& connectionSegments);  // Return-by-value version
        static void convertConnectionSegmentsToModelData(
            const std::vector<ConnectionSegment>& connectionSegments,
            MeshData& connectionData);  // Reference-parameter version (avoids temporaries)
    };
}