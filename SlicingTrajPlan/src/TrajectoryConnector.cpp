#include <SlicingTrajPlan/TrajectoryConnector.h>
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace ModelTrajPlan
{
    TrajectoryConnector::TrajectoryConnector()
    {
        // [DESIGN] Pure algorithm class - no internal data storage
    }

    TrajectoryConnector::~TrajectoryConnector()
    {
    }

    // === Public Interface Implementation (All Static) ===

    bool TrajectoryConnector::connectTrajectoryPoints(
        const std::vector<std::vector<TrajectoryPoint>>& groupedTrajectories,
        ConnectionStrategy strategy,
        TrajData& trajData)
    {
        // [ALGORITHM] Connect trajectory points using specified strategy
        // Input: groupedTrajectories - grouped trajectory data
        //        strategy - connection strategy
        // Output: trajData.connectionData_ - connection result

        if (groupedTrajectories.empty()) {
            return false;
        }

        // Clear old connection data
        trajData.connectionData_.connectedTrajectory_.clear();
        trajData.connectionData_.connectionSegments_.clear();
        trajData.connectionData_.strategy_ = strategy;

        // Currently only support BowPath strategy
        bool success = connectBowPath(groupedTrajectories, trajData.connectionData_);

        if (success) {
            trajData.connectionData_.isConnected_ = true;
        }

        return success;
    }

    // [Addition] Convert trajectory points to grouped format (using sampled-point structure)
    std::vector<std::vector<TrajectoryPoint>> TrajectoryConnector::convertTrajectoryPointsToGroupedFormat(
        const std::vector<TrajectoryPoint>& trajectoryPoints,
        const std::vector<std::vector<Eigen::Vector3d>>& sampledBoundaryPoints)
    {
        // [ALGORITHM] Convert flat trajectory point vector into grouped format
        // Input: trajectoryPoints - flat trajectory points (ordered by trajectory, then by layer)
        //        sampledBoundaryPoints - grouping structure [trajectoryIndex][layerIndex]
        // Output: vector<vector<TrajectoryPoint>> - grouped trajectory data

        std::vector<std::vector<TrajectoryPoint>> groupedTrajectoryPoints;

        if (trajectoryPoints.empty() || sampledBoundaryPoints.empty()) {
            return groupedTrajectoryPoints;
        }

        int numTrajectories = static_cast<int>(sampledBoundaryPoints.size());
        groupedTrajectoryPoints.resize(numTrajectories);

        // Compute point count (layer count) for each trajectory
        std::vector<int> pointsPerTrajectory(numTrajectories);
        for (int i = 0; i < numTrajectories; ++i) {
            pointsPerTrajectory[i] = static_cast<int>(sampledBoundaryPoints[i].size());
        }

        // Extract grouped data from flat trajectoryPoints vector
        // trajectoryPoints is ordered by trajectory: all points of trajectory 0, then trajectory 1, ...
        size_t currentIndex = 0;
        for (int trajIndex = 0; trajIndex < numTrajectories; ++trajIndex) {
            int pointCount = pointsPerTrajectory[trajIndex];
            groupedTrajectoryPoints[trajIndex].reserve(pointCount);

            for (int layerIndex = 0; layerIndex < pointCount; ++layerIndex) {
                if (currentIndex < trajectoryPoints.size()) {
                    groupedTrajectoryPoints[trajIndex].push_back(trajectoryPoints[currentIndex]);
                    ++currentIndex;
                }
                else {
                    // If data mismatches expected structure, return partially grouped result
                    break;
                }
            }
        }

        return groupedTrajectoryPoints;
    }

    // === State Query (All Static) ===

    bool TrajectoryConnector::hasValidData(const TrajData& trajData)
    {
        return !trajData.connectionData_.connectedTrajectory_.empty();
    }

    bool TrajectoryConnector::isConnected(const TrajData& trajData)
    {
        return trajData.connectionData_.isConnected_;
    }

    bool TrajectoryConnector::hasRenderableData(const TrajData& trajData)
    {
        return !trajData.connectionData_.connectionSegments_.empty();
    }

    // === Connection Algorithm Implementation (All Static) ===

    bool TrajectoryConnector::connectBowPath(
        const std::vector<std::vector<TrajectoryPoint>>& trajectoryLines,
        ConnectionData& connectionData)
    {
        // [ALGORITHM] Bow-path connection algorithm
        // Input: trajectoryLines - grouped trajectory data
        // Output: connectionData - connection result

        if (trajectoryLines.empty() || trajectoryLines[0].empty()) {
            return false;
        }

        int numTrajectories = static_cast<int>(trajectoryLines.size());
        int numLayers = static_cast<int>(trajectoryLines[0].size());

        // [Key] Trajectory points are already arranged in serpentine order by TrajectoryGenerator:
        //   - Trajectory 0 (even): [layer0, layer1, ..., layer(n-1)] (forward, bottom to top)
        //   - Trajectory 1 (odd): [layer(n-1), layer(n-2), ..., layer0] (reverse, top to bottom)
        //   - Trajectory 2 (even): [layer0, layer1, ..., layer(n-1)] (forward, bottom to top)
        //   - Trajectory 3 (odd): [layer(n-1), layer(n-2), ..., layer0] (reverse, top to bottom)
        // During connection, follow array index order directly; no reindexing or sorting is required

        // Connect two trajectories as a group in a bow shape
        for (int pairIndex = 0; pairIndex < numTrajectories; pairIndex += 2) {
            int firstTrajectoryIndex = pairIndex;
            int secondTrajectoryIndex = pairIndex + 1;

            // === First trajectory, going from bottom to top ===
            // [Key] Forward trajectory point order: [layer0, layer1, ..., layer(n-1)]
            // For the second and later groups, if previous group already connected to bottom layer, skip bottom layer (it was already added during connection)
            if (firstTrajectoryIndex < numTrajectories) {
                int startArrayIndex = 0;
                // Check whether bottom layer was already connected (by previous group)
                if (pairIndex > 0 && !connectionData.connectedTrajectory_.empty()) {
                    // Previous group already connected to bottom layer, so start first trajectory from index 1
                    startArrayIndex = 1;
                }

                // Connect by array index order (forward trajectory: from index 0/1 to index n-1)
                for (int arrayIndex = startArrayIndex; arrayIndex < numLayers; ++arrayIndex) {
                    if (arrayIndex < static_cast<int>(trajectoryLines[firstTrajectoryIndex].size())) {
                        TrajectoryPoint currentPoint = trajectoryLines[firstTrajectoryIndex][arrayIndex];

                        // Add to the total trajectory and generate a connection line
                        if (!connectionData.connectedTrajectory_.empty()) {
                            TrajectoryPoint lastPoint = connectionData.connectedTrajectory_.back();

                            ConnectionSegment segment;
                            segment.startPoint = lastPoint.pose.block<3, 1>(0, 3);
                            segment.endPoint = currentPoint.pose.block<3, 1>(0, 3);
                            segment.color = Eigen::Vector3f(0.0f, 1.0f, 0.0f);  // Green - intra-trajectory
                            segment.lineWidth = 2.0f;
                            segment.segmentType = 0;
                            connectionData.connectionSegments_.push_back(segment);
                        }

                        connectionData.connectedTrajectory_.push_back(currentPoint);
                    }
                }
            }

            // === Connect from top of first to top of second trajectory ===
            // [Fix] After reaching top layer of first trajectory, connect to top layer of second trajectory
            // [Key] Trajectory points are already arranged in serpentine order by TrajectoryGenerator:
            //   - Trajectory 0: [layer0, layer1, ..., layer(n-1)] (forward)
            //   - Trajectory 1: [layer(n-1), layer(n-2), ..., layer0] (reverse, already reversed)
            // Therefore, top layer of a reverse trajectory is array index 0 (the first element)
            if (secondTrajectoryIndex < numTrajectories && !connectionData.connectedTrajectory_.empty()) {
                // Top layer of reverse trajectory is first array element (index 0)
                int topArrayIndex = 0;

                if (topArrayIndex < static_cast<int>(trajectoryLines[secondTrajectoryIndex].size())) {
                    TrajectoryPoint lastPoint = connectionData.connectedTrajectory_.back();
                    TrajectoryPoint nextTopPoint = trajectoryLines[secondTrajectoryIndex][topArrayIndex];

                    ConnectionSegment horizontalSegment;
                    horizontalSegment.startPoint = lastPoint.pose.block<3, 1>(0, 3);
                    horizontalSegment.endPoint = nextTopPoint.pose.block<3, 1>(0, 3);
                    horizontalSegment.color = Eigen::Vector3f(1.0f, 0.5f, 0.0f);  // Orange - inter-trajectory
                    horizontalSegment.lineWidth = 2.5f;
                    horizontalSegment.segmentType = 1;
                    connectionData.connectionSegments_.push_back(horizontalSegment);

                    connectionData.connectedTrajectory_.push_back(nextTopPoint);
                }
            }

            // === Second trajectory, going from top to bottom ===
            // [Fix] Traverse from top to bottom in second trajectory, skipping the top layer (already added during connection)
            // [Key] Trajectory points are already arranged in serpentine order:
            //   Trajectory 1: [layer(n-1), layer(n-2), ..., layer0] (reverse, already reversed)
            // So connect directly by array order (skip index 0 because it was already added)
            if (secondTrajectoryIndex < numTrajectories) {
                // Start from index 1 and connect in order to bottom layer (index n-1)
                for (int arrayIndex = 1; arrayIndex < numLayers; ++arrayIndex) {
                    if (arrayIndex < static_cast<int>(trajectoryLines[secondTrajectoryIndex].size())) {
                        TrajectoryPoint currentPoint = trajectoryLines[secondTrajectoryIndex][arrayIndex];

                        if (!connectionData.connectedTrajectory_.empty()) {
                            TrajectoryPoint lastPoint = connectionData.connectedTrajectory_.back();

                            ConnectionSegment segment;
                            segment.startPoint = lastPoint.pose.block<3, 1>(0, 3);
                            segment.endPoint = currentPoint.pose.block<3, 1>(0, 3);
                            segment.color = Eigen::Vector3f(0.0f, 1.0f, 0.0f);  // Green - intra-trajectory
                            segment.lineWidth = 2.0f;
                            segment.segmentType = 0;
                            connectionData.connectionSegments_.push_back(segment);
                        }

                        connectionData.connectedTrajectory_.push_back(currentPoint);
                    }
                }
            }

            // === Connect from bottom of second to bottom of next group ===
            // [Fix] After reaching bottom of second trajectory, connect to bottom of next trajectory
            // [Key] Serpentine ordering means:
            //   - Trajectory 1: [layer(n-1), layer(n-2), ..., layer0] (reverse, bottom is last element index n-1)
            //   - Trajectory 2: [layer0, layer1, ..., layer(n-1)] (forward, bottom is first element index 0)
            int thirdTrajectoryIndex = pairIndex + 2;
            if (thirdTrajectoryIndex < numTrajectories && !connectionData.connectedTrajectory_.empty()) {
                // Next trajectory (third one) is forward, bottom layer corresponds to array index 0
                int bottomArrayIndex = 0;

                if (bottomArrayIndex < static_cast<int>(trajectoryLines[thirdTrajectoryIndex].size())) {
                    TrajectoryPoint lastPoint = connectionData.connectedTrajectory_.back();
                    TrajectoryPoint nextBottomPoint = trajectoryLines[thirdTrajectoryIndex][bottomArrayIndex];

                    ConnectionSegment horizontalSegment;
                    horizontalSegment.startPoint = lastPoint.pose.block<3, 1>(0, 3);
                    horizontalSegment.endPoint = nextBottomPoint.pose.block<3, 1>(0, 3);
                    horizontalSegment.color = Eigen::Vector3f(1.0f, 0.5f, 0.0f);  // Orange - inter-trajectory
                    horizontalSegment.lineWidth = 2.5f;
                    horizontalSegment.segmentType = 1;
                    connectionData.connectionSegments_.push_back(horizontalSegment);

                    connectionData.connectedTrajectory_.push_back(nextBottomPoint);
                }
            }

        }

        // The pair loop also handles an odd final trajectory as its first
        // member with no second member. Reprocessing it here used to connect
        // its end back to index 1, producing an unwanted return stroke.

        return true;
    }
}
