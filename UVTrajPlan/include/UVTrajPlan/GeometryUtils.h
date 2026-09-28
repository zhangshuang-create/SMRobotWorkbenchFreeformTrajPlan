#pragma once

#include "CommonDataTypes.h"
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <cstdint>

namespace SprayTrajPlan
{
    class GeometryUtils
    {
    public:
        struct ProjectionStatistics {
            std::uint64_t cpu_batches = 0;
            std::uint64_t gpu_batches = 0;
            std::uint64_t cpu_queries = 0;
            std::uint64_t gpu_queries = 0;
            double cpu_milliseconds = 0.0;
            double gpu_milliseconds = 0.0;
        };

        static void resetProjectionStatistics();
        static ProjectionStatistics projectionStatistics();

        // Compute shortest distance between two 3D line segments
        static double segmentDistance3D(
            const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
            const Eigen::Vector3d& q0, const Eigen::Vector3d& q1
        );

        // Compute distance from a line segment to a set of points
        static double lineToPointsDistance(
            const std::pair<Eigen::Vector3d, Eigen::Vector3d>& line,
            const std::vector<Eigen::Vector3d>& points
        );

        // Track a slice from start to end through sampled points
        static std::vector<Eigen::Vector3d> trackSlice(
            const std::vector<Eigen::Vector3d>& points,
            const Eigen::Vector3d& start,
            const Eigen::Vector3d& end
        );

        // Test if 2D points are inside a polygon (even-odd rule)
        static std::vector<bool> pointsInPolygon(
            const std::vector<Eigen::Vector2d>& points,
            const std::vector<Eigen::Vector2d>& polygon
        );

        // Get local connected face mask
        static std::vector<bool> localConnectedFaceMask(
            const std::vector<Eigen::Vector3d>& triangle_centers,
            const std::vector<Eigen::Vector3i>& faces,
            const std::vector<std::pair<int, int>>& face_adjacency,
            const std::vector<bool>& candidate_mask,
            const std::vector<Eigen::Vector3d>& boundary_points
        );

        // Project point to mesh surface
        static std::optional<Eigen::Vector3d> projectToMesh(
            const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<Eigen::Vector3i>& faces,
            const Eigen::Vector3d& point
        );

        // Compute geometric centroid of a closed polyline
        static Eigen::Vector3d closedPolylineCenter(
            const std::vector<Eigen::Vector3d>& points
        );

        // Robust closest point query with fallback
        static void closestPointsRobust(
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const std::vector<Eigen::Vector3d>& query_points,
            std::vector<Eigen::Vector3d>& closest_points,
            std::vector<double>& distances,
            std::vector<int>& face_indices
        );

        // Radial barrier state computation
        struct BarrierState {
            std::vector<Eigen::Vector3d> force;
            std::vector<Eigen::Vector3d> direction;
            std::vector<bool> dangerous;
            std::vector<bool> inside_any;
            std::vector<double> escape_step;
        };

        struct RadialBarrier {
            Eigen::Vector3d center;
            std::vector<Eigen::Vector3d> boundary_points;
        };

        static BarrierState radialBarrierState(
            const std::vector<Eigen::Vector3d>& points,
            const std::vector<RadialBarrier>& barriers,
            double spacing,
            double stiffness
        );

        // Special boundary geometry computation
        struct BoundaryGeometry {
            struct TrackInfo {
                std::string region;
                int segment_index;
                int phase;
            };

            struct ConnectorRecord {
                std::string kind;           // "PERIODIC", "BOUNDARY", "MODEL_BOUNDARY"
                std::string side;           // "LOWER", "UPPER"
                std::string center_role;    // "FIRST", "LAST", "PERIODIC", "ENDPOINTS"
                std::pair<Eigen::Vector3d, Eigen::Vector3d> line_3d;
                std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> line_2d;
                std::optional<int> outer_segment_index;
                std::optional<int> outer_phase;
            };

            std::vector<TrackInfo> tracks;
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> connectors_3d;
            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> connectors_2d;
            std::vector<ConnectorRecord> connector_records;

            std::map<std::string, std::vector<ConnectorRecord>> selected_connector_records;
            std::map<std::string, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> selected_void_connectors_3d;
            std::map<std::string, Eigen::Vector3d> blank_centers_3d;
            std::map<std::string, double> blank_region_areas;
            std::map<std::string, Eigen::Vector3d> repulsion_centers_3d;

            struct VoidEligibility {
                bool eligible;
                std::string reason;
                double c0_max_gap;
                double c_last_max_gap;
                double required_gap;
            };
            std::map<std::string, VoidEligibility> void_center_eligibility;

            bool periodic_pair_blocked;
            double target_distance;
        };

        static BoundaryGeometry specialBoundaryGeometry(
            const std::vector<TrajectorySegment>& trajectory,
            double spacing,
            const std::vector<Eigen::Vector3d>* mesh_vertices = nullptr,
            const std::vector<Eigen::Vector3i>* mesh_faces = nullptr
        );

    private:
        // Helper function for KD-tree style nearest neighbor search
        static int findNearestPoint(
            const std::vector<Eigen::Vector3d>& points,
            const Eigen::Vector3d& query
        );

        static std::vector<int> findNearestPoints(
            const std::vector<Eigen::Vector3d>& points,
            const std::vector<Eigen::Vector3d>& queries
        );
    };

} // namespace SprayTrajPlan
