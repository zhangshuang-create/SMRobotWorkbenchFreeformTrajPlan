#pragma once

#include "CommonDataTypes.h"
#include <vector>
#include <set>
#include <map>

namespace SprayTrajPlan
{
    // Mesh cutting and boundary detection utilities
    class MeshCutter
    {
    public:
        // Extract boundary vertices and edges from faces
        static std::pair<std::vector<int>, std::vector<std::pair<int, int>>>
            boundaryVerticesFromFaces(const std::vector<Eigen::Vector3i>& faces);

        // Order boundary loops from boundary edges
        static std::vector<std::vector<int>>
            orderedBoundaryLoops(const std::vector<Eigen::Vector3i>& faces);

        // Extract all mesh edges from faces
        static std::vector<std::pair<int, int>>
            meshEdgesFromFaces(const std::vector<Eigen::Vector3i>& faces);

        // Find faces to duplicate along seam path
        static std::map<int, std::set<int>>
            facesToDuplicateAlongSeam(
                const std::vector<Eigen::Vector3i>& faces,
                const std::vector<int>& seam_path
            );

        // Cut mesh along a path
        static bool cutAlongPath(
            const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<Eigen::Vector3i>& faces,
            const std::vector<int>& seam_path,
            std::vector<Eigen::Vector3d>& output_vertices,
            std::vector<Eigen::Vector3i>& output_faces,
            std::vector<int>& vertex_map
        );

        // Pre-cut mesh at two boundaries
        static bool precutTwoBoundaries(
            const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<Eigen::Vector3i>& faces,
            std::vector<Eigen::Vector3d>& output_vertices,
            std::vector<Eigen::Vector3i>& output_faces,
            PrecutSeam& seam_data
        );

    private:
        // Helper: Check if vertex is locally manifold
        static bool isLocallyManifold(
            int vertex,
            const std::vector<Eigen::Vector3i>& faces,
            const std::vector<int>& seam_path
        );

        // Helper: Find connected components at vertex
        static std::vector<std::set<int>> findVertexComponents(
            int vertex,
            const std::vector<int>& incident_faces,
            const std::vector<Eigen::Vector3i>& faces,
            const std::set<std::pair<int, int>>& seam_edges
        );
    };

    // Boundary analysis utilities
    class BoundaryAnalyzer
    {
    public:
        // Analyze boundary loop properties
        struct BoundaryInfo {
            std::vector<Eigen::Vector3d> points;
            Eigen::Vector3d center;
            Eigen::Vector3d normal;
            double perimeter;
            double area;
            bool is_closed;
        };

        static BoundaryInfo analyzeBoundaryLoop(
            const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<int>& loop
        );

        // Find shortest path connecting two boundaries
        static std::vector<int> findShortestSeamPath(
            const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<Eigen::Vector3i>& faces,
            const std::vector<int>& boundary1,
            const std::vector<int>& boundary2
        );

        // Detect if mesh has exactly two boundary loops
        static bool hasTwoBoundaries(
            const std::vector<Eigen::Vector3i>& faces,
            std::vector<int>& boundary1,
            std::vector<int>& boundary2
        );
    };

} // namespace SprayTrajPlan
