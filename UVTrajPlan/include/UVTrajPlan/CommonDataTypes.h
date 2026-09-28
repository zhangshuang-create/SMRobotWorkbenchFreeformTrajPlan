#pragma once

#include <vector>
#include <string>
#include <optional>
#include <memory>
#include <map>
#include <Eigen/Dense>

namespace SprayTrajPlan
{
    constexpr double EPS = 1.0e-9;
    constexpr double DEFAULT_TRAJECTORY_SPACING = 30.0;
    constexpr double DEFAULT_TRAJECTORY_SPEED = 300.0;
    constexpr double DEFAULT_SPRAY_DISTANCE = 0.0;

    // Forward declarations
    struct TrajectorySegment;
    struct PrecutSeam;
    struct ObjUVData;
    struct SeamUVSegment;
    struct PlanningFrame;
    struct Optimum;
    struct BoundaryLoop;
    struct Plane;

    // PrecutSeam - Pre-cut seam data structure
    struct PrecutSeam {
        std::vector<int> original_vertex_ids;
        std::vector<std::pair<int, int>> cut_vertex_pairs;
        std::vector<Eigen::Vector3d> xyz;

        // Get seam edges
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getEdges() const;
    };

    // ObjUVData - UV mapping data structure
    struct ObjUVData {
        std::vector<Eigen::Vector3d> xyz;
        std::vector<Eigen::Vector3i> xyz_faces;
        std::vector<Eigen::Vector2d> uv;
        std::vector<Eigen::Vector3i> uv_faces;
    };

    // SeamUVSegment - UV boundary segment representing physical pre-cut seam edge
    struct SeamUVSegment {
        int physical_index;
        Eigen::Vector2d uv_a0;
        Eigen::Vector2d uv_a1;
        Eigen::Vector2d uv_b0;
        Eigen::Vector2d uv_b1;
    };

    // TrajectorySegment - One explicit path event
    struct TrajectorySegment {
        std::string kind;               // Type: "SURFACE_SCAN", "TRANSITION", etc.
        std::vector<Eigen::Vector2d> uv;
        std::vector<Eigen::Vector3d> xyz;
        bool spray_on;
        std::string note;
        std::string region;             // "CENTER", "UPPER", "LOWER"
        std::vector<int> phases;        // Optional phase information
        std::optional<std::map<std::string, double>> geometry_meta;  // Geometric metadata

        TrajectorySegment()
            : spray_on(false) {}
    };

    // PlanningFrame - Planning coordinate frame
    struct PlanningFrame {
        Eigen::Vector3d origin;
        Eigen::Vector3d x_axis;
        Eigen::Vector3d y_axis;
        double waist_width;
        int waist_pair_index;

        PlanningFrame()
            : waist_width(0.0), waist_pair_index(0) {}
    };

    // Optimum - Optimization result
    struct Optimum {
        double spacing;
        std::vector<int> counts;
        std::vector<double> actual_spacings;
        double objective;
        std::vector<double> candidates;
        std::vector<double> objective_values;

        Optimum() : spacing(0.0), objective(0.0) {}
    };

    // BoundaryLoop - Boundary loop structure
    struct BoundaryLoop {
        std::vector<Eigen::Vector3d> points;
        std::vector<int> vertex_indices;
        Eigen::Vector3d center;
        Eigen::Vector3d normal;
        double z_level;
        std::string name;

        BoundaryLoop()
            : center(Eigen::Vector3d::Zero()),
              normal(Eigen::Vector3d::UnitZ()),
              z_level(0.0) {}
    };

    // Plane - Geometric plane
    struct Plane {
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        double d;  // Plane equation coefficient

        Plane()
            : point(Eigen::Vector3d::Zero()),
              normal(Eigen::Vector3d::UnitZ()),
              d(0.0) {}
    };

} // namespace SprayTrajPlan
