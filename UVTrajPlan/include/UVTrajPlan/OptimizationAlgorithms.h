#pragma once

#include "CommonDataTypes.h"
#include "TrajectoryCore.h"
#include <Eigen/Dense>
#include <functional>
#include <vector>

namespace SprayTrajPlan
{
    // Energy-based optimization for center trajectories
    class EnergyOptimizer
    {
    public:
        struct OptimizationParams {
            double spacing;
            double sample_step;
            double speed;
            double spray_distance;
            int max_iterations;
            double convergence_threshold;
            double blank_weight;
            double smoothness_weight;

            OptimizationParams()
                : spacing(30.0),
                  sample_step(7.5),
                  speed(DEFAULT_TRAJECTORY_SPEED),
                  spray_distance(DEFAULT_SPRAY_DISTANCE),
                  // Keep the Python OptAeigenTest default.  One visible
                  // iteration is a full CENTER -> LOWER -> UPPER cycle.
                  max_iterations(140),
                  convergence_threshold(1e-4),
                  blank_weight(1.0),
                  smoothness_weight(0.5) {}
        };

        // Optimize center trajectories with dual blank energy
        static bool optimizeCenterTrajectoriesDualBlankEnergy(
            const std::vector<TrajectorySegment>& input_trajectory,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const OptimizationParams& params,
            std::vector<TrajectorySegment>& output_trajectory,
            const std::function<void(int, int, const std::vector<TrajectorySegment>&)>& progress_callback = {}
        );

    private:
        // Extract logical surface tracks
        static std::vector<std::map<std::string, std::vector<Eigen::Vector3d>>>
            extractLogicalSurfaceTracks(const std::vector<TrajectorySegment>& trajectory);

        // Compute blank energy for a point configuration
        static double computeBlankEnergy(
            const std::vector<std::vector<Eigen::Vector3d>>& tracks,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            double spacing
        );

        // Compute smoothness energy
        static double computeSmoothnessEnergy(
            const std::vector<std::vector<Eigen::Vector3d>>& tracks
        );

        // Optimize single track position
        static std::vector<Eigen::Vector3d> optimizeTrackPosition(
            const std::vector<Eigen::Vector3d>& track,
            const std::vector<std::vector<Eigen::Vector3d>>& neighbor_tracks,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const OptimizationParams& params
        );
    };

    // Arc optimization
    class ArcOptimizer
    {
    public:
        struct ArcParams {
            double speed;
            double a_max;
            double j_max;
            double spacing;
            double sample_step;
            double center_boundary_limit;
            double arc_lock_guard_d;

            ArcParams()
                : speed(300.0),
                  a_max(1000.0),
                  j_max(5000.0),
                  spacing(30.0),
                  sample_step(7.5),
                  center_boundary_limit(0.0),
                  arc_lock_guard_d(2.0) {}
        };

        // Optimize trajectory arcs for smoother motion
        static bool optimizeTrajectoryArcs(
            const std::vector<TrajectorySegment>& input_trajectory,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const ArcParams& params,
            std::vector<TrajectorySegment>& output_trajectory
        );

        // Mirrors OptAeigenTest.py: the V paths are fitted as superellipses
        // before any kinematic tangent-circle fillet is considered.
        static bool fitSuperellipseTrajectories(
            const std::vector<TrajectorySegment>& input_trajectory,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const ArcParams& params,
            std::vector<TrajectorySegment>& output_trajectory
        );

        static bool applyTangentCircleFillets(
            const std::vector<TrajectorySegment>& input_trajectory,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const ArcParams& params,
            std::vector<TrajectorySegment>& output_trajectory
        );

    private:
        // Detect sharp corners that need arc smoothing
        static std::vector<int> detectSharpCorners(
            const std::vector<Eigen::Vector3d>& points,
            double angle_threshold
        );

        // Fit circular arc to corner
        static std::vector<Eigen::Vector3d> fitCircularArc(
            const Eigen::Vector3d& p0,
            const Eigen::Vector3d& p1,
            const Eigen::Vector3d& p2,
            double max_deviation
        );

        // Check kinematic constraints
        static bool checkKinematicConstraints(
            const std::vector<Eigen::Vector3d>& points,
            double speed,
            double a_max,
            double j_max
        );
    };

    // B-spline fitting
    class BSplineFitter
    {
    public:
        struct SplineParams {
            int degree;              // Spline degree (typically 5 for quintic)
            double smoothness;       // Smoothness parameter
            bool project_to_mesh;    // Project result to mesh surface

            SplineParams()
                : degree(5),
                  smoothness(0.1),
                  project_to_mesh(false) {}
        };

        // Fit quintic B-spline to trajectory
        static bool fitQuinticBSpline(
            const std::vector<TrajectorySegment>& input_trajectory,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const SplineParams& params,
            std::vector<TrajectorySegment>& output_trajectory
        );

    private:
        // Compute B-spline basis functions
        static double bSplineBasis(int i, int k, double t, const std::vector<double>& knots);

        // Fit B-spline to points
        static std::vector<Eigen::Vector3d> fitBSpline(
            const std::vector<Eigen::Vector3d>& points,
            int degree,
            int num_control_points
        );

        // Evaluate B-spline at parameters
        static std::vector<Eigen::Vector3d> evaluateBSpline(
            const std::vector<Eigen::Vector3d>& control_points,
            const std::vector<double>& knots,
            int degree,
            int num_samples
        );
    };

} // namespace SprayTrajPlan
