#pragma once

#include "CommonDataTypes.h"
#include "OptCutsRunner.h"
#include <vector>
#include <functional>
#include <string>
#include <map>

namespace SprayTrajPlan
{
    // Trajectory generation parameters
    struct TrajectoryParams {
        double target_spacing;
        double sample_step;
        double trajectory_speed;
        double spray_distance;
        double a_max;
        double j_max;
        double arc_center_limit;
        double arc_lock_guard_d;
        double max_seam_spray_distance;
        double arc_speed_loss_pct;

        TrajectoryParams()
            : target_spacing(DEFAULT_TRAJECTORY_SPACING),
              sample_step(0.0),
              trajectory_speed(DEFAULT_TRAJECTORY_SPEED),
              spray_distance(DEFAULT_SPRAY_DISTANCE),
              a_max(1000.0),
              j_max(5000.0),
              arc_center_limit(0.0),
              arc_lock_guard_d(4.0),
              max_seam_spray_distance(78.0),
              arc_speed_loss_pct(0.0) {}
    };

    // Trajectory generation metadata
    struct TrajectoryMetadata {
        std::string mesh_path;
        std::string source_type;
        std::string optcuts_result_path;
        std::string cache_format;
        TrajectoryParams parameter_snapshot;
        double spacing = 0.0;
        double speed = 0.0;
        double spray_distance = 0.0;
        int total_segments = 0;
        double total_length = 0.0;
        std::map<std::string, std::string> additional_info;
    };

    // Main trajectory generator class
    class TrajectoryGenerator
    {
    public:
        TrajectoryGenerator();
        ~TrajectoryGenerator();

        // Generate seam-aware trajectory
        bool generateSeamAwareTrajectory(
            const ObjUVData& uv_data,
            const PrecutSeam& precut_seam,
            const TrajectoryParams& params,
            std::vector<TrajectorySegment>& trajectory,
            PlanningFrame& planning_frame,
            TrajectoryMetadata& metadata
        );

        // Optimize trajectory arcs post-process
        bool optimizeTrajectoryArcs(
            const std::vector<TrajectorySegment>& input_trajectory,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const TrajectoryParams& params,
            std::vector<TrajectorySegment>& output_trajectory
        );

        // Optimize center trajectories with dual blank energy
        bool optimizeCenterTrajectories(
            const std::vector<TrajectorySegment>& input_trajectory,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const TrajectoryParams& params,
            std::vector<TrajectorySegment>& output_trajectory,
            const std::function<void(int, int, const std::vector<TrajectorySegment>&)>& progress_callback = {}
        );

        // Fit quintic B-spline trajectories
        bool fitQuinticBSpline(
            const std::vector<TrajectorySegment>& input_trajectory,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const TrajectoryParams& params,
            bool project_to_mesh,
            std::vector<TrajectorySegment>& output_trajectory
        );

        // Append seam transition
        bool appendSeamTransition(
            std::vector<TrajectorySegment>& trajectory,
            const PrecutSeam& precut_seam,
            double sample_step
        );

    private:
        // Internal helper methods
        bool generateBasicTrajectory(
            const ObjUVData& uv_data,
            double target_spacing,
            double sample_step,
            std::vector<TrajectorySegment>& trajectory
        );

        bool computePlanningFrame(
            const ObjUVData& uv_data,
            PlanningFrame& frame
        );
    };

} // namespace SprayTrajPlan
