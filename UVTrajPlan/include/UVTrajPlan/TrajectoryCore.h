#pragma once

#include "CommonDataTypes.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace SprayTrajPlan
{
    // UV to XYZ coordinate mapper
    class UVToXYZMapper
    {
    public:
        explicit UVToXYZMapper(const ObjUVData& data);
        ~UVToXYZMapper();

        // Map UV points to XYZ coordinates
        std::vector<Eigen::Vector3d> map(
            const std::vector<Eigen::Vector2d>& uv_points,
            double snap_tolerance = -1.0,
            bool bridge_outside = false
        );

        // Map single UV point to XYZ
        Eigen::Vector3d mapSingle(
            const Eigen::Vector2d& uv_point,
            double snap_tolerance = -1.0,
            bool bridge_outside = false
        );

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;

        // Compute barycentric coordinates
        static Eigen::Vector3d computeWeights(
            const Eigen::Vector2d& point,
            const std::vector<Eigen::Vector2d>& triangle
        );

        // Find containing triangle for UV point
        int findContainingTriangle(
            const Eigen::Vector2d& point,
            int start_hint = -1
        ) const;
    };

    // Spacing optimizer
    struct SpacingOptimum {
        double spacing;
        Eigen::Vector3i counts;        // center, upper_ref, lower_ref
        Eigen::Vector3d actual_spacings;
        double objective;
        std::vector<double> candidates;
        std::vector<double> objective_values;
    };

    class SpacingOptimizer
    {
    public:
        // Optimize trajectory spacing
        static SpacingOptimum optimizeSpacing(
            double nominal,
            const Eigen::Vector3d& dimensions  // [center_width, upper_ref, lower_ref]
        );

        // Generate balanced center positions
        static std::pair<std::vector<double>, double> balancedCenterPositions(
            double width,
            int count
        );

        // Generate balanced outer positions
        static std::pair<std::vector<double>, double> balancedOuterPositions(
            double extent,
            int count,
            double waist_x,
            double direction
        );
    };

    // Trajectory sampling utilities
    class TrajectorySampler
    {
    public:
        // Sample scan line in UV space
        static std::vector<Eigen::Vector2d> sampleScanUV(
            double x_value,
            const std::pair<double, double>& interval,
            bool upward,
            double sample_step,
            const PlanningFrame& frame
        );

        // Convert from local planning frame to UV coordinates
        static std::vector<Eigen::Vector2d> fromLocal(
            const std::vector<Eigen::Vector2d>& local_coords,
            const PlanningFrame& frame
        );

        // Convert from UV to local planning frame
        static std::vector<Eigen::Vector2d> toLocal(
            const std::vector<Eigen::Vector2d>& uv_coords,
            const PlanningFrame& frame
        );
    };

    // Seam handling utilities
    class SeamHandler
    {
    public:
        // Project point to seam and find counterpart
        static std::pair<Eigen::Vector2d, Eigen::Vector2d> seamProjectionAndCounterpart(
            const Eigen::Vector2d& point,
            const std::vector<SeamUVSegment>& seam_segments,
            bool from_a
        );

        // Create overtravel segment between two surface scans
        static TrajectorySegment createOvertravelSegment(
            const TrajectorySegment& previous,
            const TrajectorySegment& following,
            double overtravel
        );

        // Rebuild overtravel connectors after scan movement
        static std::vector<TrajectorySegment> rebuildOvertravelConnectors(
            const std::vector<TrajectorySegment>& trajectory,
            double overtravel
        );
    };

} // namespace SprayTrajPlan
