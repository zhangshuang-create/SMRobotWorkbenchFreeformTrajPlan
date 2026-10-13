#pragma once

#include "CommonDataTypes.h"
#include "TrajectoryGenerator.h"
#include <string>
#include <fstream>

namespace SprayTrajPlan
{
    struct ExportTrajectoryPoint {
        Eigen::Vector3d position; Eigen::Quaterniond orientation;
        double velocity{0}, acceleration{0}, time{0}, sprayDistance{0};
        std::size_t segmentIndex{0}; bool sprayEnabled{false}; int pointType{0};
    };
    // Trajectory export utilities
    class TrajectoryExporter
    {
    public:
        static std::vector<ExportTrajectoryPoint> prepareExportTrajectory(
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const std::vector<TrajectorySegment>& trajectory,
            double speed, double spray_distance, double arc_speed_loss_pct);
        // Export trajectory to CSV format
        static bool exportTrajectoryCSV(
            const std::string& csv_path,
            const std::vector<TrajectorySegment>& trajectory,
            const TrajectoryMetadata& metadata
        );

        // Export trajectory metadata to JSON
        static bool exportMetadataJSON(
            const std::string& json_path,
            const TrajectoryMetadata& metadata
        );

        // Export trajectory in PAQ format
        static bool exportPAQTrajectory(
            const std::string& paq_path,
            const std::vector<Eigen::Vector3d>& mesh_vertices,
            const std::vector<Eigen::Vector3i>& mesh_faces,
            const std::vector<TrajectorySegment>& trajectory,
            double speed,
            double spray_distance,
            double arc_speed_loss_pct
        );

        // Export trajectory matrix format
        static bool exportMatrixFormat(
            const std::string& matrix_path,
            const std::vector<TrajectorySegment>& trajectory,
            double speed
        );

    private:
        // Helper: Write CSV header
        static void writeCSVHeader(std::ofstream& file);

        // Helper: Format point for PAQ
        static std::string formatPAQPoint(
            const Eigen::Vector3d& position,
            const Eigen::Vector3d& normal,
            bool spray_on,
            double speed
        );

        // Helper: Compute trajectory statistics
        static void computeStatistics(
            const std::vector<TrajectorySegment>& trajectory,
            double& total_length,
            int& total_points
        );
    };

} // namespace SprayTrajPlan
