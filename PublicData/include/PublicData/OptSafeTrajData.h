#pragma once
#include "OptSafeTrajData_global.h"
#include <vector>
#include <map>
#include <string>
#include <Eigen/Dense>
#include <TrajData/TrajData.h>

namespace PublicData
{
    // Enumeration for trajectory point generation algorithms
    enum class TrajectoryGenerationMethod {
        Manual,         // Manual generation
        RRT,            // RRT generation
        Tolerance,      // Tolerance-based generation
        Other           // Other generation methods
    };

    // Public trajectory point structure
    struct OptTrajectoryPoint {
        // Pose: Transformation matrix representing position + orientation
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

        // Tolerance ranges
        Eigen::Matrix<double, 3, 2> orient_tolerance;   // Orientation tolerance [min, max]
        Eigen::Matrix<double, 3, 2> pos_tolerance;      // Position tolerance [min, max]

        // Flags
        bool is_adjustable;                             // Adjustable flag
        CoordinateFrame tolerance_frame;                // Tolerance coordinate frame

        // Trajectory point generation method
        TrajectoryGenerationMethod generation_method;   // Generation method identifier (RRT, Tolerance, etc.)

        // Backup points (for RRT generation only, stores poses of multiple candidate points)
        std::vector<Eigen::Matrix4d> backup_points;     // Stores poses of backup points

        // Spraying distance (mm), distance from tool to workpiece surface
        double spray_distance = 0.0;                    // Spraying distance for adjustment based on workpiece target points

        // Constructor
        OptTrajectoryPoint()
            : pose(Eigen::Matrix4d::Identity()),
            pos_tolerance(Eigen::Matrix<double, 3, 2>::Zero()),
            orient_tolerance(Eigen::Matrix<double, 3, 2>::Zero()),
            is_adjustable(true),
            tolerance_frame(CoordinateFrame::World),
            generation_method(TrajectoryGenerationMethod::Manual),
            spray_distance(0.0) {}  // Default generation method: Manual, default spraying distance: 0mm
    };

    using OptTrajectoryPoints = std::map<double, OptTrajectoryPoint>;

    // Public trajectory data class
    class OptSafeTrajData_SHAREDLIB OptSafeTrajData
    {
    public:
        OptSafeTrajData();
        ~OptSafeTrajData();

        // Get trajectory point collection (read-only)
        const OptTrajectoryPoints& getTrajectoryPoints() const { return trajectoryPoints_; }

        // Get trajectory point collection (modifiable)
        OptTrajectoryPoints& getTrajectoryPoints() { return trajectoryPoints_; }

        // Get transformation matrix (read-only)
        const Eigen::Matrix4d& getTransformMatrix() const { return transformMatrix_; }

        // Get transformation matrix (modifiable)
        Eigen::Matrix4d& getTransformMatrix() { return transformMatrix_; }

        // Clear all data
        void clear();

        // Check if valid data exists
        bool hasValidData() const { return !trajectoryPoints_.empty(); }

        // Get number of trajectory points
        size_t getPointCount() const { return trajectoryPoints_.size(); }

        // === Project File Path Access ===

        // Get configuration file path (read-only)
        const std::string& getConfigFilePath() const { return config_file_path_; }

        // Get configuration file path (modifiable)
        std::string& getConfigFilePath() { return config_file_path_; }

        // Get robot1 XML model file path (read-only)
        const std::string& getRobot1XmlPath() const { return robot1_xml_path_; }

        // Get robot1 XML model file path (modifiable)
        std::string& getRobot1XmlPath() { return robot1_xml_path_; }

        // Get robot1 STL file path (read-only)
        const std::string& getRobot1StlPath() const { return robot1_stl_path_; }

        // Get robot1 STL file path (modifiable)
        std::string& getRobot1StlPath() { return robot1_stl_path_; }

        // Get robot2 XML model file path (read-only)
        const std::string& getRobot2XmlPath() const { return robot2_xml_path_; }

        // Get robot2 XML model file path (modifiable)
        std::string& getRobot2XmlPath() { return robot2_xml_path_; }

        // Get robot2 STL file path (read-only)
        const std::string& getRobot2StlPath() const { return robot2_stl_path_; }

        // Get robot2 STL file path (modifiable)
        std::string& getRobot2StlPath() { return robot2_stl_path_; }

        // Get trajectory file path (read-only)
        const std::string& getTrajectoryFilePath() const { return trajectory_file_path_; }

        // Get trajectory file path (modifiable)
        std::string& getTrajectoryFilePath() { return trajectory_file_path_; }

        // === Load Trajectory Data ===
        // Load trajectory data from a TXT file
        // file_path: Full path of the trajectory file
        // Return value: true on success, false on failure
        bool loadFromFile(const std::string& file_path);

        // Load trajectory data directly from a data structure
        // traj_points: Map collection of trajectory points
        // Return value: true on success, false on failure
        bool loadFromData(const std::map<double, OptTrajectoryPoint>& traj_points);

        // === Load Project Configuration ===
        // Load project path information from a config.json file
        // config_file_path: Full path of the config.json file
        // Return value: true on success, false on failure
        // Description: Parses the JSON file, extracts all path information, and saves it to corresponding member variables
        bool loadProjectConfig(const std::string& config_file_path);

    private:
        // Trajectory point collection: uses map, key is timestamp
        OptTrajectoryPoints trajectoryPoints_;

        // Transformation matrix: for conversion between different coordinate frames
        Eigen::Matrix4d transformMatrix_;  // 4x4 transformation matrix

        // === Project File Paths ===
        std::string config_file_path_;        // Configuration file path (config.json)
        std::string robot1_xml_path_;         // Robot1 XML model file path (robot/robot1/robot1.model.xml)
        std::string robot1_stl_path_;         // Robot1 STL file path (robot/robot1/*.STL)
        std::string robot2_xml_path_;         // Robot2 XML model file path (robot/robot2/robot2.model.xml)
        std::string robot2_stl_path_;         // Robot2 STL file path (robot/robot2/*.STL)
        std::string trajectory_file_path_;    // Trajectory file path (Trajectory/trajectory.txt)
    };
}