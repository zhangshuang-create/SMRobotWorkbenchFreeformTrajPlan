#pragma once

#include <SlicingTrajPlan/GeneralData.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <SlicingTrajPlan/BurTrajPlan_global.h>
namespace ModelTrajPlan
{
    struct FanucTcpDefinition
    {
        bool enabled = false;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
        double p = 0.0;
        double r = 0.0;
    };

    struct AbbTcpDefinition
    {
        bool enabled = false;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double q1 = 1.0;
        double q2 = 0.0;
        double q3 = 0.0;
        double q4 = 0.0;
    };

    /**
     * @brief Trajectory exporter class - exports trajectory data in multiple formats
     *
     * Supported export formats:
     * - Generic trajectory file (.txt)
     * - FANUC robot program (.ls)
     * - ABB robot program (.mod)
     * - Excel/CSV file (.csv)
     */
    class BurTrajPlan_SHAREDLIB TrajectoryExporter
    {
    public:
        TrajectoryExporter();
        ~TrajectoryExporter();

        // === Export features ===

        /**
         * @brief Export trajectory to generic file format
         * @param trajectoryPoints Trajectory point data
         * @param filename Output filename
         * @return Whether export succeeds
         */
        static bool exportTrajectoryToFile(const std::vector<TrajectoryPoint>& trajectoryPoints,
            const std::string& filename);

        /**
         * @brief Export FANUC robot program
         * @param trajectoryPoints Trajectory point data
         * @param filename Output filename (.ls)
         * @return Whether export succeeds
         */
        static bool exportFANUCProgram(const std::vector<TrajectoryPoint>& trajectoryPoints,
            const std::string& filename, const FanucTcpDefinition& tcp = {});

        /**
         * @brief Export ABB robot program
         * @param trajectoryPoints Trajectory point data
         * @param txtFilename Text filename (fallback)
         * @param modFilename MOD filename (.mod)
         * @return Whether export succeeds
         */
        static bool exportABBProgram(const std::vector<TrajectoryPoint>& trajectoryPoints,
            const std::string& txtFilename,
            const std::string& modFilename, const AbbTcpDefinition& tcp = {});

        /**
         * @brief Export to Excel/CSV format (basic - position and Euler angles only)
         * @param trajectoryPoints Trajectory point data
         * @param filename Output filename (.csv)
         * @return Whether export succeeds
         */
        static bool exportToExcel(const std::vector<TrajectoryPoint>& trajectoryPoints,
            const std::string& filename);

        /**
         * @brief Export to Excel/CSV format (full - includes position, orientation, velocity, acceleration, timestamp)
         * @param trajectoryPoints Trajectory point data
         * @param filename Output filename (.csv)
         * @return Whether export succeeds
         */
        static bool exportToExcelWithVelocity(const std::vector<TrajectoryPoint>& trajectoryPoints,
            const std::string& filename);

    private:
        // === Internal helper functions ===

        /**
         * @brief Format numeric output
         * @param value Numeric value
         * @param precision Precision
         * @return Formatted string
         */
        static std::string formatNumber(double value, int precision = 3);

        /**
         * @brief Extract position from homogeneous transform matrix
         * @param pose Homogeneous transform matrix
         * @return Position vector
         */
        static Eigen::Vector3d extractPosition(const Eigen::Matrix4d& pose);

        /**
         * @brief Extract Euler angles from homogeneous transform matrix
         * @param pose Homogeneous transform matrix
         * @return Euler angles (Roll, Pitch, Yaw)
         */
        static Eigen::Vector3d extractEulerAngles(const Eigen::Matrix4d& pose);

        /**
         * @brief Extract quaternion from homogeneous transform matrix
         * @param pose Homogeneous transform matrix
         * @return Quaternion (w, x, y, z)
         */
        static Eigen::Quaterniond extractQuaternion(const Eigen::Matrix4d& pose);
    };
}
