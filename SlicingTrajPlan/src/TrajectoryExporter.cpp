#include "TrajectoryExporter.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
namespace ModelTrajPlan
{
    TrajectoryExporter::TrajectoryExporter()
    {
    }

    TrajectoryExporter::~TrajectoryExporter()
    {
    }

    // === Export feature implementations ===


    // =============================================================================
// 1. 导出轨迹文件（补充时间戳）
// =============================================================================
    bool TrajectoryExporter::exportTrajectoryToFile(const std::vector<TrajectoryPoint>& trajectoryPoints,
        const std::string& filename)
    {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                return false;
            }

            // 预先计算时间戳（和你提供的逻辑完全一致）
            std::vector<double> timestamps;
            double current_time = 0.0;
            timestamps.push_back(current_time);

            for (size_t i = 1; i < trajectoryPoints.size(); ++i)
            {
                const auto& prev = trajectoryPoints[i - 1];
                const auto& src = trajectoryPoints[i];

                Eigen::Vector3d p1 = extractPosition(prev.pose);
                Eigen::Vector3d p2 = extractPosition(src.pose);
                double dist = (p2 - p1).norm();

                if (dist > 1e-6)
                {
                    double v = std::max((src.velocity + prev.velocity) * 0.5, 1e-6);
                    double dt = dist / v;
                    current_time += dt;
                }
                else
                {
                    current_time += 1e-6;
                }
                timestamps.push_back(current_time);
            }

            // 导出所有数据，使用计算后的时间戳
            for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
                const auto& point = trajectoryPoints[i];
                Eigen::Vector3d position = extractPosition(point.pose);
                Eigen::Quaterniond quaternion = extractQuaternion(point.pose);

                file << formatNumber(position.x()) << " "
                    << formatNumber(position.y()) << " "
                    << formatNumber(position.z()) << " "
                    << formatNumber(quaternion.w()) << " "
                    << formatNumber(quaternion.x()) << " "
                    << formatNumber(quaternion.y()) << " "
                    << formatNumber(quaternion.z()) << " "
                    << formatNumber(point.velocity) << " "
                    << formatNumber(point.acceleration) << " "
                    << formatNumber(timestamps[i], 6) << " "
                    << point.point_type << " "
                    << formatNumber(point.spray_distance) << std::endl;
            }

            file.close();
            return true;
        }
        catch (const std::exception& e) {
            return false;
        }
    }

    // =============================================================================
    // 2. 导出Excel（简单版：新增 Timestamp 列）
    // =============================================================================
    bool TrajectoryExporter::exportToExcel(const std::vector<TrajectoryPoint>& trajectoryPoints,
        const std::string& filename)
    {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                return false;
            }

            // 预先计算时间戳
            std::vector<double> timestamps;
            double current_time = 0.0;
            timestamps.push_back(current_time);

            for (size_t i = 1; i < trajectoryPoints.size(); ++i)
            {
                const auto& prev = trajectoryPoints[i - 1];
                const auto& src = trajectoryPoints[i];
                Eigen::Vector3d p1 = extractPosition(prev.pose);
                Eigen::Vector3d p2 = extractPosition(src.pose);
                double dist = (p2 - p1).norm();

                if (dist > 1e-6)
                {
                    double v = std::max((src.velocity + prev.velocity) * 0.5, 1e-6);
                    double dt = dist / v;
                    current_time += dt;
                }
                else
                {
                    current_time += 1e-6;
                }
                timestamps.push_back(current_time);
            }

            // CSV header：增加 Timestamp
            file << "X,Y,Z,Roll,Pitch,Yaw,SprayDistance,Timestamp\n";

            for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
                const auto& point = trajectoryPoints[i];
                Eigen::Vector3d position = extractPosition(point.pose);
                Eigen::Vector3d eulerAngles = extractEulerAngles(point.pose);

                file << formatNumber(position.x()) << ","
                    << formatNumber(position.y()) << ","
                    << formatNumber(position.z()) << ","
                    << formatNumber(eulerAngles.x()) << ","
                    << formatNumber(eulerAngles.y()) << ","
                    << formatNumber(eulerAngles.z()) << ","
                    << formatNumber(point.spray_distance) << ","
                    << formatNumber(timestamps[i], 6) << "\n";
            }

            file.close();
            return true;
        }
        catch (const std::exception& e) {
            return false;
        }
    }

    // =============================================================================
    // 3. 导出Excel完整版：替换原有时间戳为计算值
    // =============================================================================
    bool TrajectoryExporter::exportToExcelWithVelocity(const std::vector<TrajectoryPoint>& trajectoryPoints,
        const std::string& filename)
    {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                return false;
            }

            // 预先计算时间戳
            std::vector<double> timestamps;
            double current_time = 0.0;
            timestamps.push_back(current_time);

            for (size_t i = 1; i < trajectoryPoints.size(); ++i)
            {
                const auto& prev = trajectoryPoints[i - 1];
                const auto& src = trajectoryPoints[i];
                Eigen::Vector3d p1 = extractPosition(prev.pose);
                Eigen::Vector3d p2 = extractPosition(src.pose);
                double dist = (p2 - p1).norm();

                if (dist > 1e-6)
                {
                    double v = std::max((src.velocity + prev.velocity) * 0.5, 1e-6);
                    double dt = dist / v;
                    current_time += dt;
                }
                else
                {
                    current_time += 1e-6;
                }
                timestamps.push_back(current_time);
            }

            file << "Index,X,Y,Z,Roll,Pitch,Yaw,Qw,Qx,Qy,Qz,Velocity,Acceleration,Timestamp,PointType,SprayDistance\n";

            for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
                const auto& point = trajectoryPoints[i];
                Eigen::Vector3d position = extractPosition(point.pose);
                Eigen::Vector3d eulerAngles = extractEulerAngles(point.pose);
                Eigen::Quaterniond quaternion = extractQuaternion(point.pose);

                file << (i + 1) << ","
                    << formatNumber(position.x()) << ","
                    << formatNumber(position.y()) << ","
                    << formatNumber(position.z()) << ","
                    << formatNumber(eulerAngles.x()) << ","
                    << formatNumber(eulerAngles.y()) << ","
                    << formatNumber(eulerAngles.z()) << ","
                    << formatNumber(quaternion.w()) << ","
                    << formatNumber(quaternion.x()) << ","
                    << formatNumber(quaternion.y()) << ","
                    << formatNumber(quaternion.z()) << ","
                    << formatNumber(point.velocity) << ","
                    << formatNumber(point.acceleration) << ","
                    << formatNumber(timestamps[i], 6) << ","  // 使用计算时间戳
                    << point.point_type << ","
                    << formatNumber(point.spray_distance) << "\n";
            }

            file.close();
            return true;
        }
        catch (const std::exception& e) {
            return false;
        }
    }

    bool TrajectoryExporter::exportFANUCProgram(const std::vector<TrajectoryPoint>& trajectoryPoints,
        const std::string& filename)
    {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                return false;
            }

            // FANUC program header
            file << "/PROG  GJ2\n";
            file << "/ATTR\n";
            file << "OWNER    = MNEDITOR;\n";
            file << "COMMENT  = \"Spray Trajectory\";\n";
            file << "PROG_SIZE  = 0;\n";
            file << "VERSION  = 0;\n";
            file << "/MN\n";
            file << "   1:  UFRAME_NUM=2 ;\n";
            file << "   2:  UTOOL_NUM=1 ;\n";

            // Export trajectory points - includes position, orientation, and velocity
            for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
                const auto& point = trajectoryPoints[i];
                // Extract position from homogeneous transform matrix
                Eigen::Vector3d position = extractPosition(point.pose);
                // Extract Euler angles (FANUC uses Euler angles for orientation)
                Eigen::Vector3d eulerAngles = extractEulerAngles(point.pose);

                // FANUC format: position (mm) + orientation (deg) + velocity (mm/s)
                file << "   " << (i + 3) << ":  L P[" << (i + 1) << "] "
                    << formatNumber(position.x()) << "mm "
                    << formatNumber(position.y()) << "mm "
                    << formatNumber(position.z()) << "mm "
                    << formatNumber(eulerAngles.x()) << "deg "
                    << formatNumber(eulerAngles.y()) << "deg "
                    << formatNumber(eulerAngles.z()) << "deg"
                    << " " << formatNumber(point.velocity) << "mm/sec";

                // Add point-type annotation
                if (point.point_type == 1) {
                    file << " ; Key point";
                }
                else if (point.point_type == 2) {
                    file << " ; Stop point";
                }
                file << "\n";
            }

            file << "/POS\n";
            file << "/END\n";

            file.close();
            return true;

        }
        catch (const std::exception& e) {
            return false;
        }
    }

    bool TrajectoryExporter::exportABBProgram(const std::vector<TrajectoryPoint>& trajectoryPoints,
        const std::string& txtFilename,
        const std::string& modFilename)
    {
        try {
            std::ofstream file(modFilename);
            if (!file.is_open()) {
                return false;
            }

            // ABB program header
            file << "MODULE MainModule\n";
            file << "  ! Auto-generated spray trajectory program\n";
            file << "  ! Generated points: " << trajectoryPoints.size() << "\n\n";

            // Step 1: define all robtargets (must be defined outside PROC)
            for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
                const auto& point = trajectoryPoints[i];
                // Extract position from homogeneous transform matrix
                Eigen::Vector3d position = extractPosition(point.pose);
                // Extract quaternion from homogeneous transform matrix (ABB uses quaternion orientation)
                Eigen::Quaterniond quaternion = extractQuaternion(point.pose);

                // ABB robtarget format: [position, quaternion, config data, external axes]
                file << "  CONST robtarget p" << (i + 1) << " := ["
                    << "[" << formatNumber(position.x()) << "," << formatNumber(position.y()) << "," << formatNumber(position.z()) << "],"
                    << "[" << formatNumber(quaternion.w()) << "," << formatNumber(quaternion.x()) << ","
                    << formatNumber(quaternion.y()) << "," << formatNumber(quaternion.z()) << "],"  // Actual quaternion
                    << "[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];";

                // Add point-type annotation
                if (point.point_type == 1) {
                    file << "  ! Key point";
                }
                else if (point.point_type == 2) {
                    file << "  ! Stop point";
                }
                file << "\n";
            }

            // Step 2: define main procedure
            file << "\n  PROC main()\n";

            // Step 3: output motion instructions
            for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
                const auto& point = trajectoryPoints[i];

                // Use actual speed (convert mm/s to ABB v-speed format, where v100 = 100 mm/s)
                int speedValue = static_cast<int>(point.velocity);
                if (speedValue < 1) speedValue = 50;  // Default speed

                std::string moveType = "MoveL";
                const bool mustStop = point.point_type == 2 ||
                    i + 1 == trajectoryPoints.size();
                // z10 keeps continuous motion for the roughly 10 mm samples on one
                // trajectory. The final and explicit stop targets are exact.
                std::string zoneType = mustStop ? "fine" : "z10";

                file << "    " << moveType << " p" << (i + 1) << ", v" << speedValue << ", "
                    << zoneType << ", tool0;\n";
            }

            file << "  ENDPROC\n";
            file << "ENDMODULE\n";

            file.close();
            return true;

        }
        catch (const std::exception& e) {
            return false;
        }
    }


    // === Internal helper function implementations ===

    std::string TrajectoryExporter::formatNumber(double value, int precision)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        return oss.str();
    }

    Eigen::Vector3d TrajectoryExporter::extractPosition(const Eigen::Matrix4d& pose)
    {
        return pose.block<3, 1>(0, 3);
    }

    Eigen::Vector3d TrajectoryExporter::extractEulerAngles(const Eigen::Matrix4d& pose)
    {
        // Extract Euler angles from rotation matrix (ZYX order - Roll, Pitch, Yaw)
        Eigen::Matrix3d rotationMatrix = pose.block<3, 3>(0, 0);
        Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(2, 1, 0); // ZYX order

        const double PI = 3.14159265358979323846;
        double yaw = eulerAngles[0] * 180.0 / PI;    // Z-axis rotation (Yaw)
        double pitch = eulerAngles[1] * 180.0 / PI;  // Y-axis rotation (Pitch)
        double roll = eulerAngles[2] * 180.0 / PI;   // X-axis rotation (Roll)

        return Eigen::Vector3d(roll, pitch, yaw);
    }

    Eigen::Quaterniond TrajectoryExporter::extractQuaternion(const Eigen::Matrix4d& pose)
    {
        // Extract quaternion from rotation matrix
        Eigen::Matrix3d rotationMatrix = pose.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotationMatrix);
        return quaternion;
    }

}