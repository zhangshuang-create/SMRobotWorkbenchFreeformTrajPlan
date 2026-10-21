#include <SlicingTrajPlan/TrajectoryExporter.h>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <cctype>
#include <filesystem>

namespace
{
    std::string fanucProgramName(const std::string& filename)
    {
        std::string name = std::filesystem::path(filename).stem().string();
        std::string sanitized;
        sanitized.reserve(name.size());
        for(const unsigned char character : name) {
            if(std::isalnum(character)) {
                sanitized.push_back(static_cast<char>(std::toupper(character)));
            } else {
                sanitized.push_back('_');
            }
        }
        if(sanitized.empty()) {
            sanitized = "TRAJECT";
        }
        if(!std::isalpha(static_cast<unsigned char>(sanitized.front()))) {
            sanitized.insert(sanitized.begin(), 'P');
        }
        sanitized.resize(std::min<std::size_t>(sanitized.size(), 8));
        return sanitized;
    }
}

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
// 1. �����켣�ļ�������ʱ�����
// =============================================================================
    bool TrajectoryExporter::exportTrajectoryToFile(const std::vector<TrajectoryPoint>& trajectoryPoints,
        const std::string& filename)
    {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                return false;
            }

            // Ԥ�ȼ���ʱ����������ṩ���߼���ȫһ�£�
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

            // �����������ݣ�ʹ�ü�����ʱ���
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
    // 2. ����Excel���򵥰棺���� Timestamp �У�
    // =============================================================================
    bool TrajectoryExporter::exportToExcel(const std::vector<TrajectoryPoint>& trajectoryPoints,
        const std::string& filename)
    {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                return false;
            }

            // Ԥ�ȼ���ʱ���
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

            // CSV header������ Timestamp
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
    // 3. ����Excel�����棺�滻ԭ��ʱ���Ϊ����ֵ
    // =============================================================================
    bool TrajectoryExporter::exportToExcelWithVelocity(const std::vector<TrajectoryPoint>& trajectoryPoints,
        const std::string& filename)
    {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                return false;
            }

            // Ԥ�ȼ���ʱ���
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
                    << formatNumber(timestamps[i], 6) << ","  // ʹ�ü���ʱ���
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
        const std::string& filename, const FanucTcpDefinition& tcp)
    {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                return false;
            }

            const std::string programName = fanucProgramName(filename);
            file << "/PROG  " << programName << "\n"
                 << "/ATTR\n"
                 << "OWNER\t\t= MNEDITOR;\n"
                 << "COMMENT\t\t= \"Spray Trajectory\";\n"
                 << "PROG_SIZE\t= 0;\n"
                 << "VERSION\t\t= 0;\n"
                 << "LINE_COUNT\t= " << trajectoryPoints.size() + std::count_if(trajectoryPoints.begin(), trajectoryPoints.end(),
                     [](const TrajectoryPoint& p) { return p.point_type == 1 || p.point_type == 2; })
                     + (tcp.enabled ? 10 : 2) << ";\n"
                 << "MEMORY_SIZE\t= 0;\n"
                 << "PROTECT\t\t= READ_WRITE;\n"
                 << "DEFAULT_GROUP\t= 1,*,*,*,*;\n"
                 << "/MN\n"
                 << "   1:  UFRAME_NUM=2 ;\n"
                 << "   2:  UTOOL_NUM=1 ;\n";

            std::size_t firstMotionLine = 3;
            if(tcp.enabled) {
                file << "   3:  PR[99]=LPOS ;\n"
                     << "   4:  PR[99,1]=(" << formatNumber(tcp.x) << ") ;\n"
                     << "   5:  PR[99,2]=(" << formatNumber(tcp.y) << ") ;\n"
                     << "   6:  PR[99,3]=(" << formatNumber(tcp.z) << ") ;\n"
                     << "   7:  PR[99,4]=(" << formatNumber(tcp.w) << ") ;\n"
                     << "   8:  PR[99,5]=(" << formatNumber(tcp.p) << ") ;\n"
                     << "   9:  PR[99,6]=(" << formatNumber(tcp.r) << ") ;\n"
                     << "  10:  UTOOL[1]=PR[99] ;\n";
                firstMotionLine = 11;
            }

            std::size_t motionLine = firstMotionLine;
            for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
                const auto& point = trajectoryPoints[i];
                if(point.point_type == 1 || point.point_type == 2) {
                    file << "   " << motionLine++ << ":  ! P[" << (i + 1) << "] "
                        << (point.point_type == 1 ? "Boundary point" : "Transition offset point") << " ;\n";
                }
                const int speed = std::max(1, static_cast<int>(std::lround(
                    point.velocity > 0.0 ? point.velocity : 50.0)));
                if(i == 0) {
                    // FANUC joint motion uses percentage speed, not linear mm/sec.
                    file << "   " << motionLine++ << ":  J P[1] 10% CNT100    ;\n";
                } else {
                    file << "   " << motionLine++ << ":  L P[" << (i + 1) << "] "
                        << speed << "mm/sec CNT100    ;\n";
                }
            }

            file << "/POS\n";
            for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
                const auto& point = trajectoryPoints[i];
                const Eigen::Vector3d position = extractPosition(point.pose);
                const Eigen::Vector3d eulerAngles = extractEulerAngles(point.pose);
                file << "P[" << (i + 1) << "]{\n"
                     << "   GP1:\n"
                     << "\tUF : 2, UT : 1,\t\tCONFIG : 'N U T, 0, 0, 0',\n"
                     << "\tX = " << formatNumber(position.x()) << "  mm,\tY = "
                     << formatNumber(position.y()) << "  mm,\tZ = "
                     << formatNumber(position.z()) << "  mm,\n"
                     << "\tW = " << formatNumber(eulerAngles.x()) << " deg,\tP = "
                     << formatNumber(eulerAngles.y()) << " deg,\tR = "
                     << formatNumber(eulerAngles.z()) << " deg\n};\n";
            }
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
        const std::string& modFilename, const AbbTcpDefinition& tcp)
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
            if(tcp.enabled) {
                file << "  PERS tooldata toolTCP := [TRUE,[["
                     << formatNumber(tcp.x) << "," << formatNumber(tcp.y) << "," << formatNumber(tcp.z)
                     << "],[" << formatNumber(tcp.q1) << "," << formatNumber(tcp.q2) << ","
                     << formatNumber(tcp.q3) << "," << formatNumber(tcp.q4)
                     << "]],[0,[0,0,0],[1,0,0,0],0,0,0]];\n\n";
            }

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
                    file << "  ! Boundary point";
                }
                else if (point.point_type == 2) {
                    file << "  ! Transition offset point";
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

                std::string moveType = i == 0 ? "MoveJ" : "MoveL";
                // All exported samples use the requested full blending policy.
                // trajectory. The first and final targets are exact; type 2 is an offset, not a stop.
                std::string zoneType = "z100";

                file << "    " << moveType << " p" << (i + 1) << ", v" << speedValue << ", "
                    << zoneType << ", " << (tcp.enabled ? "toolTCP" : "tool0") << ";";
                if(point.point_type == 1) file << " ! Boundary point";
                else if(point.point_type == 2) file << " ! Transition offset point";
                file << "\n";
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
