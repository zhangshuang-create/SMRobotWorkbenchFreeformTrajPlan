#include <TrajData/TrajData.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <vector>
#ifdef _WIN32
#include <io.h>
#include <direct.h>
#include <windows.h>
#else
#include <unistd.h>
#include <dirent.h>
#endif

namespace PublicData
{

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    TrajData::TrajData()
        : transformMatrix_(Eigen::Matrix4d::Identity()),
        trajectory_file_path_("")
    {
    }

    TrajData::~TrajData()
    {
    }

    void TrajData::clear()
    {
        trajectoryPoints_.clear();
        spiralTrajectoryPoints_.clear();
        transformMatrix_.setIdentity();
        trajectory_file_path_.clear();
    }

    bool TrajData::loadFromFile(const std::string& file_path,
        TrajectoryPoints& traj_points)
    {
        std::ifstream fin(file_path);
        if (!fin.is_open())
        {
            std::cerr << "[TrajData] Error: Cannot open file: "
                << file_path << std::endl;
            return false;
        }

        // Clear internal data and external container
        clear();
        traj_points.clear();

        std::string line;
        double lastTimestamp = -1.0;
        Eigen::Vector3d lastPosition = Eigen::Vector3d::Zero();
        bool hasLastPosition = false;

        while (std::getline(fin, line))
        {
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#')
                continue;

            std::istringstream iss(line);
            std::vector<double> values;
            double value{ 0.0 };
            while (iss >> value)
            {
                values.push_back(value);
            }

            double timestamp{ 0.0 };
            double x{ 0.0 }, y{ 0.0 }, z{ 0.0 };
            double qw{ 1.0 }, qx{ 0.0 }, qy{ 0.0 }, qz{ 0.0 };
            double spray_distance{ 0.0 };
            double velocity{ 0.0 };

            if (values.size() >= 12)
            {
                // PAQ format:
                // x y z qw qx qy qz velocity acceleration timestamp point_type spray_distance
                x = values[0];
                y = values[1];
                z = values[2];
                qw = values[3];
                qx = values[4];
                qy = values[5];
                qz = values[6];
                velocity = values[7];
                timestamp = values[9];
                spray_distance = values[11];
            }
            else if (values.size() >= 9)
            {
                // Legacy public trajectory format:
                // timestamp x y z qw qx qy qz spray_distance
                timestamp = values[0];
                x = values[1];
                y = values[2];
                z = values[3];
                qw = values[4];
                qx = values[5];
                qy = values[6];
                qz = values[7];
                spray_distance = values[8];
            }
            else
            {
                std::cerr << "[TrajData] Parse error: " << line << std::endl;
                continue;
            }

            if (!std::isfinite(timestamp) || timestamp <= lastTimestamp)
            {
                const Eigen::Vector3d currentPosition(x, y, z);
                if (hasLastPosition && velocity > 1e-6)
                {
                    timestamp = lastTimestamp + (currentPosition - lastPosition).norm() / velocity;
                }
                else
                {
                    timestamp = lastTimestamp + 1e-6;
                }
            }

            TrajectoryPoint pt;

            // Pose
            pt.pose.setIdentity();
            pt.pose(0, 3) = x;
            pt.pose(1, 3) = y;
            pt.pose(2, 3) = z;

            Eigen::Quaterniond q(qw, qx, qy, qz);
            q.normalize();
            pt.pose.block<3, 3>(0, 0) = q.toRotationMatrix();

            // Spray distance
            pt.spray_distance = spray_distance;
            pt.velocity = velocity;

            // ===== Key Points =====
            // 1. Store in internal data
            trajectoryPoints_[timestamp] = pt;

            // 2. Return to external caller
            traj_points[timestamp] = pt;

            lastTimestamp = timestamp;
            lastPosition = Eigen::Vector3d(x, y, z);
            hasLastPosition = true;
        }

        fin.close();

        std::cout << "[TrajData] Loaded "
            << trajectoryPoints_.size()
            << " points from file: " << file_path << std::endl;

        return !trajectoryPoints_.empty();
    }

    bool TrajData::loadFromData(TrajectoryPoints& traj_points)
    {
        // The caller supplies the new canonical public trajectory.  The old
        // direction silently discarded imported and transformed trajectories.
        trajectoryPoints_ = traj_points;
        spiralTrajectoryPoints_.clear();
        interpolatedPoints_.clear();
        return !trajectoryPoints_.empty();
    }

    // Helper function: Get directory containing the file
    static std::string getFileDirectory(const std::string& file_path)
    {
        size_t lastSlash = file_path.find_last_of("/\\");
        if (lastSlash != std::string::npos) {
            return file_path.substr(0, lastSlash + 1);
        }
        return "./";
    }

}
