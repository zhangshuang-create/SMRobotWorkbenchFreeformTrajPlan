#include <TrajOptimization/TrajectoryPlaybackController.h>
#include <CollisionDetection/CollisionDetection.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace ModelTrajPlan
{
    namespace
    {
        constexpr double kDefaultSpeed = 50.0;

        bool parseLineToDoubles(const std::string& line, std::vector<double>& values)
        {
            values.clear();
            std::string normalized = line;
            std::replace(normalized.begin(), normalized.end(), ',', ' ');
            std::replace(normalized.begin(), normalized.end(), ';', ' ');

            std::istringstream stream(normalized);
            std::string token;
            while (stream >> token) {
                try {
                    values.push_back(std::stod(token));
                }
                catch (...) {
                    return false;
                }
            }

            return !values.empty();
        }

        Eigen::Matrix4d makePoseMatrix(
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation)
        {
            Eigen::Quaterniond normalized = orientation;
            normalized.normalize();

            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose.block<3, 3>(0, 0) = normalized.toRotationMatrix();
            pose.block<3, 1>(0, 3) = position;
            return pose;
        }
    }

    bool TrajectoryPlaybackController::loadPaqFile(
        const std::string& filePath,
        std::string* errorMessage)
    {
        clear();

        std::ifstream file(filePath);
        if (!file.is_open()) {
            if (errorMessage) {
                *errorMessage = "Failed to open trajectory file: " + filePath;
            }
            return false;
        }

        std::string line;
        std::vector<double> values;
        int lineNumber = 0;
        while (std::getline(file, line)) {
            ++lineNumber;
            if (line.empty() || line[0] == '#' || line[0] == '%') {
                continue;
            }

            if (!parseLineToDoubles(line, values)) {
                if (errorMessage) {
                    *errorMessage = "Invalid numeric value at line " + std::to_string(lineNumber);
                }
                return false;
            }

            if (values.size() < 7) {
                continue;
            }

            PlaybackPoint point;
            point.position = Eigen::Vector3d(values[0], values[1], values[2]);

            // new_PAQ.txt format: x y z qw qx qy qz velocity acceleration timestamp point_type spray_distance
            point.orientation = Eigen::Quaterniond(values[3], values[4], values[5], values[6]);
            if (point.orientation.norm() < 1e-9) {
                point.orientation = Eigen::Quaterniond::Identity();
            }
            else {
                point.orientation.normalize();
            }

            point.speed = values.size() >= 8 && values[7] > 1e-6 ? values[7] : kDefaultSpeed;
            point.timestamp = values.size() >= 10 ? values[9] : 0.0;
            point.pointType = values.size() >= 11 ? static_cast<int>(values[10]) : 0;
            point.sprayDistance = values.size() >= 12 ? values[11] : 0.0;
            points_.push_back(point);
        }

        reset();
        if (points_.size() < 2) {
            if (errorMessage) {
                *errorMessage = "Trajectory playback needs at least 2 valid points.";
            }
            clear();
            return false;
        }

        return true;
    }


    bool TrajectoryPlaybackController::loadOptimizedTrajectory(const TrajOptimizationData& data)
    {
        clear();
        if (data.tolerancedTrajectory_.empty()) {
            return false;
        }

        points_.reserve(data.tolerancedTrajectory_.size());
        for (const auto& item : data.tolerancedTrajectory_) {
            const TolerancedTrajectoryPoint& source = item.second;
            PlaybackPoint point;
            point.position = source.pose.block<3, 1>(0, 3);
            point.orientation = Eigen::Quaterniond(source.pose.block<3, 3>(0, 0));
            point.orientation.normalize();
            point.speed = 0.5 * (source.vel_range(0) + source.vel_range(1));
            if (point.speed < 1e-6) point.speed = kDefaultSpeed;
            point.timestamp = item.first;
            point.sprayDistance = source.spray_distance;
            points_.push_back(point);
        }
        return !points_.empty();
    }
    void TrajectoryPlaybackController::clear()
    {
        points_.clear();
        reset();
    }

    void TrajectoryPlaybackController::reset()
    {
        currentSegment_ = 0;
        segmentU_ = 0.0;
    }

    void TrajectoryPlaybackController::setSpeedScale(double scale)
    {
        speedScale_ = std::max(0.05, scale);
    }

    void TrajectoryPlaybackController::setRobotIndex(unsigned int robotIndex)
    {
        robotIndex_ = robotIndex;
    }

    bool TrajectoryPlaybackController::stepByTime(TrajOptimizationData& data, double deltaSeconds)
    {
        if (!isLoaded()) {
            return false;
        }

        if (isFinished()) {
            return applyCurrentPose(data);
        }

        deltaSeconds = std::max(0.0, deltaSeconds);

        while (deltaSeconds > 0.0 && currentSegment_ < points_.size() - 1) {
            const PlaybackPoint& start = points_[currentSegment_];
            const PlaybackPoint& end = points_[currentSegment_ + 1];

            double segmentDistance = (end.position - start.position).norm();
            if (segmentDistance < 1e-6) {
                segmentDistance = 1e-6;
            }

            double segmentSpeed = 0.5 * (start.speed + end.speed);
            if (segmentSpeed < 1e-6) {
                segmentSpeed = kDefaultSpeed;
            }

            const double du = (segmentSpeed * deltaSeconds * speedScale_) / segmentDistance;
            segmentU_ += du;

            if (segmentU_ < 1.0) {
                break;
            }

            if (currentSegment_ >= points_.size() - 2) {
                currentSegment_ = points_.size() - 1;
                segmentU_ = 0.0;
                break;
            }

            segmentU_ -= 1.0;
            ++currentSegment_;
            deltaSeconds = 0.0;
        }

        return applyCurrentPose(data);
    }

    bool TrajectoryPlaybackController::stepToNextPoint(TrajOptimizationData& data)
    {
        if (!isLoaded()) {
            return false;
        }

        if (currentSegment_ < points_.size() - 1) {
            ++currentSegment_;
        }
        segmentU_ = 0.0;
        return applyCurrentPose(data);
    }

    bool TrajectoryPlaybackController::applyCurrentPose(TrajOptimizationData& data)
    {
        if (!isLoaded()) {
            return false;
        }

        return applyPose(data, currentPoseMatrix());
    }

    bool TrajectoryPlaybackController::isFinished() const
    {
        return isLoaded() && currentSegment_ >= points_.size() - 1;
    }

    std::size_t TrajectoryPlaybackController::currentPointIndex() const
    {
        if (points_.empty()) {
            return 0;
        }
        return std::min(currentSegment_, points_.size() - 1);
    }

    double TrajectoryPlaybackController::currentProgress() const
    {
        if (!isLoaded()) {
            return 0.0;
        }

        const double segmentCount = static_cast<double>(points_.size() - 1);
        const double position = std::min(
            static_cast<double>(currentSegment_) + segmentU_,
            segmentCount);
        return segmentCount > 0.0 ? position / segmentCount : 0.0;
    }

    Eigen::Matrix4d TrajectoryPlaybackController::currentPoseMatrix() const
    {
        if (points_.empty()) {
            return Eigen::Matrix4d::Identity();
        }

        if (currentSegment_ >= points_.size() - 1) {
            const PlaybackPoint& last = points_.back();
            return makePoseMatrix(last.position, last.orientation);
        }

        const PlaybackPoint& start = points_[currentSegment_];
        const PlaybackPoint& end = points_[currentSegment_ + 1];
        const double u = std::max(0.0, std::min(1.0, segmentU_));

        const Eigen::Vector3d position = (1.0 - u) * start.position + u * end.position;
        Eigen::Quaterniond orientation = start.orientation.slerp(u, end.orientation);
        orientation.normalize();
        return makePoseMatrix(position, orientation);
    }

    bool TrajectoryPlaybackController::applyPose(
        TrajOptimizationData& data,
        const Eigen::Matrix4d& pose)
    {
        if (data.m_robots.empty() && !data.toolRobot_) {
            return false;
        }

        const unsigned int index = resolveRobotIndex(data);
        const glm::mat4 glmPose = CollisionDetection::eigenMatrixToGlm(pose);

        if (index < data.m_robots.size()) {
            return CollisionDetection::set_robot_pose_matrix(data, index, glmPose);
        }

        if (data.toolRobot_) {
            data.toolRobot_->set_base_mat(glmPose);
            return data.toolRobot_->forward_kinematic();
        }

        return false;
    }

    unsigned int TrajectoryPlaybackController::resolveRobotIndex(const TrajOptimizationData& data) const
    {
        if (data.toolRobot_) {
            for (std::size_t i = 0; i < data.m_robots.size(); ++i) {
                if (data.m_robots[i] == data.toolRobot_) {
                    return static_cast<unsigned int>(i);
                }
            }
        }

        if (robotIndex_ < data.m_robots.size()) {
            return robotIndex_;
        }

        return robotIndex_;
    }
}
