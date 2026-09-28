#pragma once

#include "TrajOptimization_global.h"
#include <TrajOptimizationData/TrajOptimizationData.h>
#include <Eigen/Dense>
#include <cstddef>
#include <string>
#include <vector>

namespace ModelTrajPlan
{
    class TrajOptimization_SHAREDLIB TrajectoryPlaybackController
    {
    public:
        struct PlaybackPoint
        {
            Eigen::Vector3d position = Eigen::Vector3d::Zero();
            Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
            double speed = 50.0;
            double timestamp = 0.0;
            int pointType = 0;
            double sprayDistance = 0.0;
        };

        bool loadPaqFile(const std::string& filePath, std::string* errorMessage = nullptr);
        bool loadOptimizedTrajectory(const TrajOptimizationData& data);
        void clear();
        void reset();

        void setSpeedScale(double scale);
        double speedScale() const { return speedScale_; }

        void setRobotIndex(unsigned int robotIndex);
        unsigned int robotIndex() const { return robotIndex_; }

        bool stepByTime(TrajOptimizationData& data, double deltaSeconds);
        bool stepToNextPoint(TrajOptimizationData& data);
        bool applyCurrentPose(TrajOptimizationData& data);

        bool isLoaded() const { return points_.size() >= 2; }
        bool isFinished() const;
        std::size_t pointCount() const { return points_.size(); }
        std::size_t currentPointIndex() const;
        double currentProgress() const;
        const std::vector<PlaybackPoint>& points() const { return points_; }

    private:
        Eigen::Matrix4d currentPoseMatrix() const;
        bool applyPose(TrajOptimizationData& data, const Eigen::Matrix4d& pose);
        unsigned int resolveRobotIndex(const TrajOptimizationData& data) const;

        std::vector<PlaybackPoint> points_;
        std::size_t currentSegment_ = 0;
        double segmentU_ = 0.0;
        double speedScale_ = 1.0;
        unsigned int robotIndex_ = 1;
    };
}
