#pragma once
#include "TrajData_global.h"
#include <map>
#include <string>
#include <Eigen/Dense>


namespace PublicData
{

// Coordinate-frame enum
enum class CoordinateFrame {
    World,          // World frame (robot base)
    Workpiece,      // Workpiece frame
    Local           // Local frame
};


// Public trajectory-point struct
struct TrajectoryPoint {
    // Pose: transformation matrix stores position + orientation
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    // Spray distance (mm), distance from tool to workpiece surface
    double spray_distance = 0.0;                    // Used for adjustment based on workpiece target point
    double velocity = 0.0;                          // Tool speed (mm/s); <= 0 uses a consumer fallback
    // Scan-line identifier; -1 indicates that no spray pass is assigned.
    int spray_pass_id = -1;
    
    // Constructor
    TrajectoryPoint()
        : pose(Eigen::Matrix4d::Identity()),
          spray_distance(0.0),
          velocity(0.0),
          spray_pass_id(-1) {}  // Default spray distance is 0 mm

    void clear() {
        pose.setIdentity();
        spray_distance = 0.0;
        velocity = 0.0;
        spray_pass_id = -1;
    }
};

using TrajectoryPoints =std::map<double, TrajectoryPoint>;
using InterpolatedPoints = std::map<float, Eigen::Matrix4f>;

// Public trajectory-data class
class TrajData_SHAREDLIB TrajData
{
public:
    TrajData();
    ~TrajData();
    // Get trajectory-point collection (read-only)
    const TrajectoryPoints& getTrajectoryPoints() const { return trajectoryPoints_; }
    // Get trajectory-point collection (modifiable)
    TrajectoryPoints& getTrajectoryPoints() { return trajectoryPoints_; }
    // Get transformation matrix (read-only)
    const Eigen::Matrix4d& getTransformMatrix() const { return transformMatrix_; }
    // Get transformation matrix (modifiable)
    Eigen::Matrix4d& getTransformMatrix() { return transformMatrix_; }
    // Clear data
    void clear();
    // Check whether valid data exists
    bool hasValidData() const { return !trajectoryPoints_.empty(); }
    // Get trajectory-point count
    size_t getPointCount() const { return trajectoryPoints_.size(); }
    
    // Get trajectory file path (read-only)
    const std::string& getTrajectoryFilePath() const { return trajectory_file_path_; }
    
    // Get trajectory file path (modifiable)
    std::string& getTrajectoryFilePath() { return trajectory_file_path_; }
    
    // === Load trajectory data ===
    // Load trajectory data from txt file
    // file_path: full trajectory-file path
    // Return: true on success, false on failure
    bool TrajData::loadFromFile(const std::string& file_path, TrajectoryPoints& traj_points);
    // Load trajectory data directly from data structure
    // traj_points: map collection of trajectory points
    // Return: true on success, false on failure
    bool loadFromData( TrajectoryPoints& traj_points);

    void addInterpolatedPoint(float timestamp, const Eigen::Matrix4f& pose) {
        interpolatedPoints_[timestamp] = pose;
    }

    // Get interpolated-point collection (read-only)
    const InterpolatedPoints& getInterpolatedPoints() const { return interpolatedPoints_; }
    // Get interpolated-point collection (modifiable)
    InterpolatedPoints& getInterpolatedPoints() { return interpolatedPoints_; }

    // 获取螺旋轨迹点集合 (读写)
    const TrajectoryPoints& getSpiralTrajectoryPoints() const { return spiralTrajectoryPoints_; }
    TrajectoryPoints& getSpiralTrajectoryPoints() { return spiralTrajectoryPoints_; }

private:
    // Trajectory-point collection: uses map with timestamp as key
    TrajectoryPoints trajectoryPoints_;
    TrajectoryPoints spiralTrajectoryPoints_; // 新增的：螺旋轨迹
    // Transformation matrix: conversion matrix between different coordinate frames
    Eigen::Matrix4d transformMatrix_;  // 4x4 transformation matrix
    
    InterpolatedPoints interpolatedPoints_;

    // === Project file path ===
    std::string trajectory_file_path_;    // Trajectory file path (Trajectory/trajectory.txt)
};


}