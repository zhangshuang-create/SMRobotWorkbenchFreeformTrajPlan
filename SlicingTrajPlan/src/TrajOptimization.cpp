#include <TrajOptimization/TrajOptimization.h>
#include <CollisionDetection/CollisionDetection.h>
#include <OptiAlgorithm/OptiAlgorithm.h>
#include <OptSafeTrajData/OptSafeTrajData.h>
#include <QDebug>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <algorithm>
#include <ctime>
#include <vector>
#include <map>
#include <utility>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <unordered_set>
#ifdef _WIN32
#include <direct.h>
#include <io.h>
#include <stdio.h>
#define access _access
#define F_OK 0
#define unlink _unlink
#else
#include <unistd.h>
#include <dirent.h>
#endif

// ========== Algorithm Implementation: TrajOptimizationData ==========
namespace ModelTrajPlan
{
static unsigned int resolveToolRobotIndex(const TrajOptimizationData& data)
{
    if (data.toolRobot_) {
        for (std::size_t i = 0; i < data.m_robots.size(); ++i) {
            if (data.m_robots[i] == data.toolRobot_) {
                return static_cast<unsigned int>(i);
            }
        }
    }

    if (data.m_robots.size() > 1) {
        return 1U;
    }

    return 0U;
}

bool TrajOptimizationAlgorithm::initializeRobotSystem(
    TrajOptimizationData& data,
    const std::vector<std::string>& robot_paths)
{
    if (robot_paths.size() < 2) {
        std::cerr << "[TrajOptAlgo] Error: Need at least 2 robot paths!" << std::endl;
        return false;
    }
    
    // Initialize the robot system using CollisionDetection's static methods
    if (!CollisionDetection::initializeRobots(data, robot_paths)) {
        std::cerr << "[TrajOptAlgo] Error: Failed to initialize robots!" << std::endl;
        return false;
    }
    
    data.systemInitialized_ = true;
    std::cout << "[TrajOptAlgo] Robot system initialized successfully." << std::endl;
    return true;
}

bool TrajOptimizationAlgorithm::loadOriginalModel(
    TrajOptimizationData& data,
    const std::string& file_path)
{
    // TODO: Implement loading of the original model
    data.originalModelPath_ = file_path;
    data.hasOriginalModel_ = true;
    std::cout << "[TrajOptAlgo] Original model loaded: " << file_path << std::endl;
    return true;
}

bool TrajOptimizationAlgorithm::loadActualModel(
    TrajOptimizationData& data,
    const std::string& file_path)
{
    // TODO: Implement loading of the actual model
    data.actualModelPath_ = file_path;
    data.hasActualModel_ = true;
    std::cout << "[TrajOptAlgo] Actual model loaded: " << file_path << std::endl;
    return true;
}

bool TrajOptimizationAlgorithm::loadTrajectoryFromFile(
    TrajOptimizationData& data,
    const std::string& file_path)
{
    bool result = parseTrajectoryFile(data, file_path);
    if (result) {
        // Reset transform matrix to identity when loading a new trajectory
        data.transformMatrix_ = Eigen::Matrix4d::Identity();
        std::cout << "[TrajOptAlgo] Transform matrix reset to identity" << std::endl;
        
        // Set trajectory as visible and flag it for update so it renders during draw
        data.trajectoryVisible_ = true;
        data.trajectoryFrameNeedsUpdate_ = true;  // Needs update
    }
    return result;
}

bool TrajOptimizationAlgorithm::loadTrajectoryFromGeneralData(
    TrajOptimizationData& data,
    const GeneralData& generalData)
{
    // Load trajectory data from GeneralData
    data.originalTrajectory_ = generalData.trajData_.trajectoryPointsbf_;
    
    if (data.originalTrajectory_.empty()) {
        std::cerr << "[TrajOptAlgo] Error: No trajectory in GeneralData!" << std::endl;
        return false;
    }
    
    // Reset transform matrix to identity when loading a new trajectory
    data.transformMatrix_ = Eigen::Matrix4d::Identity();
    
    // Process trajectory data (Convert STL to trajectory optimization)
    // Note: If GeneralData contains STL files, trajectory optimization may be needed
    if (!data.originalTrajectory_.empty()) {
        // Note: Casting away const to modify processing state
        const_cast<GeneralData&>(generalData).processingState_.isSTLProcessed = true;
    }
    
    std::cout << "[TrajOptAlgo] Loaded " << data.originalTrajectory_.size() 
              << " trajectory points from GeneralData." << std::endl;
    std::cout << "[TrajOptAlgo] Transform matrix reset to identity" << std::endl;
    return true;
}

void TrajOptimizationAlgorithm::setTransformMatrix(
    TrajOptimizationData& data,
    const Eigen::Matrix4d& matrix)
{
    data.transformMatrix_ = matrix;
    std::cout << "[TrajOptAlgo] Transform matrix updated." << std::endl;
}

bool TrajOptimizationAlgorithm::autoCalculateTransform(
    TrajOptimizationData& data)
{
    if (!data.hasOriginalModel_ || !data.hasActualModel_) {
        std::cerr << "[TrajOptAlgo] Error: Need both original and actual models for auto-calculation!" << std::endl;
        return false;
    }
    
    // TODO: Implement ICP (Iterative Closest Point) calibration algorithm
    std::cout << "[TrajOptAlgo] Auto-calculate transform (ICP) - Not implemented yet." << std::endl;
    return false;
}

bool TrajOptimizationAlgorithm::transformTrajectory(
    TrajOptimizationData& data)
{
    if (data.originalTrajectory_.empty()) {
        std::cerr << "[TrajOptAlgo] Error: No original trajectory to transform!" << std::endl;
        return false;
    }
    
    data.transformedTrajectory_.clear();
    
    // Apply transformation to every trajectory point
    for (const auto& item : data.originalTrajectory_) {
        double time = item.first;
        const TolerancedTrajectoryPoint& point = item.second;
        
        TolerancedTrajectoryPoint transformed_point = point;
        
        // Transform pose: T_new = TransformMatrix * T_original
        transformed_point.pose = data.transformMatrix_ * point.pose;
        
        data.transformedTrajectory_[time] = transformed_point;
    }
    
    std::cout << "[TrajOptAlgo] Trajectory transformed: " << data.transformedTrajectory_.size() 
              << " points." << std::endl;
    data.trajectoryFrameNeedsUpdate_ = true;
    return true;
}

bool TrajOptimizationAlgorithm::initCollision(
    TrajOptimizationData& data)
{
    if (!data.systemInitialized_) {
        std::cerr << "[TrajOptAlgo] Error: System not initialized!" << std::endl;
        return false;
    }
    
    // Reset old detector
    data.m_spDetector.reset();
    
    // Initialize collision detector using static methods of CollisionDetection
    if (!CollisionDetection::init_collision(data)) {
        std::cerr << "[TrajOptAlgo] Error: Failed to initialize collision detector!" << std::endl;
        return false;
    }
    
    std::cout << "[TrajOptAlgo] Collision detector initialized successfully." << std::endl;
    return true;
}

void TrajOptimizationAlgorithm::setCollisionVisualization(
    TrajOptimizationData& data,
    bool enabled)
{
    // Use CollisionDetection static method to set collision visualization
    CollisionDetection::setCollisionVisualization(data, enabled);
}

bool TrajOptimizationAlgorithm::startCollisionDetection(
    TrajOptimizationData& data)
{
    if (!data.systemInitialized_) {
        std::cerr << "[TrajOptAlgo] Error: System not initialized!" << std::endl;
        return false;
    }
    
    data.collisionDetectionRunning_ = true;
    std::cout << "[TrajOptAlgo] Collision detection started." << std::endl;
    return true;
}

void TrajOptimizationAlgorithm::stopCollisionDetection(
    TrajOptimizationData& data)
{
    data.collisionDetectionRunning_ = false;
    std::cout << "[TrajOptAlgo] Collision detection stopped." << std::endl;
}

bool TrajOptimizationAlgorithm::isCollision(
    TrajOptimizationData& data)
{
    if (!data.systemInitialized_) {
        return false;
    }
    
    // Use CollisionDetection static method to check for collision
    return CollisionDetection::is_collision(data);
}

bool TrajOptimizationAlgorithm::computeTrajectoryToleranceWithProgress(
    TrajOptimizationData& data,
    const ToleranceLimitConfig& tol_limit,
    std::function<void(int, int, const std::string&)> progressCallback)
{
    if (data.transformedTrajectory_.empty()) {
        return false;
    }
    
    if (!data.systemInitialized_) {
        return false;
    }
    
    data.tolerancedTrajectory_.clear();
    data.pointCalculationTimes_.clear();  // Clear previous timing statistics
    data.pointCollisionCheckCounts_.clear();  // Clear previous collision check counts
    
    // Reset total collision check time
    data.totalCollisionCheckTime_ = 0.0;
    data.totalCollisionCheckCount_ = 0;

    const unsigned int toolRobotIndex = resolveToolRobotIndex(data);
    Eigen::Matrix4d previousSelectedPose = Eigen::Matrix4d::Identity();
    bool hasPreviousSelectedPose = false;
    
    int totalPoints = data.transformedTrajectory_.size();
    int currentPoint = 0;
    
    for (const auto& pair : data.transformedTrajectory_) {
        // Check if stop requested
        if (!data.collisionDetectionRunning_) {
            qDebug() << "[computeTrajectoryToleranceWithProgress] Stopped at point" << currentPoint 
                     << "/" << totalPoints << "(collisionDetectionRunning_ = false)";
            return false;
        }
        
        // ?????????????????????????????????
        // if (currentPoint % 10 == 0) {
        //     qDebug() << "[computeTrajectoryToleranceWithProgress] Processing point" << currentPoint 
        //              << "/" << totalPoints << ", running =" << data.collisionDetectionRunning_;
        // }
        
        double t = pair.first;
        const TolerancedTrajectoryPoint& pt = pair.second;
        TolerancedTrajectoryPoint new_pt = pt;
        
        // Reset current point collision check count
        data.currentPointCollisionCheckCount_ = 0;
        
        // Record start time for current point
        auto pointStartTime = std::chrono::high_resolution_clock::now();
        
        // ??????????§Ù??????????
        // ????????§Ø????spray_distance??parseTrajectoryFile??????????????????????20.0??
        double spray_distance = pt.spray_distance;
        if (spray_distance < 1e-6) {
            spray_distance = 0.0;  // ????20.0????parseTrajectoryFile?????????????????????demo6????????100.0??
        }
        OptiAlgorithm::computeSinglePointTolerance(
            data, static_cast<int>(toolRobotIndex), pt.pose, new_pt.pos_tolerance, new_pt.orient_tolerance, tol_limit, spray_distance);
        
        // ?????¦Ë?????????????????????????¦Ë??????computeSinglePointTolerance?????????¦Ë???
        glm::mat4 original_glm_pose = CollisionDetection::eigenMatrixToGlm(pt.pose);
        CollisionDetection::set_robot_pose_matrix(data, toolRobotIndex, original_glm_pose);
        bool original_is_collision_free = !CollisionDetection::is_collision(data);
        
        Eigen::Matrix4d collision_free_pose;
        bool found_collision_free = false;
        
        // ????????????????????????
        bool tried_tolerance_method = false;
        bool tolerance_method_succeeded = false;
        bool tried_ga_method = false;
        bool ga_method_succeeded = false;
        
        // ????????????????¨°???????
        if (data.useParallelGAMethod_) {
            // ========== ???§Ö??? GA + RRT ??==========
            // ?????¦Ë????????????¨°???GA??
            if (!original_is_collision_free) {
                // ??¦Ë????????????¨°???GA??
                Eigen::Vector3d tool_tcp_pos = pt.pose.block<3, 1>(0, 3);
                Eigen::Matrix3d original_rot = pt.pose.block<3, 3>(0, 0);
                Eigen::Vector3d tool_z_axis = -original_rot.col(2).normalized();
                if (tool_z_axis.norm() < 1e-6) {
                    tool_z_axis = Eigen::Vector3d(0, 0, -1);
                }
                
                // ?????????
                double sy = std::sqrt(original_rot(0,0)*original_rot(0,0) + original_rot(1,0)*original_rot(1,0));
                bool singular = sy < 1e-6;
                Eigen::Vector3d goal_orient(0, 0, 0);
                if (!singular) {
                    goal_orient.x() = std::atan2(original_rot(2,1), original_rot(2,2));
                    goal_orient.y() = std::atan2(-original_rot(2,0), sy);
                    goal_orient.z() = std::atan2(original_rot(1,0), original_rot(0,0));
                } else {
                    goal_orient.x() = std::atan2(-original_rot(1,2), original_rot(1,1));
                    goal_orient.y() = std::atan2(-original_rot(2,0), sy);
                    goal_orient.z() = 0;
                }
                
                OptiAlgorithm optimizer;
                std::vector<Eigen::Matrix4d> ga_solutions;
                double max_z_translation = tol_limit.max_translation.norm();
                if (max_z_translation < 1.0) {
                    max_z_translation = 5.5;
                } else {
                    max_z_translation = std::min(max_z_translation, 7.0);
                }
                
                tried_ga_method = true;
                ga_method_succeeded = optimizer.optimizeWithGAIslandsRRT(
                    data, static_cast<int>(toolRobotIndex), tool_tcp_pos, tool_z_axis, goal_orient, spray_distance, false, 0.0, ga_solutions);
                
                if (!ga_method_succeeded || ga_solutions.empty()) {
                    ga_method_succeeded = optimizer.optimizeWithGAIslandsRRT(
                        data, static_cast<int>(toolRobotIndex), tool_tcp_pos, tool_z_axis, goal_orient, spray_distance, true, max_z_translation, ga_solutions);
                }
                
                if (ga_method_succeeded && !ga_solutions.empty()) {
                    OptiAlgorithm::selectContinuousSolution(
                        ga_solutions, pt.pose,
                        hasPreviousSelectedPose ? &previousSelectedPose : nullptr,
                        collision_free_pose);
                    // ???????????
                    glm::mat4 test_glm = CollisionDetection::eigenMatrixToGlm(collision_free_pose);
                    CollisionDetection::set_robot_pose_matrix(data, toolRobotIndex, test_glm);
                    if (!CollisionDetection::is_collision(data)) {
                        found_collision_free = true;
                        data.rrtBackupPoints_[t] = ga_solutions;
                        data.rrtGeneratedPoints_[t] = true;
                    }
                }
            } else {
                // ??¦Ë?????????????????¦Ë??
                collision_free_pose = pt.pose;
                found_collision_free = true;
            }
        } else if (data.useACOMethod_) {
            // ========== ????? ACO==========
            // ?????¦Ë?????????????ACO??
            if (!original_is_collision_free) {
                // ??¦Ë?????????????ACO??
                Eigen::Vector3d tool_tcp_pos = pt.pose.block<3, 1>(0, 3);
                Eigen::Matrix3d original_rot = pt.pose.block<3, 3>(0, 0);
                Eigen::Vector3d tool_z_axis = -original_rot.col(2).normalized();
                if (tool_z_axis.norm() < 1e-6) {
                    tool_z_axis = Eigen::Vector3d(0, 0, -1);
                }
                
                // ?????????
                double sy = std::sqrt(original_rot(0,0)*original_rot(0,0) + original_rot(1,0)*original_rot(1,0));
                bool singular = sy < 1e-6;
                Eigen::Vector3d goal_orient(0, 0, 0);
                if (!singular) {
                    goal_orient.x() = std::atan2(original_rot(2,1), original_rot(2,2));
                    goal_orient.y() = std::atan2(-original_rot(2,0), sy);
                    goal_orient.z() = std::atan2(original_rot(1,0), original_rot(0,0));
                } else {
                    goal_orient.x() = std::atan2(-original_rot(1,2), original_rot(1,1));
                    goal_orient.y() = std::atan2(-original_rot(2,0), sy);
                    goal_orient.z() = 0;
                }
                
                OptiAlgorithm optimizer;
                std::vector<Eigen::Matrix4d> ga_solutions;
                
                tried_ga_method = true;
                ga_method_succeeded = optimizer.optimizeWithACO(
                    data, static_cast<int>(toolRobotIndex), tool_tcp_pos, tool_z_axis, goal_orient, spray_distance, ga_solutions);
                
                if (ga_method_succeeded && !ga_solutions.empty()) {
                    OptiAlgorithm::selectContinuousSolution(
                        ga_solutions, pt.pose,
                        hasPreviousSelectedPose ? &previousSelectedPose : nullptr,
                        collision_free_pose);
                    // ???????????
                    glm::mat4 test_glm = CollisionDetection::eigenMatrixToGlm(collision_free_pose);
                    CollisionDetection::set_robot_pose_matrix(data, toolRobotIndex, test_glm);
                    if (!CollisionDetection::is_collision(data)) {
                        found_collision_free = true;
                        data.rrtBackupPoints_[t] = ga_solutions;
                        data.rrtGeneratedPoints_[t] = true;
                    }
                }
            } else {
                // ??¦Ë?????????????????¦Ë??
                collision_free_pose = pt.pose;
                found_collision_free = true;
            }
        } else {
            // ========== ??? GA + ??? RRT ??==========
            // 1. ?????¦Ë?????????????????????????¦¶??????computeSinglePointTolerance????????????????¦Ë??
            // 2. ?????¦Ë?????????????demo6???????GA???????????????
            
            if (original_is_collision_free) {
                // ??¦Ë????????????¦¶?????computeSinglePointTolerance?????????????????¦Ë??
                collision_free_pose = pt.pose;
                found_collision_free = true;
            } else {
                // ??¦Ë?????????????demo6???????GA??
                // ?¦Ï?demo6.cpp??optimizeSingleTrajectoryPoint????
                Eigen::Vector3d pos = pt.pose.block<3, 1>(0, 3);
                Eigen::Matrix3d R = pt.pose.block<3, 3>(0, 0);
                
                // ???????????????? ZXY??
                double sy = std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
                bool singular = sy < 1e-6;
                Eigen::Vector3d goal_orient(0, 0, 0);
                if (!singular) {
                    goal_orient.x() = std::atan2(R(2,1), R(2,2));
                    goal_orient.y() = std::atan2(-R(2,0), sy);
                    goal_orient.z() = std::atan2(R(1,0), R(0,0));
                } else {
                    goal_orient.x() = std::atan2(-R(1,2), R(1,1));
                    goal_orient.y() = std::atan2(-R(2,0), sy);
                    goal_orient.z() = 0;
                }
                
                // ????z??????????
                Eigen::Vector3d tool_z_axis = -R.col(2).normalized();
                if (tool_z_axis.norm() < 1e-6) {
                    tool_z_axis = Eigen::Vector3d(0, 0, -1);
                }
                
                // ???GA?????????demo6????
                OptiAlgorithm optimizer;
                std::vector<Eigen::Matrix4d> solutions;
                
                tried_ga_method = true;
                if (optimizer.optimizeWithGA(data, static_cast<int>(toolRobotIndex), pos, tool_z_axis, goal_orient, spray_distance, false, 0.0, solutions)) {
                    if (!solutions.empty()) {
                        // ??????????????
                        OptiAlgorithm::selectContinuousSolution(
                            solutions, pt.pose,
                            hasPreviousSelectedPose ? &previousSelectedPose : nullptr,
                            collision_free_pose);
                        
                        // ???????????????demo6????
                        glm::mat4 test_glm = CollisionDetection::eigenMatrixToGlm(collision_free_pose);
                        CollisionDetection::set_robot_pose_matrix(data, toolRobotIndex, test_glm);
                        if (!CollisionDetection::is_collision(data)) {
                            found_collision_free = true;
                            ga_method_succeeded = true;
                            data.rrtBackupPoints_[t] = solutions;
                            data.rrtGeneratedPoints_[t] = true;
                        }
                    }
                }
                
                // ???GA???????????????????¦Ë?????demo6????
                if (!found_collision_free) {
                    CollisionDetection::set_robot_pose_matrix(data, toolRobotIndex, original_glm_pose);
                    if (!CollisionDetection::is_collision(data)) {
                        collision_free_pose = pt.pose;
                        found_collision_free = true;
                    }
                }
            }
        }
        
        // ????????????????????????????¦Ë??
        if (found_collision_free) {
            // ?????????????????????????????????????????????
            new_pt.pose = collision_free_pose;
            previousSelectedPose = collision_free_pose;
            hasPreviousSelectedPose = true;
        } else {
            // ???§Ù??????????
            // ??????????????????????????¦Ë??
            new_pt.pose = pt.pose;
            data.unavoidableCollisionPoints_.push_back(currentPoint);
            
            // ?????????????????§Ø???????????
            qDebug() << QString("[Collision Point #%1] Marked as unavoidable collision point:")
                        .arg(currentPoint);
            qDebug() << QString("  - Original pose collision: %1")
                        .arg(original_is_collision_free ? "No (collision-free)" : "Yes (has collision)");
            
            // ?????????
            double avgPosTol = (new_pt.pos_tolerance.col(1) - new_pt.pos_tolerance.col(0)).mean();
            double avgOriTol = (new_pt.orient_tolerance.col(1) - new_pt.orient_tolerance.col(0)).mean() * 180.0 / M_PI;
            qDebug() << QString("  - Tolerance range: Position ??%1 mm, Rotation ??%2 deg")
                        .arg(avgPosTol, 0, 'f', 2)
                        .arg(avgOriTol, 0, 'f', 2);
            
            // ???????????????
            qDebug() << QString("  - Methods tried:");
            if (original_is_collision_free) {
                qDebug() << QString("    * Original pose check: Passed (collision-free)");
            } else {
                qDebug() << QString("    * Original pose check: Failed (has collision)");
                
                if (tried_ga_method) {
                    qDebug() << QString("    * Optimization algorithm: %1")
                                .arg(ga_method_succeeded ? "Succeeded" : "Failed (no collision-free solution found)");
                }
            }
            
            // ????????????
            QStringList failureReasons;
            if (!original_is_collision_free) {
                failureReasons << "Original pose has collision";
                
                if (tried_ga_method && !ga_method_succeeded) {
                    failureReasons << "Optimization algorithm failed";
                }
            } else {
                // ?????????????????¦Ë????????found_collision_free????true??
                failureReasons << "Unexpected: Original pose is collision-free but still marked as collision point!";
            }
            
            qDebug() << QString("  - Failure chain: %1")
                        .arg(failureReasons.join(" -> "));
            
            // ???¦Ë?????
            Eigen::Vector3d pos = pt.pose.block<3, 1>(0, 3);
            Eigen::Matrix3d R = pt.pose.block<3, 3>(0, 0);
            double sy = std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
            bool singular = sy < 1e-6;
            Eigen::Vector3d euler(0, 0, 0);
            if (!singular) {
                euler.x() = std::atan2(R(2,1), R(2,2)) * 180.0 / M_PI;
                euler.y() = std::atan2(-R(2,0), sy) * 180.0 / M_PI;
                euler.z() = std::atan2(R(1,0), R(0,0)) * 180.0 / M_PI;
            } else {
                euler.x() = std::atan2(-R(1,2), R(1,1)) * 180.0 / M_PI;
                euler.y() = std::atan2(-R(2,0), sy) * 180.0 / M_PI;
                euler.z() = 0;
            }
            qDebug() << QString("  - Pose: Position (%1, %2, %3) mm, Orientation (Rx=%4??, Ry=%5??, Rz=%6??)")
                        .arg(pos.x(), 0, 'f', 2)
                        .arg(pos.y(), 0, 'f', 2)
                        .arg(pos.z(), 0, 'f', 2)
                        .arg(euler.x(), 0, 'f', 2)
                        .arg(euler.y(), 0, 'f', 2)
                        .arg(euler.z(), 0, 'f', 2);
        }
        
        // Record end time for current point calculation
        auto pointEndTime = std::chrono::high_resolution_clock::now();
        auto pointDuration = std::chrono::duration_cast<std::chrono::milliseconds>(pointEndTime - pointStartTime);
        double pointTimeMs = pointDuration.count();
        
        data.tolerancedTrajectory_[t] = new_pt;
        data.pointCalculationTimes_[t] = pointTimeMs;  // Store calculation time
        data.pointCollisionCheckCounts_[t] = data.currentPointCollisionCheckCount_;  // Store collision check count
        
        // Update progress
        currentPoint++;
        if (progressCallback) {
            progressCallback(currentPoint, totalPoints, "Computing tolerance...");
        }
    }
    
    // ??????????????
    qDebug() << "\n========== Optimization Statistics ==========";
    
    // ????????????
    if (data.totalCollisionCheckCount_ > 0) {
        double avgCollisionCheckTime = data.totalCollisionCheckTime_ / data.totalCollisionCheckCount_;
        qDebug() << "[Collision Detection] Total calls:" << data.totalCollisionCheckCount_ 
                 << ", Total time:" << (data.totalCollisionCheckTime_ / 1000.0) << "s"
                 << ", Avg time:" << avgCollisionCheckTime << "ms";
    }
    
    // ?????????????????unavoidable?§Ò??§Ö???????????????
    std::vector<int> failedPoints;
    int pointIndex = 0;
    for (const auto& pair : data.transformedTrajectory_) {
        bool isUnavoidable = std::find(data.unavoidableCollisionPoints_.begin(), 
                                       data.unavoidableCollisionPoints_.end(), 
                                       pointIndex) != data.unavoidableCollisionPoints_.end();
        
        // ?????unavoidable?§Ò??§Ö????????????????????????????????
        if (isUnavoidable) {
            failedPoints.push_back(pointIndex);
        }
        pointIndex++;
    }
    
    // ???????????
    if (!failedPoints.empty()) {
        qDebug() << "\n[Failed Points] Total:" << failedPoints.size() << "points";
        QString indicesStr = "Point indices: ";
        for (size_t i = 0; i < failedPoints.size(); ++i) {
            indicesStr += QString::number(failedPoints[i]);
            if (i < failedPoints.size() - 1) {
                indicesStr += ", ";
            }
            if ((i + 1) % 20 == 0) {  // ?20??????
                indicesStr += "\n                 ";
            }
        }
        qDebug() << indicesStr;
    } else {
        qDebug() << "\n[Success] All points found collision-free solutions!";
    }

    // ???????????????????????????????¦Ë???
    if (!failedPoints.empty()) {
        // ????????????????¨´???????????????????
        auto getDir = [](const std::string& p) {
            if (p.empty()) return std::string(".");
            size_t pos = p.find_last_of("/\\");
            if (pos == std::string::npos) return std::string(".");
            return p.substr(0, pos);
        };

        std::string baseDir = getDir(data.trajectoryFilePath_);
        std::string failedFilePath = baseDir + "/failed_points.txt";

        std::ofstream ofs(failedFilePath);
        if (!ofs.is_open()) {
            qWarning() << "[TrajOptAlgo] Failed to open failed_points.txt for writing at" << QString::fromStdString(failedFilePath);
        } else {
            ofs << "# Failed collision-free trajectory points\n";
            ofs << "# index,time_s,pos_x_mm,pos_y_mm,pos_z_mm,rx_deg,ry_deg,rz_deg\n";

            std::unordered_set<int> failedSet(failedPoints.begin(), failedPoints.end());

            int idx = 0;
            for (const auto& pair : data.transformedTrajectory_) {
                if (failedSet.find(idx) == failedSet.end()) {
                    idx++;
                    continue;
                }

                double t = pair.first;
                const TolerancedTrajectoryPoint& pt = pair.second;

                Eigen::Vector3d pos = pt.pose.block<3,1>(0,3);
                Eigen::Matrix3d R = pt.pose.block<3,3>(0,0);

                double sy = std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
                bool singular = sy < 1e-6;
                Eigen::Vector3d euler(0,0,0);
                if (!singular) {
                    euler.x() = std::atan2(R(2,1), R(2,2));
                    euler.y() = std::atan2(-R(2,0), sy);
                    euler.z() = std::atan2(R(1,0), R(0,0));
                } else {
                    euler.x() = std::atan2(-R(1,2), R(1,1));
                    euler.y() = std::atan2(-R(2,0), sy);
                    euler.z() = 0;
                }

                // §Õ????§µ??????????
                ofs << idx << ","
                    << std::fixed << std::setprecision(3) << t << ","
                    << std::setprecision(2)
                    << pos.x() << "," << pos.y() << "," << pos.z() << ","
                    << (euler.x() * 180.0 / M_PI) << ","
                    << (euler.y() * 180.0 / M_PI) << ","
                    << (euler.z() * 180.0 / M_PI)
                    << "\n";

                idx++;
            }

            ofs.close();
            qDebug() << "[TrajOptAlgo] Failed points exported to" << QString::fromStdString(failedFilePath);
        }
    }
    
    // ????????????????GA???????
    if (!data.unavoidableCollisionPoints_.empty()) {
        qDebug() << "\n[GA Algorithm Failed Points] Total:" << data.unavoidableCollisionPoints_.size() << "points";
        qDebug() << "These points could not find collision-free solutions using GA algorithm (including Z-translation).";
        qDebug() << "";
        
        // ??????????§Ò???????????
        QString indicesStr = "Point indices: ";
        for (size_t i = 0; i < data.unavoidableCollisionPoints_.size(); ++i) {
            indicesStr += QString::number(data.unavoidableCollisionPoints_[i]);
            if (i < data.unavoidableCollisionPoints_.size() - 1) {
                indicesStr += ", ";
            }
            if ((i + 1) % 20 == 0) {  // ?20??????
                indicesStr += "\n                 ";
            }
        }
        qDebug() << indicesStr;
        qDebug() << "";
        
        // ?????????????????
        qDebug() << "Detailed information for failed points:";
        qDebug() << "----------------------------------------";
        
        int pointIndex = 0;
        for (const auto& pair : data.transformedTrajectory_) {
            double t = pair.first;
            bool isUnavoidable = std::find(data.unavoidableCollisionPoints_.begin(), 
                                           data.unavoidableCollisionPoints_.end(), 
                                           pointIndex) != data.unavoidableCollisionPoints_.end();
            
            if (isUnavoidable) {
                const TolerancedTrajectoryPoint& pt = pair.second;
                Eigen::Vector3d pos = pt.pose.block<3, 1>(0, 3);
                Eigen::Matrix3d R = pt.pose.block<3, 3>(0, 0);
                
                // ?????????
                double sy = std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
                bool singular = sy < 1e-6;
                Eigen::Vector3d euler(0, 0, 0);
                if (!singular) {
                    euler.x() = std::atan2(R(2,1), R(2,2)) * 180.0 / M_PI;  // Rx (??)
                    euler.y() = std::atan2(-R(2,0), sy) * 180.0 / M_PI;     // Ry (??)
                    euler.z() = std::atan2(R(1,0), R(0,0)) * 180.0 / M_PI;  // Rz (??)
                } else {
                    euler.x() = std::atan2(-R(1,2), R(1,1)) * 180.0 / M_PI;
                    euler.y() = std::atan2(-R(2,0), sy) * 180.0 / M_PI;
                    euler.z() = 0;
                }
                
                // ?????????
                double avgPosTol = (pt.pos_tolerance.col(1) - pt.pos_tolerance.col(0)).mean();
                double avgOriTol = (pt.orient_tolerance.col(1) - pt.orient_tolerance.col(0)).mean() * 180.0 / M_PI;
                
                qDebug() << QString("Point #%1 (Time: %2s):")
                            .arg(pointIndex)
                            .arg(t, 0, 'f', 3);
                qDebug() << QString("  Position: (%1, %2, %3) mm")
                            .arg(pos.x(), 0, 'f', 2)
                            .arg(pos.y(), 0, 'f', 2)
                            .arg(pos.z(), 0, 'f', 2);
                qDebug() << QString("  Orientation: (Rx=%1??, Ry=%2??, Rz=%3??)")
                            .arg(euler.x(), 0, 'f', 2)
                            .arg(euler.y(), 0, 'f', 2)
                            .arg(euler.z(), 0, 'f', 2);
                qDebug() << QString("  Tolerance: Pos=%1mm, Ori=%2??")
                            .arg(avgPosTol, 0, 'f', 2)
                            .arg(avgOriTol, 0, 'f', 2);
                
                // ?????????????????
                if (data.pointCollisionCheckCounts_.find(t) != data.pointCollisionCheckCounts_.end()) {
                    int checkCount = data.pointCollisionCheckCounts_[t];
                    qDebug() << "  Collision checks:" << checkCount;
                }
                
                qDebug() << "";
            }
            pointIndex++;
        }
    }
    
    qDebug() << "==========================================";
    qDebug() << "";
    
    return true;
}

bool TrajOptimizationAlgorithm::exportTransformedTrajectory(
    TrajOptimizationData& data,
    const std::string& file_path)
{
    if (data.transformedTrajectory_.empty()) {
        std::cerr << "[TrajOptAlgo] Error: No transformed trajectory to export!" << std::endl;
        return false;
    }
    
    std::ofstream fout(file_path);
    if (!fout.is_open()) {
        std::cerr << "[TrajOptAlgo] Error: Cannot create file: " << file_path << std::endl;
        return false;
    }
    
    fout << "# Transformed Trajectory\n";
    fout << "# Format: time px py pz qx qy qz qw\n";
    fout << std::fixed << std::setprecision(6);
    
    for (const auto& item : data.transformedTrajectory_) {
        double time = item.first;
        const TolerancedTrajectoryPoint& point = item.second;
        
        Eigen::Vector3d pos = point.pose.block<3, 1>(0, 3);
        Eigen::Matrix3d rot = point.pose.block<3, 3>(0, 0);
        Eigen::Quaterniond quat(rot);
        
        fout << time << " "
             << pos.x() << " " << pos.y() << " " << pos.z() << " "
             << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << "\n";
    }
    
    fout.close();
    std::cout << "[TrajOptAlgo] Transformed trajectory exported to: " << file_path << std::endl;
    return true;
}

bool TrajOptimizationAlgorithm::exportTolerancedTrajectory(
    TrajOptimizationData& data,
    const std::string& file_path)
{
    if (data.tolerancedTrajectory_.empty()) {
        std::cerr << "[TrajOptAlgo] Error: No toleranced trajectory to export!" << std::endl;
        return false;
    }
    
    std::ofstream fout(file_path);
    if (!fout.is_open()) {
        std::cerr << "[TrajOptAlgo] Error: Cannot create file: " << file_path << std::endl;
        return false;
    }
    
    fout << "# Toleranced Trajectory Export\n";
    fout << "# Format: x y z qw qx qy qz velocity acceleration timestamp point_type spray_distance\n";
    fout << "# Time is calculated from distance and velocity between consecutive points\n";
    fout << std::fixed << std::setprecision(3);
    
    // Accumulate timestamp
    double accumulated_time = 0.0;
    Eigen::Vector3d prev_position(0, 0, 0);
    bool is_first_point = true;
    
    for (const auto& item : data.tolerancedTrajectory_) {
        const TolerancedTrajectoryPoint& point = item.second;
        
        // Get position
        Eigen::Vector3d position = point.pose.block<3, 1>(0, 3);
        
        // Get rotation matrix and convert to quaternion
        Eigen::Matrix3d rotation = point.pose.block<3, 3>(0, 0);
        Eigen::Quaterniond quat(rotation);
        
        // Calculate time delta (starting from the second point)
        if (!is_first_point) {
            double distance = (position - prev_position).norm();
            double avg_velocity = (point.vel_range(0) + point.vel_range(1)) / 2.0;
            if (avg_velocity <= 0.0) {
                avg_velocity = 100.0;  // Default velocity 100 mm/s
            }
            double delta_time = distance / avg_velocity;
            accumulated_time += delta_time;
        }
        
        // Output format: x y z qw qx qy qz velocity acceleration timestamp point_type spray_distance
        // Note: Quaternion order is qw qx qy qz, matching TrajectoryExporter
        fout << position.x() << " "
             << position.y() << " "
             << position.z() << " "
             << quat.w() << " "  // w first
             << quat.x() << " "
             << quat.y() << " "
             << quat.z() << " "
             << (point.vel_range(0) + point.vel_range(1)) / 2.0 << " "
             << "0.000 "  // acceleration (temp 0)
             << std::setprecision(6) << accumulated_time << std::setprecision(3) << " "
             << "0 "  // point_type (default 0)
             << point.spray_distance << "\n";  // spray_distance (mm)
        
        // Update previous position
        prev_position = position;
        is_first_point = false;
    }
    
    fout.close();
    std::cout << "[TrajOptAlgo] Toleranced trajectory exported to: " << file_path << std::endl;
    return true;
}

// Helper: Convert TolerancedTrajectoryPoint to PublicTrajectoryPoint
PublicData::OptTrajectoryPoint TrajOptimizationAlgorithm::convertToPublicTrajectoryPoint(
    TrajOptimizationData& data,
    double timestamp,
    const TolerancedTrajectoryPoint& tol_point)
{
    PublicData::OptTrajectoryPoint public_point;
    
    // Copy basic properties
    public_point.pose = tol_point.pose;
    public_point.pos_tolerance = tol_point.pos_tolerance;
    public_point.orient_tolerance = tol_point.orient_tolerance;
    public_point.is_adjustable = tol_point.is_adjustable;
    public_point.tolerance_frame = tol_point.tolerance_frame;
    public_point.spray_distance = tol_point.spray_distance;  // Copy spray_distance
    
    // Check if generated by RRT
    bool is_rrt_generated = (data.rrtGeneratedPoints_.find(timestamp) != data.rrtGeneratedPoints_.end() 
                             && data.rrtGeneratedPoints_[timestamp]);
    
    if (is_rrt_generated) {
        public_point.generation_method = PublicData::TrajectoryGenerationMethod::RRT;
        
        // Get backup candidates from rrtBackupPoints_
        if (data.rrtBackupPoints_.find(timestamp) != data.rrtBackupPoints_.end()) {
            public_point.backup_points = data.rrtBackupPoints_[timestamp];
        }
    } else {
        public_point.generation_method = PublicData::TrajectoryGenerationMethod::Tolerance;
    }
    
    return public_point;
}

// Convert all points to a map of PublicTrajectoryPoints
PublicData::OptTrajectoryPoints TrajOptimizationAlgorithm::convertToPublicTrajectoryPoints(
    TrajOptimizationData& data)
{
    PublicData::OptTrajectoryPoints result;
    
    for (const auto& item : data.tolerancedTrajectory_) {
        double timestamp = item.first;
        const TolerancedTrajectoryPoint& tol_point = item.second;
        
        // Use helper conversion function
        PublicData::OptTrajectoryPoint public_point = convertToPublicTrajectoryPoint(data, timestamp, tol_point);
        
        result[timestamp] = public_point;
    }
    
    return result;
}

bool TrajOptimizationAlgorithm::exportToPublicTrajData(
    TrajOptimizationData& data,
    const std::string& file_path)
{
    if (data.tolerancedTrajectory_.empty()) {
        std::cerr << "[TrajOptAlgo] Error: No toleranced trajectory to export!" << std::endl;
        return false;
    }
    
    // Generate readable txt file
    std::ofstream fout_txt(file_path);
    if (!fout_txt.is_open()) {
        std::cerr << "[TrajOptAlgo] Error: Cannot create txt file: " << file_path << std::endl;
        return false;
    }
    
    size_t point_count = data.tolerancedTrajectory_.size();
    
    fout_txt << "# Public Trajectory Data Export (Human Readable)\n";
    fout_txt << "# Total Points: " << point_count << "\n";
    fout_txt << "# Format: [Point Index] Pose(4x4) | PosTolerance(X,Y,Z min/max) | OrientTolerance(Rx,Ry,Rz min/max) | Adjustable | Frame\n";
    fout_txt << "#\n";
    fout_txt << std::fixed << std::setprecision(6);
    
    int point_index = 0;
    for (const auto& item : data.tolerancedTrajectory_) {
        double t = item.first;
        const TolerancedTrajectoryPoint& tol_point = item.second;
        
        // Convert to PublicTrajectoryPoint using helper
        PublicData::OptTrajectoryPoint public_point = convertToPublicTrajectoryPoint(data, t, tol_point);
        
        fout_txt << "\n=== Point " << point_index << " ===\n";
        
        // Pose matrix
        fout_txt << "Pose (4x4 matrix):\n";
        for (int r = 0; r < 4; ++r) {
            fout_txt << "  ";
            for (int c = 0; c < 4; ++c) {
                fout_txt << std::setw(12) << public_point.pose(r, c) << " ";
            }
            fout_txt << "\n";
        }
        
        // Orientation Tolerance (converted to degrees, order Z, X, Y)
        fout_txt << "Orientation Tolerance (deg) [Z, X, Y order]:\n";
        fout_txt << "  Rz: [" << public_point.orient_tolerance(2, 0) * 180.0 / M_PI << ", " 
                 << public_point.orient_tolerance(2, 1) * 180.0 / M_PI << "]\n";
        fout_txt << "  Rx: [" << public_point.orient_tolerance(0, 0) * 180.0 / M_PI << ", " 
                 << public_point.orient_tolerance(0, 1) * 180.0 / M_PI << "]\n";
        fout_txt << "  Ry: [" << public_point.orient_tolerance(1, 0) * 180.0 / M_PI << ", " 
                 << public_point.orient_tolerance(1, 1) * 180.0 / M_PI << "]\n";
        
        // Position Tolerance (order Z, X, Y)
        fout_txt << "Position Tolerance (mm) [Z, X, Y order]:\n";
        fout_txt << "  Z: [" << public_point.pos_tolerance(2, 0) << ", " << public_point.pos_tolerance(2, 1) << "]\n";
        fout_txt << "  X: [" << public_point.pos_tolerance(0, 0) << ", " << public_point.pos_tolerance(0, 1) << "]\n";
        fout_txt << "  Y: [" << public_point.pos_tolerance(1, 0) << ", " << public_point.pos_tolerance(1, 1) << "]\n";
        
        // Properties
        fout_txt << "Adjustable: " << (public_point.is_adjustable ? "true" : "false") << "\n";
        
        // Tolerance Frame
        std::string frame_str = "World";
        if (public_point.tolerance_frame == PublicData::CoordinateFrame::Workpiece) {
            frame_str = "Workpiece";
        } else if (public_point.tolerance_frame == PublicData::CoordinateFrame::Local) {
            frame_str = "Local";
        }
        fout_txt << "Tolerance Frame: " << frame_str << "\n";
        
        // Generation Method
        std::string method_str = "Manual";
        if (public_point.generation_method == PublicData::TrajectoryGenerationMethod::RRT) {
            method_str = "RRT";
        } else if (public_point.generation_method == PublicData::TrajectoryGenerationMethod::Tolerance) {
            method_str = "Tolerance";
        } else if (public_point.generation_method == PublicData::TrajectoryGenerationMethod::Other) {
            method_str = "Other";
        }
        fout_txt << "Generation Method: " << method_str << "\n";
        
        // Spray Distance
        fout_txt << "Spray Distance: " << public_point.spray_distance << " mm\n";
        
        // If generated by RRT, output backup candidates
        if (public_point.generation_method == PublicData::TrajectoryGenerationMethod::RRT && !public_point.backup_points.empty()) {
            fout_txt << "Backup Points Count: " << public_point.backup_points.size() << "\n";
            for (size_t i = 0; i < public_point.backup_points.size(); ++i) {
                fout_txt << "Backup Point " << i << " (4x4 matrix):\n";
                for (int r = 0; r < 4; ++r) {
                    fout_txt << "  ";
                    for (int c = 0; c < 4; ++c) {
                        fout_txt << std::setw(12) << public_point.backup_points[i](r, c) << " ";
                    }
                    fout_txt << "\n";
                }
            }
        }
        
        point_index++;
    }
    
    fout_txt.close();
    std::cout << "[TrajOptAlgo] Trajectory exported to: " << file_path << std::endl;
    std::cout << "[TrajOptAlgo] Exported " << point_count << " points in PublicTrajectoryPoint format" << std::endl;
    
    return true;
}

bool TrajOptimizationAlgorithm::draw(TrajOptimizationData& data)
{
    // Load Workpiece (needs OpenGL context)
    if (data.hasOriginalModel_ && data.m_spScene) {
        // Check if already loaded
        bool alreadyLoaded = false;
        for (const auto& robot : data.m_robots) {
            if (robot == data.workpieceRobot_) {
                alreadyLoaded = true;
                break;
            }
        }
        
        if (!alreadyLoaded && data.workpieceRobot_) {
            bool isXml = (data.originalModelPath_.find(".xml") != std::string::npos || 
                          data.originalModelPath_.find(".XML") != std::string::npos);
            bool isRbt = (data.originalModelPath_.find(".rbt") != std::string::npos || 
                          data.originalModelPath_.find(".RBT") != std::string::npos);
            
            bool loadSuccess = false;
            
            if (isXml && data.workpieceLoader_) {
                // XML format: use loader->compile_robot()
                std::vector<irobot_spointer> robots = data.workpieceLoader_->compile_robot(data.m_spScene);
                if (!robots.empty()) {
                    data.workpieceRobot_ = robots[0];
                    loadSuccess = true;
                }
            } else if (isRbt) {
                // RBT format: use robot->load()
                loadSuccess = data.workpieceRobot_->load(data.originalModelPath_.c_str(), data.m_spScene);
            }
            
            if (loadSuccess && data.workpieceRobot_) {
                data.workpieceRobot_->set_robot_name("Workpiece");
                data.workpieceRobot_->update_instance_name();
                
                // Set Workpiece at origin (Identity matrix)
                glm::mat4 base = glm::mat4(1.0f);
                data.workpieceRobot_->set_base_mat(base);
                
                data.m_robots.push_back(data.workpieceRobot_);
                std::cout << "[TrajOptAlgo] Workpiece loaded at origin (0, 0, 0)" << std::endl;
            }
        }
    }
    
    // Load Tool (needs OpenGL context)
    if (data.hasActualModel_ && data.m_spScene) {
        // Check if already loaded
        bool alreadyLoaded = false;
        for (const auto& robot : data.m_robots) {
            if (robot == data.toolRobot_) {
                alreadyLoaded = true;
                break;
            }
        }
        
        if (!alreadyLoaded && data.toolRobot_) {
            bool isXml = (data.actualModelPath_.find(".xml") != std::string::npos || 
                          data.actualModelPath_.find(".XML") != std::string::npos);
            bool isRbt = (data.actualModelPath_.find(".rbt") != std::string::npos || 
                          data.actualModelPath_.find(".RBT") != std::string::npos);
            
            bool loadSuccess = false;
            
            if (isXml && data.toolLoader_) {
                // XML format: use loader->compile_robot()
                std::vector<irobot_spointer> robots = data.toolLoader_->compile_robot(data.m_spScene);
                if (!robots.empty()) {
                    data.toolRobot_ = robots[0];
                    loadSuccess = true;
                }
            } else if (isRbt) {
                // RBT format: use robot->load()
                loadSuccess = data.toolRobot_->load(data.actualModelPath_.c_str(), data.m_spScene);
            }
            
            if (loadSuccess && data.toolRobot_) {
                data.toolRobot_->set_robot_name("Tool");
                data.toolRobot_->update_instance_name();
                
                // Set Tool at origin (Identity matrix)
                glm::mat4 base = glm::mat4(1.0f);
                data.toolRobot_->set_base_mat(base);
                
                data.m_robots.push_back(data.toolRobot_);
                std::cout << "[TrajOptAlgo] Tool loaded at origin (0, 0, 0)" << std::endl;
            }
        }
    }
    
    // Draw trajectory frames. Check if trajectory is loaded and scene exists.
    const bool hasTrajectoryForDisplay =
        !data.transformedTrajectory_.empty() || !data.originalTrajectory_.empty();
    if (hasTrajectoryForDisplay && data.m_spScene) {
        // Deferred initialization: if frames are empty or null, create them
        if (data.trajectoryFrameNeedsUpdate_ || data.trajectoryFrameModels_.empty()) {
            createTrajectoryFrames(data, 50.0f);
            data.trajectoryFrameNeedsUpdate_ = false;
        }
        
        // Draw frames
        drawTrajectoryFrames(data);
    }
    
    // Draw robots (CollisionDetection handles this)
    if (!data.m_robots.empty()) {
        return CollisionDetection::draw(data);
    }
    
    return true;
}

bool TrajOptimizationAlgorithm::parseTrajectoryFile(
    TrajOptimizationData& data,
    const std::string& file_path)
{
    std::ifstream fin(file_path);
    if (!fin.is_open()) {
        std::cerr << "[TrajOptAlgo] Error: Cannot open file: " << file_path << std::endl;
        return false;
    }
    
    data.originalTrajectory_.clear();
    std::string line;
    int line_no = 0;
    
    // Accumulate timestamp
    double accumulated_time = 0.0;
    Eigen::Vector3d prev_position(0, 0, 0);
    bool is_first_point = true;
    
    while (std::getline(fin, line)) {
        ++line_no;
        if (line.empty() || line[0] == '#' || line[0] == '%') continue;
        
        std::istringstream iss(line);
        std::vector<double> vals;
        double val;
        while (iss >> val) {
            vals.push_back(val);
        }
        
        // Format: x y z qw qx qy qz velocity acceleration timestamp point_type spray_distance
        // Indices: 0 1 2 3  4  5  6  7        8           9         10         11
        if (vals.size() < 7) continue;  // Need at least px py pz qw qx qy qz
        
        TolerancedTrajectoryPoint point;
        
        // Calculate pose matrix
        Eigen::Vector3d pos(vals[0], vals[1], vals[2]);
        Eigen::Quaterniond quat(vals[3], vals[4], vals[5], vals[6]);  // w, x, y, z
        if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z()) ||
            !std::isfinite(quat.w()) || !std::isfinite(quat.x()) ||
            !std::isfinite(quat.y()) || !std::isfinite(quat.z()) ||
            !std::isfinite(quat.norm()) || quat.norm() < 1e-12) {
            std::cerr << "[TrajOptimization] Skipping invalid trajectory point at line "
                      << line_no << std::endl;
            continue;
        }
        quat.normalize();
        
        point.pose = Eigen::Matrix4d::Identity();
        point.pose.block<3, 3>(0, 0) = quat.toRotationMatrix();
        point.pose.block<3, 1>(0, 3) = pos;
        
        // Get velocity (8th column in file, index 7)
        double velocity = 100.0;  // Default velocity 100 mm/s
        if (vals.size() > 7 && vals[7] > 0.0) {
            velocity = vals[7];
        }
        
        // Set velocity range (min=max for now)
        point.vel_range(0) = velocity;
        point.vel_range(1) = velocity;
        
        // Get spray_distance (last column in file, index 11)
        // Format: x y z qw qx qy qz velocity acceleration timestamp point_type spray_distance
        // Indices: 0  1  2  3  4  5  6  7        8           9         10         11
        if (vals.size() >= 12) {
            // File has spray_distance field (index 11)
            point.spray_distance = vals[11];
        } else if (vals.size() >= 8) {
            // File doesn't have spray_distance, use default 20.0 (matching demo6's output format)
            point.spray_distance = 0.0;
        } else {
            // Very old format, use default
            point.spray_distance = 0.0;
        }
        
        // Calculate timestamp based on distance and velocity
        if (!is_first_point) {
            // Distance from previous point
            double distance = (pos - prev_position).norm();
            
            // Time delta = distance / velocity
            double delta_time = distance / velocity;
            accumulated_time += delta_time;
        }
        
        // Use accumulated time as key
        data.originalTrajectory_[accumulated_time] = point;
        
        // Update previous position
        prev_position = pos;
        is_first_point = false;
    }
    
    fin.close();
    
    std::cout << "[TrajOptAlgo] Loaded " << data.originalTrajectory_.size() 
              << " trajectory points from file." << std::endl;
    std::cout << "[TrajOptAlgo] Total trajectory time: " << accumulated_time << " seconds" << std::endl;
    return !data.originalTrajectory_.empty();
}

// ========== Trajectory Visualization Implementation ==========

// Helper: Create coordinate system model data
static void createAxisData(MeshData& data, float scale)
{
    // Clear data
    data.m_vertices.clear();
    data.m_indices.clear();
    
    // Vertex definition
    // Format: x, y, z, r, g, b, a
    
    // X Axis (Red): Origin to (scale, 0, 0)
    data.m_vertices.insert(data.m_vertices.end(), {
        0.0f, 0.0f, 0.0f,      1.0f, 0.0f, 0.0f, 1.0f,  // Origin, Red
        scale, 0.0f, 0.0f,     1.0f, 0.0f, 0.0f, 1.0f   // X Endpoint, Red
    });
    
    // Y Axis (Green): Origin to (0, scale, 0)
    data.m_vertices.insert(data.m_vertices.end(), {
        0.0f, 0.0f, 0.0f,      0.0f, 1.0f, 0.0f, 1.0f,  // Origin, Green
        0.0f, scale, 0.0f,     0.0f, 1.0f, 0.0f, 1.0f   // Y Endpoint, Green
    });
    
    // Z Axis (Blue): Origin to (0, 0, scale)
    data.m_vertices.insert(data.m_vertices.end(), {
        0.0f, 0.0f, 0.0f,      0.0f, 0.0f, 1.0f, 1.0f,  // Origin, Blue
        0.0f, 0.0f, scale,     0.0f, 0.0f, 1.0f, 1.0f   // Z Endpoint, Blue
    });
    
    // Indices
    data.m_indices = {
        0, 1,  // X Axis
        2, 3,  // Y Axis
        4, 5   // Z Axis
    };
}

// Helper: Apply pose transformation to vertices
static void applyTransformationToVertices(MeshData& data, const Eigen::Matrix4d& pose)
{
    // Each vertex format: x, y, z, r, g, b, a (7 floats)
    const int stride = 7;
    
    for (size_t i = 0; i < data.m_vertices.size(); i += stride) {
        // Get original position
        Eigen::Vector4d vertex(
            data.m_vertices[i],
            data.m_vertices[i + 1],
            data.m_vertices[i + 2],
            1.0
        );
        
        // Apply transform
        Eigen::Vector4d transformed = pose * vertex;
        
        // Update vertex position
        data.m_vertices[i] = static_cast<float>(transformed.x());
        data.m_vertices[i + 1] = static_cast<float>(transformed.y());
        data.m_vertices[i + 2] = static_cast<float>(transformed.z());
        // Color remains unchanged (i+3 to i+6)
    }
}

bool TrajOptimizationAlgorithm::createTrajectoryFrames(
    TrajOptimizationData& data,
    float scale)
{
    if (!data.m_spScene) {
        std::cerr << "[TrajOptAlgo] Error: Scene not initialized!" << std::endl;
        return false;
    }
    
    const auto& trajectoryToDisplay =
        data.transformedTrajectory_.empty() ? data.originalTrajectory_ : data.transformedTrajectory_;

    if (trajectoryToDisplay.empty()) {
        std::cerr << "[TrajOptAlgo] Warning: No trajectory to visualize!" << std::endl;
        return false;
    }
    
    // Clear previous models
    data.trajectoryFrameModels_.clear();
    
    // Create coordinate system model for each trajectory point
    for (const auto& item : trajectoryToDisplay) {
        const TolerancedTrajectoryPoint& point = item.second;
        
        // Create new model
        imodel_spointer frame = data.m_spScene->get_new_model(MODELTYPE::MODELTYPE_COLORLINE_ELEMENT);
        
        if (frame) {
            // Create and transform axis data
            MeshData transformedData;
            createAxisData(transformedData, scale);
            applyTransformationToVertices(transformedData, point.pose);
            
            // Import transformed data
            frame->import(transformedData);
            
            data.trajectoryFrameModels_.push_back(frame);
        }
    }
    
    std::cout << "[TrajOptAlgo] Created " << data.trajectoryFrameModels_.size() 
              << " trajectory frames with scale " << scale << std::endl;
    
    return !data.trajectoryFrameModels_.empty();
}

void TrajOptimizationAlgorithm::setTrajectoryVisibility(
    TrajOptimizationData& data,
    bool visible)
{
    data.trajectoryVisible_ = visible;
    
    std::cout << "[TrajOptAlgo] Trajectory visibility set to: " 
              << (visible ? "visible" : "hidden") << std::endl;
}

bool TrajOptimizationAlgorithm::drawTrajectoryFrames(
    TrajOptimizationData& data)
{
    // Check if scene exists and OpenGL context is ready
    if (!data.m_spScene) {
        return false;
    }
    
    if (data.trajectoryFrameModels_.empty()) {
        return false;
    }
    
    if (!data.trajectoryVisible_) {
        return true;  // If hidden, just return true (no draw call)
    }
    
    // Draw all coordinate frame models
    for (auto& frame : data.trajectoryFrameModels_) {
        if (frame) {
            frame->draw();
        }
    }
    
    return true;
}

// ========== Project Export Implementation ==========

// Helper: Create directory (Cross-platform)
static bool createDirectory(const std::string& dir_path)
{
#ifdef _WIN32
    if (_mkdir(dir_path.c_str()) == 0 || errno == EEXIST) {
        return true;
    }
#else
    if (mkdir(dir_path.c_str(), 0755) == 0 || errno == EEXIST) {
        return true;
    }
#endif
    return false;
}

// Helper: Copy file
static bool copyFile(const std::string& src, const std::string& dst)
{
    std::ifstream src_file(src, std::ios::binary);
    if (!src_file.is_open()) {
        std::cerr << "[Export] Error: Cannot open source file: " << src << std::endl;
        return false;
    }
    
    std::ofstream dst_file(dst, std::ios::binary);
    if (!dst_file.is_open()) {
        std::cerr << "[Export] Error: Cannot create destination file: " << dst << std::endl;
        src_file.close();
        return false;
    }
    
    dst_file << src_file.rdbuf();
    
    src_file.close();
    dst_file.close();
    
    return true;
}

// Helper: Check if file exists
static bool fileExists(const std::string& path)
{
#ifdef _WIN32
    return (_access(path.c_str(), 0) == 0);
#else
    return (access(path.c_str(), F_OK) == 0);
#endif
}

// Helper: Get file extension (lowercase)
static std::string getFileExtension(const std::string& file_path)
{
    size_t dot_pos = file_path.find_last_of('.');
    if (dot_pos == std::string::npos) {
        return "";
    }
    
    std::string ext = file_path.substr(dot_pos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext;
}

// Helper: Get file directory
static std::string getFileDirectory(const std::string& file_path)
{
    size_t slash_pos = file_path.find_last_of("/\\");
    if (slash_pos == std::string::npos) {
        return "";
    }
    return file_path.substr(0, slash_pos);
}

// Helper: Extract STL file references from XML
static std::vector<std::string> extractSTLReferencesFromXML(const std::string& xml_file_path)
{
    std::vector<std::string> stl_files;
    
    std::ifstream xml_file(xml_file_path);
    if (!xml_file.is_open()) {
        return stl_files;
    }
    
    std::string line;
    while (std::getline(xml_file, line)) {
        // Find pattern like: <GeometryFile name="xxx.stl"
        size_t geo_pos = line.find("GeometryFile");
        if (geo_pos != std::string::npos) {
            size_t name_pos = line.find("name=\"", geo_pos);
            if (name_pos != std::string::npos) {
                name_pos += 6; // Skip name="
                size_t quote_end = line.find("\"", name_pos);
                if (quote_end != std::string::npos) {
                    std::string stl_name = line.substr(name_pos, quote_end - name_pos);
                    // Check if it's an STL file
                    std::string ext = getFileExtension(stl_name);
                    if (ext == "stl" || ext == "STL") {
                        stl_files.push_back(stl_name);
                    }
                }
            }
        }
    }
    
    xml_file.close();
    return stl_files;
}

// Helper: Copy robot model file
static std::string copyRobotModel(
    const std::string& source_path,
    const std::string& dest_dir,
    const std::string& robot_id)
{
    if (source_path.empty() || !fileExists(source_path)) {
        std::cerr << "[Export] Warning: Model file does not exist: " << source_path << std::endl;
        return "";
    }
    
    std::string ext = getFileExtension(source_path);
    std::string dest_file;
    std::string relative_path;
    
    if (ext == "rbt") {
        // RBT format: copy directly
        dest_file = dest_dir + "/" + robot_id + "/" + robot_id + ".rbt";
        relative_path = "robot/" + robot_id + "/" + robot_id + ".rbt";
    } else if (ext == "xml") {
        // XML format: rename XML file
        dest_file = dest_dir + "/" + robot_id + "/" + robot_id + ".model.xml";
        relative_path = "robot/" + robot_id + "/" + robot_id + ".model.xml";
    } else {
        std::cerr << "[Export] Warning: Unsupported model format: " << ext << std::endl;
        return "";
    }
    
    // Create destination directory
    std::string robot_dir = dest_dir + "/" + robot_id;
    if (!createDirectory(robot_dir)) {
        std::cerr << "[Export] Error: Failed to create robot directory: " << robot_dir << std::endl;
        return "";
    }
    
    // If destination file exists, delete it
    if (fileExists(dest_file)) {
        unlink(dest_file.c_str());
    }
    
    // Copy main file
    if (!copyFile(source_path, dest_file)) {
        std::cerr << "[Export] Error: Failed to copy model file" << std::endl;
        return "";
    }
    
    std::cout << "[Export] Copied " << ext << " model: " << dest_file << std::endl;
    
    // If XML file, need to copy referenced STL files
    if (ext == "xml") {
        std::vector<std::string> stl_files = extractSTLReferencesFromXML(source_path);
        std::string model_dir = getFileDirectory(source_path);
        
        for (const auto& stl_name : stl_files) {
            std::string stl_source = model_dir + "/" + stl_name;
            std::string stl_dest = robot_dir + "/" + stl_name;
            
            if (fileExists(stl_source)) {
                if (fileExists(stl_dest)) {
                    unlink(stl_dest.c_str());
                }
                
                if (copyFile(stl_source, stl_dest)) {
                    std::cout << "[Export] Copied STL file: " << stl_name << std::endl;
                } else {
                    std::cerr << "[Export] Warning: Failed to copy STL file: " << stl_name << std::endl;
                }
            } else {
                std::cerr << "[Export] Warning: STL file not found: " << stl_source << std::endl;
            }
        }
    }
    
    return relative_path;
}

// Helper: Get current time as ISO string
static std::string getCurrentTimeISO()
{
    std::time_t now = std::time(nullptr);
    std::tm* timeinfo = std::localtime(&now);
    
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", timeinfo);
    return std::string(buffer);
}

// Helper: Generate JSON string
static std::string generateConfigJSON(
    const std::string& project_name,
    const std::vector<std::pair<std::string, std::string>>& robots, // robot_id -> model_file
    size_t trajectory_point_count,
    const std::vector<int>& collision_points)
{
    std::ostringstream json;
    json << "{\n";
    json << "  \"version\": \"1.0\",\n";
    json << "  \"project_name\": \"" << project_name << "\",\n";
    json << "  \"export_date\": \"" << getCurrentTimeISO() << "\",\n";
    json << "  \"robots\": [\n";
    
    for (size_t i = 0; i < robots.size(); ++i) {
        json << "    {\n";
        json << "      \"robot_id\": \"" << robots[i].first << "\",\n";
        json << "      \"robot_name\": \"" << (robots[i].first == "robot1" ? "Workpiece Robot" : "Tool Robot") << "\",\n";
        json << "      \"model_file\": \"" << robots[i].second << "\"\n";
        json << "    }";
        if (i < robots.size() - 1) {
            json << ",";
        }
        json << "\n";
    }
    
    json << "  ],\n";
    json << "  \"trajectory\": {\n";
    json << "    \"trajectory_file\": \"Trajectory/trajectory.txt\",\n";
    json << "    \"point_count\": " << trajectory_point_count << ",\n";
    json << "    \"has_tolerance\": true,\n";
    
    if (!collision_points.empty()) {
        json << "    \"collision_points\": [";
        for (size_t i = 0; i < collision_points.size(); ++i) {
            json << collision_points[i];
            if (i < collision_points.size() - 1) {
                json << ", ";
            }
        }
        json << "],\n";
        json << "    \"collision_count\": " << collision_points.size() << "\n";
    } else {
        json << "    \"collision_count\": 0\n";
    }
    
    json << "  }\n";
    json << "}\n";
    
    return json.str();
}

bool TrajOptimizationAlgorithm::exportProject(
    TrajOptimizationData& data,
    const std::string& project_dir,
    const std::string& project_name)
{
    // 1. Validation
    if (data.tolerancedTrajectory_.empty()) {
        std::cerr << "[Export] Error: No toleranced trajectory to export!" << std::endl;
        return false;
    }
    
    // 2. Create directory structure
    std::string robot_dir = project_dir + "/robot";
    std::string traj_dir = project_dir + "/Trajectory";
    
    if (!createDirectory(robot_dir)) {
        std::cerr << "[Export] Error: Failed to create robot directory!" << std::endl;
        return false;
    }
    
    if (!createDirectory(traj_dir)) {
        std::cerr << "[Export] Error: Failed to create Trajectory directory!" << std::endl;
        return false;
    }
    
    // 3. Export trajectory file
    std::string traj_file = traj_dir + "/trajectory.txt";
    if (!exportToPublicTrajData(data, traj_file)) {
        std::cerr << "[Export] Error: Failed to export trajectory!" << std::endl;
        return false;
    }
    
    // 3.1 ???????????????????? Trajectory ??
    {
        auto extractEulerDeg = [](const Eigen::Matrix3d& R) {
            double sy = std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
            bool singular = sy < 1e-6;
            Eigen::Vector3d euler(0,0,0);
            if (!singular) {
                euler.x() = std::atan2(R(2,1), R(2,2));
                euler.y() = std::atan2(-R(2,0), sy);
                euler.z() = std::atan2(R(1,0), R(0,0));
            } else {
                euler.x() = std::atan2(-R(1,2), R(1,1));
                euler.y() = std::atan2(-R(2,0), sy);
                euler.z() = 0;
            }
            return euler * 180.0 / M_PI;
        };

        // ??????unavoidable?§Ò??§Ö??????????????????????????????????????
        std::vector<int> failedPoints;
        int pointIndex = 0;
        for (const auto& pair : data.transformedTrajectory_) {
            bool isUnavoidable = std::find(data.unavoidableCollisionPoints_.begin(), 
                                           data.unavoidableCollisionPoints_.end(), 
                                           pointIndex) != data.unavoidableCollisionPoints_.end();
            
            // ?????unavoidable?§Ò??§Ö?????????
            if (isUnavoidable) {
                failedPoints.push_back(pointIndex);
            }
            pointIndex++;
        }

        if (!failedPoints.empty()) {
            std::string failedFilePath = traj_dir + "/failed_points.txt";
            std::ofstream ofs(failedFilePath);
            if (ofs.is_open()) {
                ofs << "# Failed collision-free trajectory points\n";
                ofs << "# index,time_s,pos_x_mm,pos_y_mm,pos_z_mm,rx_deg,ry_deg,rz_deg\n";

                std::unordered_set<int> failedSet(failedPoints.begin(), failedPoints.end());
                int idx = 0;
                for (const auto& pair : data.transformedTrajectory_) {
                    if (failedSet.find(idx) == failedSet.end()) {
                        idx++;
                        continue;
                    }

                    double t = pair.first;
                    const TolerancedTrajectoryPoint& pt = pair.second;
                    Eigen::Vector3d pos = pt.pose.block<3,1>(0,3);
                    Eigen::Matrix3d R = pt.pose.block<3,3>(0,0);
                    Eigen::Vector3d eulerDeg = extractEulerDeg(R);

                    ofs << idx << ","
                        << std::fixed << std::setprecision(3) << t << ","
                        << std::setprecision(2)
                        << pos.x() << "," << pos.y() << "," << pos.z() << ","
                        << eulerDeg.x() << ","
                        << eulerDeg.y() << ","
                        << eulerDeg.z() << "\n";
                    idx++;
                }
                ofs.close();
                std::cout << "[Export] Failed points exported to: " << failedFilePath << std::endl;
            } else {
                std::cerr << "[Export] Warning: Cannot write failed_points.txt at " << failedFilePath << std::endl;
            }
        }
    }
    
    // 4. Copy robot model files
    std::vector<std::pair<std::string, std::string>> robots; // robot_id -> model_file
    
    // Workpiece Robot (robot1)
    if (data.workpieceRobot_ && !data.originalModelPath_.empty()) {
        std::string model_file = copyRobotModel(data.originalModelPath_, robot_dir, "robot1");
        if (!model_file.empty()) {
            robots.push_back({"robot1", model_file});
        }
    }
    
    // Tool Robot (robot2)
    if (data.toolRobot_ && !data.actualModelPath_.empty()) {
        std::string model_file = copyRobotModel(data.actualModelPath_, robot_dir, "robot2");
        if (!model_file.empty()) {
            robots.push_back({"robot2", model_file});
        }
    }
    
    // 5. Generate config.json
    std::vector<int> collision_points(data.unavoidableCollisionPoints_.begin(), 
                                      data.unavoidableCollisionPoints_.end());
    std::string json_content = generateConfigJSON(
        project_name,
        robots,
        data.tolerancedTrajectory_.size(),
        collision_points
    );
    
    std::string config_file = project_dir + "/config.json";
    std::ofstream config_out(config_file);
    if (!config_out.is_open()) {
        std::cerr << "[Export] Error: Failed to create config.json!" << std::endl;
        return false;
    }
    
    config_out << json_content;
    config_out.close();
    
    std::cout << "[Export] Project exported successfully!" << std::endl;
    std::cout << "[Export] Project directory: " << project_dir << std::endl;
    std::cout << "[Export] Total points: " << data.tolerancedTrajectory_.size() << std::endl;
    if (!collision_points.empty()) {
        std::cout << "[Export] Warning: " << collision_points.size() 
                  << " points have unavoidable collisions" << std::endl;
    }
    return true;
}

}


