#pragma once

#include "CollisionDetection_global.h"
#include <Eigen/Dense>
#include <ModelTangQing/imodel.h>
#include <GeneralData/GeneralData.h>
#include <RenderData/RenderData.h>
#include <CameraTangQing/icamera.h>
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/icollision_detector.h>
#include <SMRobotTangQing/robot_factory.h>
#include <vector>
#include <string>
#include <map>
#include <TrajOptimizationData/TrajOptimizationData.h>
// Forward declaration

namespace ModelTrajPlan
{
    class TrajOptimizationData;

    // ========== Collision detection algorithm class (pure algorithm, stateless) ==========
    // Responsibility: operate on TrajOptimizationData and implement all collision-detection algorithms
    // Principle: static methods, no member variables, only operate on input data
    // Note: all data members are moved to TrajOptimizationData
    class CollisionDetection_SHAREDLIB CollisionDetection
    {
    public:
        // === Initialization methods ===
        static bool initializeRobots(
            TrajOptimizationData& data,
            const std::vector<std::string>& robot_model_paths);

        // === Rendering methods ===
        static bool draw(TrajOptimizationData& data);
        static bool resize_window(TrajOptimizationData& data, int width, int height);
        static bool run_job1(TrajOptimizationData& data);

        // === Scene access ===
        static iscene_spointer get_scene(TrajOptimizationData& data);

        // === Scene setup ===
        static bool set_global_lights(TrajOptimizationData& data);
        static bool set_global_materials(TrajOptimizationData& data);

        // === Collision detection methods ===
        static bool init_collision(TrajOptimizationData& data);
        static bool is_collision(TrajOptimizationData& data);

        // === Utility methods ===
        static glm::mat4 eigenMatrixToGlm(const Eigen::Matrix4d& m);
        static Eigen::Matrix4d glmMatrixToEigen(const glm::mat4& m);
        static void collect_all_models(
            const irobot_spointer& robot,
            std::vector<imodel_spointer>& models);

        // === Robot control methods ===
        static bool set_robot_pose(
            TrajOptimizationData& data,
            unsigned int robot_index,
            const glm::mat4& transform);

        static bool robot_joint_decrease(
            TrajOptimizationData& data,
            unsigned int robot_index,
            unsigned int joint_index,
            float decrement,
            AngleUnit unit = AngleUnit::Degree);

        static bool robot_joint_increase(
            TrajOptimizationData& data,
            unsigned int robot_index,
            unsigned int joint_index,
            float increment,
            AngleUnit unit = AngleUnit::Degree);

        static bool move_robot_pose(
            TrajOptimizationData& data,
            unsigned int robot_index,
            const glm::vec3& translation_delta,
            const glm::vec3& rotation_delta_deg,
            bool use_local_coord);

        static bool set_robot_pose_absolute(
            TrajOptimizationData& data,
            unsigned int robot_index,
            const glm::vec3& position,
            const glm::vec3& euler_deg);

        static bool set_robot_pose_matrix(
            TrajOptimizationData& data,
            unsigned int robot_index,
            const glm::mat4& pose_matrix);

        // === Collision-visualization methods ===
        static void setCollisionVisualization(
            TrajOptimizationData& data,
            bool enable);

        static void updateCollisionColors(
            TrajOptimizationData& data);
    };


}