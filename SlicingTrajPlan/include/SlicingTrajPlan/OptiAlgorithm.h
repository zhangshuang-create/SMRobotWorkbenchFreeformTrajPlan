#pragma once
#include "OptiAlgorithm_global.h"
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <TrajOptimizationData/TrajOptimizationData.h>
#include <GeneralData/PTrajData.h>
namespace ModelTrajPlan
{
    // Optimization algorithm type enum
    enum class OptimizationAlgorithmType {
        RRT,    // Rapidly-exploring Random Tree
        ACO,    // Ant Colony Optimization
        GA      // Genetic Algorithm
    };

    // ============================================================================
    // 数据结构定义
    // ============================================================================

    // STL triangle structure
    struct Triangle {
        Eigen::Vector3d v0, v1, v2;
        Eigen::Vector3d normal;

        Triangle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);
    };

    // Free-space region
    struct CSpaceFreeRegion {
        Eigen::Vector3d center;
        Eigen::Vector3d min_bound;
        Eigen::Vector3d max_bound;
        Eigen::Vector3d size;
        bool is_valid = false;
    };

    // RRT node
    struct RRTNode {
        Eigen::Vector3d position;
        Eigen::Vector3d orientation;
        RRTNode* parent;
        int node_id;

        RRTNode();
        RRTNode(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient);

        Eigen::Matrix4d toMatrix4d() const;
    };

    // GA individual
    struct Individual {
        Eigen::Vector3d orient;      // Orientation (Euler angles)
        double z_translation;          // Z-normal translation (mm), positive/negative allowed
        double fitness;
        bool is_valid;

        Individual();
    };

    // Optimization algorithm class
    class OptiAlgorithm_SHAREDLIB OptiAlgorithm
    {
    public:
        OptiAlgorithm();
        virtual ~OptiAlgorithm();

        // ============================================================================
        // Main interface: collision-free pose search
        // ============================================================================

        // ============================================================================
        // Core functions of each algorithm (directly callable)
        // ============================================================================

        /**
         * @brief ACO algorithm: search collision-free orientation at fixed position
         *
         * Uses Ant Colony Optimization in orientation space.
         * Characteristics:
         * - 模拟蚂蚁觅食行为
         * - 使用信息素（pheromone）引导搜索
         * - 搜索半径随信息素浓度动态调整
         * - 找到多个解（最多 8 个）
         *
         * @param data trajectory optimization data (collision detection system)
         * @param robot_index robot index (typically 0)
         * @param fixed_pos fixed position (XYZ, unchanged during search)
         * @param goal_orient target orientation (Euler Rx, Ry, Rz, used as search center)
         * @param[out] solutions all found collision-free solutions (up to 8, sorted by cost)
         * @return true if at least one solution is found, otherwise false
         *
         * @note
         * - Number of ants: 30
         * - Iterations: 40
         * - Initial search radius: 5-50 degrees (dynamically adjusted by pheromone)
         * - Pheromone evaporation rate: 0.2
         * - Cost function: Euler-angle distance to target orientation (degree)
         * - Solutions are sorted by ascending cost
         */
        bool optimizeWithACO(
            TrajOptimizationData& data,
            int robot_index,
            const Eigen::Vector3d& original_tool_tcp_pos,  // Tool TCP position of original trajectory point (fixed)
            const Eigen::Vector3d& original_tool_z_axis,   // Tool Z-axis of original trajectory point (normalized, points to workpiece)
            const Eigen::Vector3d& goal_orient,             // Target orientation (Euler angles)
            double spray_distance,                          // Spray distance (tool-to-workpiece surface distance)
            std::vector<Eigen::Matrix4d>& solutions);

        /**
         * @brief GA algorithm: search collision-free orientation at fixed position
         *
         * Uses Genetic Algorithm in orientation space.
         * Characteristics:
         * - Simulates biological evolution process
         * - Uses selection, crossover, and mutation
         * - Keeps elite individuals
         * - Finds multiple solutions (up to 12)
         * - Supports two-stage search: orientation-only first, then enables Z-normal translation on failure
         *
         * @param data trajectory optimization data (collision detection system)
         * @param robot_index robot index (typically 0)
         * @param fixed_pos fixed position (XYZ, unchanged during search)
         * @param goal_orient target orientation (Euler Rx, Ry, Rz, used as initial-population center)
         * @param z_normal Z-normal vector (normalized, used for Z-direction translation)
         * @param enable_z_translation whether to enable Z translation (if false, orientation only)
         * @param max_z_translation max Z translation (mm, positive/negative allowed)
         * @param[out] solutions all found collision-free solutions (up to 12, sorted by fitness)
         * @return true if at least one solution is found, otherwise false
         *
         * @note
         * - Population size: 80
         * - Generations: 80
         * - Initial perturbation range: +/-90 degrees
         * - Crossover rate: 0.9
         * - Mutation rate: 0.25
         * - Mutation amplitude: +/-25 degrees
         * - Elite count: 8
         * - Selection method: tournament selection (tournament size = 3)
         * - Fitness function: Euler-angle distance to target orientation (deg) + Z-translation penalty
         * - Solutions are sorted by ascending fitness
         */
        bool optimizeWithGA(
            TrajOptimizationData& data,
            int robot_index,
            const Eigen::Vector3d& original_tool_tcp_pos,  // Tool TCP position of original trajectory point (fixed)
            const Eigen::Vector3d& original_tool_z_axis,   // Tool Z-axis of original trajectory point (normalized, points to workpiece)
            const Eigen::Vector3d& goal_orient,             // Target orientation (Euler angles)
            double spray_distance,                          // Spray distance (tool-to-workpiece surface distance)
            bool enable_z_translation,
            double max_z_translation,
            std::vector<Eigen::Matrix4d>& solutions);

        /**
         * @brief Enhanced island GA + RRT hybrid algorithm (independent of existing GA)
         *
         * - 10 independent populations (islands) evolve in parallel without migration
         * - Best individual is selected from each island after several generations
         * - Island bests are aggregated; global best is selected and then expanded using RRT
         * - Returns merged GA + RRT solution set, where solutions[0] is the best GA solution
         *
         * @param data trajectory optimization data
         * @param robot_index robot index
         * @param fixed_pos fixed position
         * @param goal_orient target orientation (Euler Rx,Ry,Rz)
         * @param z_normal Z normal (normalized, for Z translation)
         * @param enable_z_translation whether Z translation is allowed
         * @param max_z_translation max Z translation (mm)
         * @param[out] solutions output: best GA solution + RRT expansion solutions
         * @return true if at least one valid solution is found
         */
        bool optimizeWithGAIslandsRRT(
            TrajOptimizationData& data,
            int robot_index,
            const Eigen::Vector3d& original_tool_tcp_pos,  // Tool TCP position of original trajectory point (fixed)
            const Eigen::Vector3d& original_tool_z_axis,   // Tool Z-axis of original trajectory point (normalized, points to workpiece)
            const Eigen::Vector3d& goal_orient,             // Target orientation (Euler angles)
            double spray_distance,                          // Spray distance (tool-to-workpiece surface distance)
            bool enable_z_translation,
            double max_z_translation,
            std::vector<Eigen::Matrix4d>& solutions);

        /**
         * @brief Local RRT expansion near given solutions
         *
         * Performs local RRT expansion near multiple solutions with small XYZ and orientation perturbations.
         * Used to further explore local neighborhoods around already found solutions.
         *
         * @param data trajectory optimization data (collision detection system)
         * @param robot_index robot index (typically 0)
         * @param base_positions base position list (solution centroid positions)
         * @param base_orientations base orientation list (solution Euler angles)
         * @param max_iterations max iteration count (default 200)
         * @param step_angle_deg orientation step size (degree, default 5)
         * @param step_pos_mm position step size (mm, default 10)
         * @param[out] rrt_nodes output local RRT node list (position + orientation)
         * @return true if at least one node is generated, otherwise false
         *
         * @note
         * - Randomly chooses parent nodes from base solutions
         * - Samples new positions and orientations near parent in each iteration
         * - Keeps collision-free nodes only
         * - Appends all valid nodes to output list
         */
        bool runLocalRRT(
            TrajOptimizationData& data,
            int robot_index,
            const std::vector<Eigen::Vector3d>& base_positions,
            const std::vector<Eigen::Vector3d>& base_orientations,
            int max_iterations,
            double step_angle_deg,
            double step_pos_mm,
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& rrt_nodes);

        static bool selectContinuousSolution(
            const std::vector<Eigen::Matrix4d>& solutions,
            const Eigen::Matrix4d& originalPose,
            const Eigen::Matrix4d* previousPose,
            Eigen::Matrix4d& selectedPose);

        // ============================================================================
        // Tolerance-search methods (moved from CollisionDetection)
        // ============================================================================

        /**
         * @brief Compute single-point tolerance
         *
         * For a given pose, searches the maximum collision-free tolerance range per axis
         * using binary search or step search.
         * Tolerance search is based on the spray point (workpiece surface point), not directly on trajectory points.
         *
         * @param data trajectory optimization data
         * @param pose original pose (4x4 homogeneous transform, trajectory-point pose)
         * @param[out] pos_tol position tolerance [min, max] (3x2 matrix, rows correspond to X/Y/Z)
         * @param[out] ori_tol orientation tolerance [min, max] (3x2 matrix, rows correspond to Rx/Ry/Rz)
         * @param tol_limit tolerance-limit config defining search range and precision
         * @param spray_distance spray distance (distance from trajectory point to workpiece surface, mm)
         */
        static void computeSinglePointTolerance(
            TrajOptimizationData& data,
            int robot_index,
            const Eigen::Matrix4d& pose,
            Eigen::Matrix<double, 3, 2>& pos_tol,
            Eigen::Matrix<double, 3, 2>& ori_tol,
            const ToleranceLimitConfig& tol_limit,
            double spray_distance = 0.0);

        /**
         * @brief Binary search: maximum collision-free translation distance
         *
         * Uses binary search to find the maximum collision-free translation distance in a given direction.
         * Applies translation on the spray point (workpiece surface point), then back-solves trajectory-point position.
         *
         * @param data trajectory optimization data
         * @param base_pose base pose (GLM format, trajectory-point pose)
         * @param direction search direction (normalized, direction on workpiece surface)
         * @param max_range maximum search range (mm)
         * @param tolerance search stop precision (mm)
         * @param spray_distance spray distance (distance from trajectory point to workpiece surface, mm)
         * @param tool_z_axis tool Z-axis direction (toward workpiece, for spray-point computation)
         * @return maximum collision-free distance found (mm)
         */
        static float searchLimit(
            TrajOptimizationData& data,
            int robot_index,
            const glm::mat4& base_pose,
            const glm::vec3& direction,
            float max_range,
            double tolerance,
            double spray_distance = 0.0,
            const glm::vec3& tool_z_axis = glm::vec3(0, 0, -1));

        /**
         * @brief Binary search: maximum collision-free rotation angle
         *
         * 使用二分法在指定轴上搜索最大的无碰撞旋转角度。
         * 围绕喷涂点旋转工具，然后根据新的工具姿态和spray_distance反推轨迹点位置。
         *
         * @param data trajectory optimization data
         * @param base_pose base pose (GLM format, trajectory-point pose)
         * @param axis rotation axis (normalized)
         * @param max_angle maximum search angle (radian, can be negative)
         * @param tolerance search stop precision (radian)
         * @param spray_distance spray distance (distance from trajectory point to workpiece surface, mm)
         * @param tool_z_axis tool Z-axis direction (toward workpiece, for spray-point computation)
         * @return maximum collision-free angle found (radian, signed)
         */
        static float searchRotLimit(
            TrajOptimizationData& data,
            int robot_index,
            const glm::mat4& base_pose,
            const glm::vec3& axis,
            float max_angle,
            double tolerance,
            double spray_distance = 0.0,
            const glm::vec3& tool_z_axis = glm::vec3(0, 0, -1));

        /**
         * @brief Step search: maximum collision-free translation distance
         *
         * Uses step search to find the maximum collision-free translation distance in a given direction.
         * Applies translation on spray point (workpiece surface point), then back-solves trajectory-point position.
         *
         * @param data trajectory optimization data
         * @param base_pose base pose (GLM format, trajectory-point pose)
         * @param direction search direction (normalized, direction on workpiece surface)
         * @param max_range maximum search range (mm)
         * @param step_size step size (mm)
         * @param spray_distance spray distance (distance from trajectory point to workpiece surface, mm)
         * @param tool_z_axis tool Z-axis direction (toward workpiece, for spray-point computation)
         * @return maximum collision-free distance found (mm)
         */
        static float searchLimitByStep(
            TrajOptimizationData& data,
            int robot_index,
            const glm::mat4& base_pose,
            const glm::vec3& direction,
            float max_range,
            double step_size,
            double spray_distance = 0.0,
            const glm::vec3& tool_z_axis = glm::vec3(0, 0, -1));

        /**
         * @brief Step search: maximum collision-free rotation angle
         *
         * Uses step search to find the maximum collision-free rotation angle around a given axis.
         * Rotates tool around spray point, then back-solves trajectory-point position using new tool orientation and spray_distance.
         *
         * @param data trajectory optimization data
         * @param base_pose base pose (GLM format, trajectory-point pose)
         * @param axis rotation axis (normalized)
         * @param max_angle maximum search angle (radian, can be negative)
         * @param step_size step size (radian)
         * @param spray_distance spray distance (distance from trajectory point to workpiece surface, mm)
         * @param tool_z_axis tool Z-axis direction (toward workpiece, for spray-point computation)
         * @return maximum collision-free angle found (radian, signed)
         */
        static float searchRotLimitByStep(
            TrajOptimizationData& data,
            int robot_index,
            const glm::mat4& base_pose,
            const glm::vec3& axis,
            float max_angle,
            double step_size,
            double spray_distance = 0.0,
            const glm::vec3& tool_z_axis = glm::vec3(0, 0, -1));

    private:
        // Internal helper functions are implemented in .cpp
    };

}