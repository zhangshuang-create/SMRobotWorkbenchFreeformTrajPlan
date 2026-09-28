#include <OptiAlgorithm/OptiAlgorithm.h>
#include <CollisionDetection/CollisionDetection.h>
#include <TrajOptimizationData/TrajOptimizationData.h>
#include <GeneralData/PTrajData.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <SMRobotTangQing/robot_factory.h>
#include <SMRobotTangQing/iloader.h>
#include <ModelTangQing/icollision_detector.h>
#include <limits>
#include <random>
#include <algorithm>
#include <memory>
#include <cmath>
#include <future>
#include <QDebug>

// Windows headers (for Sleep function)
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX  // Prevent Windows.h from defining min/max macros
#endif
#include <Windows.h>
#else
#include <unistd.h>
#endif

#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace ModelTrajPlan
{
// ============================================================================
// Helper functions: Eigen-to-GLM conversion (must be defined before use)
// ============================================================================

static glm::mat4 eigenMatrixToGlm(const Eigen::Matrix4d& m) {
    glm::mat4 result;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result[j][i] = static_cast<float>(m(i, j));
        }
    }
    return result;
}

static Eigen::Matrix4d glmMatrixToEigen(const glm::mat4& m) {
    Eigen::Matrix4d out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out(r, c) = static_cast<double>(m[c][r]); // GLM is column-major; convert back to row-major
        }
    }
    return out;
}

// ============================================================================
// CSpaceSTL-related structures (already defined in header)
// ============================================================================

// Triangle 构造函数实现
Triangle::Triangle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) 
    : v0(a), v1(b), v2(c) {
    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2_local = v2 - v0;
    normal = edge1.cross(edge2_local).normalized();
}

// ============================================================================
// RRT algorithm core (extracted from demo4)
// ============================================================================

// RRTNode 构造函数和成员函数实现
RRTNode::RRTNode() : parent(nullptr), node_id(-1) {}

RRTNode::RRTNode(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient)
    : position(pos), orientation(orient), parent(nullptr), node_id(-1) {}

Eigen::Matrix4d RRTNode::toMatrix4d() const {
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(orientation.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(orientation.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(orientation.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d R = Rz * Rx * Ry;
    
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = position;
    return T;
}

// Helper function: rotation matrix to Euler angles
static Eigen::Vector3d rotationMatrixToEuler(const Eigen::Matrix3d& R) {
    double sy = std::sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    
    double x, y, z;
    if (!singular) {
        x = std::atan2(R(2,1), R(2,2));
        y = std::atan2(-R(2,0), sy);
        z = std::atan2(R(1,0), R(0,0));
    } else {
        x = std::atan2(-R(1,2), R(1,1));
        y = std::atan2(-R(2,0), sy);
        z = 0;
    }
    return Eigen::Vector3d(x, y, z);
}

// ============================================================================
// ACO algorithm core
// ============================================================================

bool OptiAlgorithm::optimizeWithACO(
    TrajOptimizationData& data,
    int robot_index,
    const Eigen::Vector3d& original_tool_tcp_pos,  // Tool TCP position of original trajectory point (fixed)
    const Eigen::Vector3d& original_tool_z_axis,   // Tool Z-axis of original trajectory point (normalized, points to workpiece)
    const Eigen::Vector3d& goal_orient,             // Target orientation (Euler angles)
    double spray_distance,                          // Spray distance (tool-to-workpiece surface distance)
    std::vector<Eigen::Matrix4d>& solutions)
{
    // [Core logic] Compute fixed workpiece surface point (spray target point)
    // Trajectory point position is TCP position (tool center), and workpiece surface point = TCP - tool_z_axis * spray_distance
    // original_tool_z_axis points to the workpiece (-R.col(2)), so workpiece surface point = TCP - original_tool_z_axis * spray_distance
    // This workpiece surface point is the optimization reference and remains unchanged throughout optimization
    Eigen::Vector3d workpiece_surface_pos = original_tool_tcp_pos - original_tool_z_axis * spray_distance;
    
    const int num_ants = 30;
    const int num_iterations = 40;
    const int max_solutions = 8;
    
    double pheromone = 0.5;
    const double evap = 0.2;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    
    struct Sol {
        Eigen::Vector3d pos;
        Eigen::Vector3d orient;
        double cost;
    };
    std::vector<Sol> best_solutions;
    
    const double max_angle_rad = 90.0 * M_PI / 180.0;  // 最大90度
    
    for (int iter = 0; iter < num_iterations; ++iter) {
        for (int k = 0; k < num_ants; ++k) {
            double local_radius = 5.0 + 45.0 * (1.0 - pheromone);
            double delta_rad = local_radius * M_PI / 180.0;
            // Limit angle to within 90 degrees
            delta_rad = std::min(delta_rad, max_angle_rad);
            std::uniform_real_distribution<double> d(-delta_rad, delta_rad);
            
            Eigen::Vector3d orient = goal_orient;
            orient.x() += d(gen);
            orient.y() += d(gen);
            orient.z() += d(gen);
            
            // 限制角度不超过90度
            Eigen::Vector3d d_orient = orient - goal_orient;
            for (int i = 0; i < 3; ++i) {
                while (d_orient[i] > M_PI) d_orient[i] -= 2.0 * M_PI;
                while (d_orient[i] < -M_PI) d_orient[i] += 2.0 * M_PI;
            }
            if (d_orient.norm() > max_angle_rad) {
                // If over 90 degrees, scale back to 90 degrees
                d_orient = d_orient.normalized() * max_angle_rad;
            }
            orient = goal_orient + d_orient;
            
            // [Core logic] Compute tool TCP position by back-solving from fixed workpiece surface point (spray target)
            Eigen::Matrix3d Rz = Eigen::AngleAxisd(orient.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Eigen::Matrix3d Rx = Eigen::AngleAxisd(orient.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
            Eigen::Matrix3d Ry = Eigen::AngleAxisd(orient.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
            Eigen::Matrix3d R = Rz * Rx * Ry;
            
            // Tool Z-axis direction for new orientation (points to workpiece)
            Eigen::Vector3d new_tool_z_axis = -R.col(2).normalized();
            // Workpiece surface point = tool TCP - new_tool_z_axis * spray_distance
            // Therefore: tool TCP = workpiece surface point + new_tool_z_axis * spray_distance
            // Key: workpiece surface point and spray_distance remain fixed; only orientation changes (thus tool Z-axis changes)
            Eigen::Vector3d tool_tcp_pos = workpiece_surface_pos + new_tool_z_axis * spray_distance;
            
            // Compute position change (for fitness evaluation, not a hard constraint)
            // Note: orientation changes also change tool Z-axis, so TCP position will deviate from original position
            // This is expected because the workpiece surface point (spray target) must remain fixed
            Eigen::Vector3d pos_diff = tool_tcp_pos - original_tool_tcp_pos;
            double translation_along_normal = pos_diff.dot(original_tool_z_axis);
            Eigen::Vector3d perpendicular_component = pos_diff - original_tool_z_axis * translation_along_normal;
            double perpendicular_error = perpendicular_component.norm();
            
            // Build tool TCP pose matrix (for collision checking)
            Eigen::Matrix4d tool_tcp_pose = Eigen::Matrix4d::Identity();
            tool_tcp_pose.block<3,3>(0,0) = R;  // 使用新姿态
            tool_tcp_pose.block<3,1>(0,3) = tool_tcp_pos;  // 工具TCP位置
            
            // Collision check at tool TCP pose
            glm::mat4 gpose = eigenMatrixToGlm(tool_tcp_pose);
            CollisionDetection::set_robot_pose_matrix(data, robot_index, gpose);
            
            if (CollisionDetection::is_collision(data)) {
                continue;
            }
            
            // Compute fitness: orientation error (based on spray-target pose) + position error
            // Build spray-target pose matrix (for orientation assessment)
            // Spray-target pose = (workpiece surface position, new orientation)
            // Workpiece surface point remains fixed, ensuring spray target remains fixed
            Eigen::Matrix4d spray_point_pose = Eigen::Matrix4d::Identity();
            spray_point_pose.block<3,3>(0,0) = R;  // 使用新姿态
            spray_point_pose.block<3,1>(0,3) = workpiece_surface_pos;  // Workpiece surface position (spray target)
            
            // 姿态误差：喷涂目标点的姿态与目标姿态的差异
            double orient_cost = d_orient.norm() * 180.0 / M_PI;
            double position_penalty = perpendicular_error * 10.0;
            double total_cost = orient_cost + position_penalty;
            best_solutions.push_back({tool_tcp_pos, orient, total_cost});
        }
        
        if (!best_solutions.empty()) {
            std::sort(best_solutions.begin(), best_solutions.end(),
                      [](const Sol& a, const Sol& b) { return a.cost < b.cost; });
            int elite = std::min<int>(3, best_solutions.size());
            double improve = 0.0;
            for (int i = 0; i < elite; ++i) {
                improve += 1.0 / (1.0 + best_solutions[i].cost);
            }
            pheromone = (1.0 - evap) * pheromone + evap * std::min(1.0, improve);
        }
    }
    
    if (best_solutions.empty()) {
        return false;
    }
    
    std::sort(best_solutions.begin(), best_solutions.end(),
              [](const Sol& a, const Sol& b) { return a.cost < b.cost; });
    
    int keep = std::min<int>(best_solutions.size(), max_solutions);
    for (int i = 0; i < keep; ++i) {
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(best_solutions[i].orient.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(best_solutions[i].orient.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(best_solutions[i].orient.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d R = Rz * Rx * Ry;
        
        // [Correct logic] Directly use tool TCP position computed during evaluation
        Eigen::Vector3d tool_tcp_pos = best_solutions[i].pos;
        
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = tool_tcp_pos;
        
        // [Key] Re-validate collision: ensure no collision at tool TCP position (required)
        glm::mat4 gpose = eigenMatrixToGlm(T);
        CollisionDetection::set_robot_pose_matrix(data, robot_index, gpose);
        if (CollisionDetection::is_collision(data)) {
            // If colliding, skip this solution
            continue;
        }
        
        solutions.push_back(T);
    }
    
    // If all solutions collide, return false
    return !solutions.empty();
}

// ============================================================================
// GA algorithm core
// ============================================================================

// Individual 构造函数实现
Individual::Individual() : z_translation(0.0), fitness(std::numeric_limits<double>::max()), is_valid(false) {}

bool OptiAlgorithm::optimizeWithGA(
    TrajOptimizationData& data,
    int robot_index,
    const Eigen::Vector3d& original_tool_tcp_pos,  // Tool TCP position of original trajectory point (fixed)
    const Eigen::Vector3d& original_tool_z_axis,   // Tool Z-axis of original trajectory point (normalized, points to workpiece)
    const Eigen::Vector3d& goal_orient,             // Target orientation (Euler angles)
    double spray_distance,                          // Spray distance (tool-to-workpiece surface distance)
    bool enable_z_translation,
    double max_z_translation,
    std::vector<Eigen::Matrix4d>& solutions)
{
    // More aggressive search parameters for greater solution diversity and count
    const int population_size = 80;      // Was 50
    const int num_generations = 80;      // Was 40
    const int max_solutions = 12;        // Was 8
    const double mutation_rate = 0.25;   // Was 0.15
    const double crossover_rate = 0.9;   // Was 0.7
    const int elite_count = 8;           // Was 5
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    
    // [Core logic] Compute fixed workpiece surface point (spray target point)
    // Trajectory point position is TCP position (tool center), and workpiece surface point = TCP - tool_z_axis * spray_distance
    // original_tool_z_axis points to the workpiece (-R.col(2)), so workpiece surface point = TCP - original_tool_z_axis * spray_distance
    // This workpiece surface point is the optimization reference and remains unchanged throughout optimization
    Eigen::Vector3d workpiece_surface_pos = original_tool_tcp_pos - original_tool_z_axis * spray_distance;
    
    // Evaluate individuals
    // [Optimization strategy] optimize around fixed workpiece surface point (spray target point)
    // 1. Limit orientation adjustment within 90 degrees
    // 2. If z_translation is enabled, allow slight adjustment along original normal (adjusted point remains fixed during evaluation)
    // 3. Back-solve tool TCP position from the (possibly adjusted) workpiece point using the new tool Z-axis
    auto evaluateIndividual = [&](Individual& ind) {
        // 1. Limit orientation adjustment within 90 degrees
        Eigen::Vector3d d_orient = ind.orient - goal_orient;
        for (int i = 0; i < 3; ++i) {
            while (d_orient[i] > M_PI) d_orient[i] -= 2.0 * M_PI;
            while (d_orient[i] < -M_PI) d_orient[i] += 2.0 * M_PI;
        }
        double max_angle_rad = 90.0 * M_PI / 180.0;
        if (d_orient.norm() > max_angle_rad) {
            // Reject solutions whose angle exceeds 90 degrees
            ind.is_valid = false;
            ind.fitness = std::numeric_limits<double>::max();
            return;
        }
        
        // 2. Generate tool TCP pose from encoding (orientation only)
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(ind.orient.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(ind.orient.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(ind.orient.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d R = Rz * Rx * Ry;
        
        // 3. Tool Z-axis direction under new orientation (points to workpiece)
        Eigen::Vector3d new_tool_z_axis = -R.col(2).normalized();
        
        // 4. Compute tool TCP position by back-solving from fixed workpiece surface point (spray target)
        // Core logic: workpiece surface point must remain fixed regardless of orientation changes
        // - Workpiece point = tool TCP - tool_z_axis * spray_distance
        // - Therefore: tool TCP = workpiece point + tool_z_axis * spray_distance
        // - z_translation offsets along new tool Z-axis and only adjusts TCP-to-workpiece distance
        //   The workpiece point always remains fixed at workpiece_surface_pos
        double effective_spray_distance = spray_distance;
        if (enable_z_translation) {
            // z_translation adjusts TCP-to-workpiece distance along new tool Z-axis
            // Workpiece point = new_tool_tcp_pos - new_tool_z_axis * spray_distance = workpiece_surface_pos (fixed)
            effective_spray_distance += ind.z_translation;
        }
        // Back-solve TCP position from fixed workpiece point using new tool Z-axis
        // Key: workpiece point always remains workpiece_surface_pos regardless of orientation changes
        Eigen::Vector3d new_tool_tcp_pos = workpiece_surface_pos + new_tool_z_axis * effective_spray_distance;
        
        // 6. Compute position change (for fitness, not hard constraint)
        // Note: orientation changes alter tool Z-axis, so TCP position naturally deviates from original position
        // This is expected because the workpiece surface point (spray target) must remain fixed
        Eigen::Vector3d pos_diff = new_tool_tcp_pos - original_tool_tcp_pos;
        // Projection along original normal (used for Z-translation constraint)
        double translation_along_normal = pos_diff.dot(original_tool_z_axis);
        // Component perpendicular to original normal (fitness penalty to reduce deviation)
        Eigen::Vector3d perpendicular_component = pos_diff - original_tool_z_axis * translation_along_normal;
        double perpendicular_error = perpendicular_component.norm();
        
        // 7. Constrain z_translation range (if Z translation is enabled)
        // z_translation adjusts distance along tool-TCP frame Z direction (spray direction)
        if (enable_z_translation) {
            if (std::abs(ind.z_translation) > max_z_translation) {
                ind.is_valid = false;
                ind.fitness = std::numeric_limits<double>::max();
                return;
            }
        }
        // If Z translation is disabled, do not constrain position change (orientation changes naturally induce position change)
        
        // 8. Build tool TCP pose matrix (for collision checking)
        Eigen::Matrix4d tool_tcp_pose = Eigen::Matrix4d::Identity();
        tool_tcp_pose.block<3,3>(0,0) = R;  // New orientation
        tool_tcp_pose.block<3,1>(0,3) = new_tool_tcp_pos;  // Tool TCP position
        
        // 9. Collision check at tool TCP pose
        glm::mat4 gpose = eigenMatrixToGlm(tool_tcp_pose);
        CollisionDetection::set_robot_pose_matrix(data, robot_index, gpose);
        
        if (CollisionDetection::is_collision(data)) {
            ind.is_valid = false;
            ind.fitness = std::numeric_limits<double>::max();
            return;
        }
        
        // 10. Compute fitness: orientation error (spray-target pose based) + position penalty (smaller is better)
        // Build spray-target pose matrix (for orientation evaluation)
        // Spray-target pose = (workpiece surface position, new orientation)
        // Workpiece surface point remains fixed to keep spray target fixed
        Eigen::Matrix4d spray_point_pose = Eigen::Matrix4d::Identity();
        spray_point_pose.block<3,3>(0,0) = R;  // New orientation
        spray_point_pose.block<3,1>(0,3) = workpiece_surface_pos;  // Workpiece surface position (spray target), fully fixed
        
        // Orientation error: difference between spray-point orientation and target orientation
        // Target orientation should represent expected spray-point orientation (derived from original trajectory point)
        double orient_cost = d_orient.norm() * 180.0 / M_PI;
        
        // Position-error penalty (error perpendicular to original normal)
        double position_penalty = perpendicular_error * 10.0;  // Each mm adds 10-degree equivalent cost
        
        // Z-translation penalty: each mm adds 0.1-degree equivalent cost (encourages smaller translation)
        double translation_penalty = enable_z_translation ? std::abs(ind.z_translation) * 0.1 : 0.0;
        
        ind.fitness = orient_cost + position_penalty + translation_penalty;
        ind.is_valid = true;
    };
    
    // Tournament selection
    auto tournamentSelection = [&](const std::vector<Individual>& population, int tournament_size) -> Individual {
        std::uniform_int_distribution<int> dist(0, static_cast<int>(population.size()) - 1);
        Individual best = population[dist(gen)];
        for (int i = 1; i < tournament_size; ++i) {
            const Individual& candidate = population[dist(gen)];
            if ((candidate.is_valid && !best.is_valid) ||
                (candidate.is_valid == best.is_valid && candidate.fitness < best.fitness)) {
                best = candidate;
            }
        }
        return best;
    };
    
    // Crossover
    auto crossover = [&](const Individual& p1, const Individual& p2, Individual& c1, Individual& c2) {
        double alpha = uni01(gen);
        c1.orient = alpha * p1.orient + (1.0 - alpha) * p2.orient;
        c2.orient = (1.0 - alpha) * p1.orient + alpha * p2.orient;
        
        // Apply crossover to Z-translation as well
        if (enable_z_translation) {
            c1.z_translation = alpha * p1.z_translation + (1.0 - alpha) * p2.z_translation;
            c2.z_translation = (1.0 - alpha) * p1.z_translation + alpha * p2.z_translation;
        }
    };
    
    // Mutation (limit orientation within 90 degrees)
    auto mutate = [&](Individual& ind) {
        const double mutation_scale = 25.0 * M_PI / 180.0;  // Mutation range: 25 degrees
        std::uniform_real_distribution<double> mut(-mutation_scale, mutation_scale);
        ind.orient.x() += mut(gen);
        ind.orient.y() += mut(gen);
        ind.orient.z() += mut(gen);
        
        // Limit orientation within 90 degrees
        Eigen::Vector3d d_orient = ind.orient - goal_orient;
        for (int i = 0; i < 3; ++i) {
            while (d_orient[i] > M_PI) d_orient[i] -= 2.0 * M_PI;
            while (d_orient[i] < -M_PI) d_orient[i] += 2.0 * M_PI;
        }
        double max_angle_rad = 90.0 * M_PI / 180.0;
        if (d_orient.norm() > max_angle_rad) {
            // If over 90 degrees, scale back to 90 degrees
            d_orient = d_orient.normalized() * max_angle_rad;
        }
        ind.orient = goal_orient + d_orient;
        
        // Apply mutation to Z-translation as well
        if (enable_z_translation) {
            const double z_mutation_scale = max_z_translation * 0.3;  // Mutation range is 30% of max translation
            std::uniform_real_distribution<double> z_mut(-z_mutation_scale, z_mutation_scale);
            ind.z_translation += z_mut(gen);
            // Clamp into allowed range
            ind.z_translation = std::max(-max_z_translation, std::min(max_z_translation, ind.z_translation));
        }
    };
    
    // Initialize population (limit orientation within 90 degrees)
    std::vector<Individual> population(population_size);
    for (auto& ind : population) {
        double max_perturb = 90.0 * M_PI / 180.0;  // 最大90度
        std::uniform_real_distribution<double> perturb(-max_perturb, max_perturb);
        ind.orient = goal_orient;
        ind.orient.x() += perturb(gen);
        ind.orient.y() += perturb(gen);
        ind.orient.z() += perturb(gen);
        
        // Limit orientation within 90 degrees
        Eigen::Vector3d d_orient = ind.orient - goal_orient;
        for (int i = 0; i < 3; ++i) {
            while (d_orient[i] > M_PI) d_orient[i] -= 2.0 * M_PI;
            while (d_orient[i] < -M_PI) d_orient[i] += 2.0 * M_PI;
        }
        if (d_orient.norm() > max_perturb) {
            // If over 90 degrees, scale back to 90 degrees
            d_orient = d_orient.normalized() * max_perturb;
        }
        ind.orient = goal_orient + d_orient;
        
        // Initialize Z translation: random within allowed range when enabled
        if (enable_z_translation) {
            std::uniform_real_distribution<double> z_trans(-max_z_translation, max_z_translation);
            ind.z_translation = z_trans(gen);
        } else {
            ind.z_translation = 0.0;
        }
    }
    
    for (auto& ind : population) {
        evaluateIndividual(ind);
    }
    
    // Main loop: multi-generation evolution
    for (int generation = 0; generation < num_generations; ++generation) {
        std::vector<Individual> new_population;
        new_population.reserve(population_size);
        
        std::sort(population.begin(), population.end(),
                  [](const Individual& a, const Individual& b) {
                      if (a.is_valid != b.is_valid) return a.is_valid > b.is_valid;
                      return a.fitness < b.fitness;
                  });
        for (int i = 0; i < elite_count && i < static_cast<int>(population.size()); ++i) {
            if (population[i].is_valid) {
                new_population.push_back(population[i]);
            }
        }
        
        while (static_cast<int>(new_population.size()) < population_size) {
            Individual parent1 = tournamentSelection(population, 3);
            Individual parent2 = tournamentSelection(population, 3);
            
            Individual child1, child2;
            if (uni01(gen) < crossover_rate) {
                crossover(parent1, parent2, child1, child2);
            } else {
                child1 = parent1;
                child2 = parent2;
            }
            
            if (uni01(gen) < mutation_rate) {
                mutate(child1);
            }
            if (uni01(gen) < mutation_rate) {
                mutate(child2);
            }
            
            evaluateIndividual(child1);
            evaluateIndividual(child2);
            
            new_population.push_back(child1);
            if (static_cast<int>(new_population.size()) < population_size) {
                new_population.push_back(child2);
            }
        }
        
        population = new_population;
    }
    
    // Collect all collision-free solutions
    std::vector<Individual> valid_solutions;
    for (const auto& ind : population) {
        if (ind.is_valid) {
            valid_solutions.push_back(ind);
        }
    }
    
    if (valid_solutions.empty()) {
        return false;
    }
    
    // Sort by fitness and keep top solutions
    std::sort(valid_solutions.begin(), valid_solutions.end(),
              [](const Individual& a, const Individual& b) {
                  return a.fitness < b.fitness;
              });
    
    int keep = std::min<int>(valid_solutions.size(), max_solutions);
    for (int i = 0; i < keep; ++i) {
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(valid_solutions[i].orient.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(valid_solutions[i].orient.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(valid_solutions[i].orient.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d R = Rz * Rx * Ry;
        
        // [Correct logic] Compute tool TCP by back-solving from fixed workpiece surface point (spray target)
        Eigen::Vector3d new_tool_z_axis = -R.col(2).normalized();
        // z_translation adjusts TCP-to-workpiece distance along new tool Z-axis, while workpiece point remains fixed
        double effective_spray_distance = spray_distance;
        if (enable_z_translation) {
            effective_spray_distance += valid_solutions[i].z_translation;
        }
        // Back-solve tool TCP from fixed workpiece point using new orientation Z-axis
        Eigen::Vector3d tool_tcp_pos = workpiece_surface_pos + new_tool_z_axis * effective_spray_distance;
        
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = tool_tcp_pos;
        
        // [Key] Re-validate collision: ensure no collision at tool TCP pose (required)
        glm::mat4 gpose = eigenMatrixToGlm(T);
        CollisionDetection::set_robot_pose_matrix(data, robot_index, gpose);
        if (CollisionDetection::is_collision(data)) {
            // If colliding, skip this solution
            continue;
        }
        
        solutions.push_back(T);
    }
    
    // If all solutions collide, return false
    return !solutions.empty();
}

// ============================================================================
// Enhanced island GA + RRT hybrid algorithm (independent of existing GA)
// ============================================================================
bool OptiAlgorithm::optimizeWithGAIslandsRRT(
    TrajOptimizationData& data,
    int robot_index,
    const Eigen::Vector3d& original_tool_tcp_pos,  // Tool TCP position of original trajectory point (fixed)
    const Eigen::Vector3d& original_tool_z_axis,   // Tool Z-axis of original trajectory point (normalized, points to workpiece)
    const Eigen::Vector3d& goal_orient,             // Target orientation (Euler angles)
    double spray_distance,                          // Spray distance (tool-to-workpiece surface distance)
    bool enable_z_translation,
    double max_z_translation,
    std::vector<Eigen::Matrix4d>& solutions)
{
    solutions.clear();

    const int island_count = 5;        // 5 independent populations (reduced to lower reload overhead)
    const int population_size = 40;     // Per-population size (reduced for speed)
    const int num_generations = 50;     // Iteration generations (reduced for speed)
    const int elite_count = 4;
    const double mutation_rate = 0.25;
    const double crossover_rate = 0.9;
    const int tournament_size = 3;

    // [Core logic] Compute fixed workpiece surface point (spray target point)
    // Trajectory point position is TCP position (tool center), and workpiece surface point = TCP - tool_z_axis * spray_distance
    // original_tool_z_axis points to the workpiece (-R.col(2)), so workpiece surface point = TCP - original_tool_z_axis * spray_distance
    // This workpiece surface point is the optimization reference and remains unchanged throughout optimization
    Eigen::Vector3d workpiece_surface_pos = original_tool_tcp_pos - original_tool_z_axis * spray_distance;
    
    // Evaluation function (reuses existing logic), allows specified detector
    // [Optimization strategy] optimize around fixed workpiece surface point (spray target point)
    // 1. Limit orientation adjustment within 90 degrees
    // 2. If z_translation is enabled, allow slight adjustment along original normal (adjusted point remains fixed during evaluation)
    // 3. Back-solve tool TCP position from the (possibly adjusted) workpiece point using the new tool Z-axis
    auto evaluateIndividual = [&](Individual& ind, TrajOptimizationData& localData) {
        // 1. Limit orientation adjustment within 90 degrees
        Eigen::Vector3d d_orient = ind.orient - goal_orient;
        for (int i = 0; i < 3; ++i) {
            while (d_orient[i] > M_PI) d_orient[i] -= 2.0 * M_PI;
            while (d_orient[i] < -M_PI) d_orient[i] += 2.0 * M_PI;
        }
        double max_angle_rad = 90.0 * M_PI / 180.0;
        if (d_orient.norm() > max_angle_rad) {
            // Reject solutions whose angle exceeds 90 degrees
            ind.is_valid = false;
            ind.fitness = std::numeric_limits<double>::max();
            return;
        }
        
        // 2. Generate tool TCP pose from encoding (orientation only)
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(ind.orient.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(ind.orient.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(ind.orient.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d R = Rz * Rx * Ry;

        // 3. Tool Z-axis direction under new orientation (points to workpiece)
        Eigen::Vector3d new_tool_z_axis = -R.col(2).normalized();
        
        // 4. Compute tool TCP position by back-solving from fixed workpiece surface point (spray target)
        // Core logic: workpiece surface point must remain fixed regardless of orientation changes
        // - Workpiece point = tool TCP - tool_z_axis * spray_distance
        // - Therefore: tool TCP = workpiece point + tool_z_axis * spray_distance
        // - z_translation offsets along new tool Z-axis and only adjusts TCP-to-workpiece distance
        //   The workpiece point always remains fixed at workpiece_surface_pos
        double effective_spray_distance = spray_distance;
        if (enable_z_translation) {
            // z_translation adjusts TCP-to-workpiece distance along new tool Z-axis
            // Workpiece point = new_tool_tcp_pos - new_tool_z_axis * spray_distance = workpiece_surface_pos (fixed)
            effective_spray_distance += ind.z_translation;
        }
        // Back-solve TCP position from fixed workpiece point using new tool Z-axis
        // Key: workpiece point always remains workpiece_surface_pos regardless of orientation changes
        Eigen::Vector3d new_tool_tcp_pos = workpiece_surface_pos + new_tool_z_axis * effective_spray_distance;

        // 6. Compute position change (for fitness, not hard constraint)
        // Note: orientation changes alter tool Z-axis, so TCP position naturally deviates from original position
        // This is expected because the workpiece surface point (spray target) must remain fixed
        Eigen::Vector3d pos_diff = new_tool_tcp_pos - original_tool_tcp_pos;
        // Projection along original normal (used for Z-translation constraint)
        double translation_along_normal = pos_diff.dot(original_tool_z_axis);
        // Component perpendicular to original normal (fitness penalty to reduce deviation)
        Eigen::Vector3d perpendicular_component = pos_diff - original_tool_z_axis * translation_along_normal;
        double perpendicular_error = perpendicular_component.norm();
        
        // 7. Constrain z_translation range (if Z translation is enabled)
        // z_translation adjusts distance along tool-TCP frame Z direction (spray direction)
        if (enable_z_translation) {
            if (std::abs(ind.z_translation) > max_z_translation) {
                ind.is_valid = false;
                ind.fitness = std::numeric_limits<double>::max();
                return;
            }
        }
        // If Z translation is disabled, do not constrain position change (orientation changes naturally induce position change)

        // 8. Build tool TCP pose matrix (for collision checking)
        Eigen::Matrix4d tool_tcp_pose = Eigen::Matrix4d::Identity();
        tool_tcp_pose.block<3,3>(0,0) = R;  // New orientation
        tool_tcp_pose.block<3,1>(0,3) = new_tool_tcp_pos;  // Tool TCP position

        // 9. Collision check at tool TCP pose
        glm::mat4 gpose = eigenMatrixToGlm(tool_tcp_pose);
        CollisionDetection::set_robot_pose_matrix(localData, robot_index, gpose);
        if (CollisionDetection::is_collision(localData)) {
            ind.is_valid = false;
            ind.fitness = std::numeric_limits<double>::max();
            return;
        }

        // 10. Compute fitness: orientation error (spray-target pose based) + position penalty (smaller is better)
        // Build spray-target pose matrix (for orientation evaluation)
        // Spray-target pose = (workpiece surface position, new orientation)
        // Workpiece surface point remains fixed to keep spray target fixed
        Eigen::Matrix4d spray_point_pose = Eigen::Matrix4d::Identity();
        spray_point_pose.block<3,3>(0,0) = R;  // New orientation
        spray_point_pose.block<3,1>(0,3) = workpiece_surface_pos;  // Workpiece surface position (spray target), fully fixed
        
        // Orientation error: difference between spray-target orientation and target orientation
        double orient_cost = d_orient.norm() * 180.0 / M_PI;
        
        // Position-error penalty (error perpendicular to original normal)
        double position_penalty = perpendicular_error * 10.0;  // Each mm adds 10-degree equivalent cost
        
        // Z-translation penalty: each mm adds 0.1-degree equivalent cost (encourages smaller translation)
        double translation_penalty = enable_z_translation ? std::abs(ind.z_translation) * 0.1 : 0.0;
        ind.fitness = orient_cost + position_penalty + translation_penalty;
        ind.is_valid = true;
    };

    auto tournamentSelection = [&](const std::vector<Individual>& population, int t_size, std::mt19937& gen) -> Individual {
        std::uniform_int_distribution<int> dist(0, static_cast<int>(population.size()) - 1);
        Individual best = population[dist(gen)];
        for (int i = 1; i < t_size; ++i) {
            const Individual& cand = population[dist(gen)];
            if ((cand.is_valid && !best.is_valid) ||
                (cand.is_valid == best.is_valid && cand.fitness < best.fitness)) {
                best = cand;
            }
        }
        return best;
    };

    auto crossover = [&](const Individual& p1, const Individual& p2, Individual& c1, Individual& c2, std::mt19937& gen) {
        std::uniform_real_distribution<double> uni01(0.0, 1.0);
        double alpha = uni01(gen);
        c1.orient = alpha * p1.orient + (1.0 - alpha) * p2.orient;
        c2.orient = (1.0 - alpha) * p1.orient + alpha * p2.orient;
        if (enable_z_translation) {
            c1.z_translation = alpha * p1.z_translation + (1.0 - alpha) * p2.z_translation;
            c2.z_translation = (1.0 - alpha) * p1.z_translation + alpha * p2.z_translation;
        }
    };

    auto mutate = [&](Individual& ind, std::mt19937& gen) {
        const double mutation_scale = 25.0 * M_PI / 180.0;
        std::uniform_real_distribution<double> mut(-mutation_scale, mutation_scale);
        ind.orient.x() += mut(gen);
        ind.orient.y() += mut(gen);
        ind.orient.z() += mut(gen);
        
        // 限制角度不超过90度
        Eigen::Vector3d d_orient = ind.orient - goal_orient;
        for (int i = 0; i < 3; ++i) {
            while (d_orient[i] > M_PI) d_orient[i] -= 2.0 * M_PI;
            while (d_orient[i] < -M_PI) d_orient[i] += 2.0 * M_PI;
        }
        double max_angle_rad = 90.0 * M_PI / 180.0;
        if (d_orient.norm() > max_angle_rad) {
            // 如果超过90度，缩放到90度
            d_orient = d_orient.normalized() * max_angle_rad;
        }
        ind.orient = goal_orient + d_orient;
        
        if (enable_z_translation) {
            const double z_mutation_scale = max_z_translation * 0.3;
            std::uniform_real_distribution<double> z_mut(-z_mutation_scale, z_mutation_scale);
            ind.z_translation += z_mut(gen);
            ind.z_translation = std::max(-max_z_translation, std::min(max_z_translation, ind.z_translation));
        }
    };

    // 单岛进化，返回最佳个体
    auto runIsland = [&](int seed, icollis_spointer detector) -> Individual {
        std::mt19937 gen(seed);
        std::uniform_real_distribution<double> uni01(0.0, 1.0);
        std::vector<Individual> population(population_size);
        double max_perturb = 90.0 * M_PI / 180.0;
        std::uniform_real_distribution<double> perturb(-max_perturb, max_perturb);

        // 为本岛构造局部数据副本，绑定独立检测器
        TrajOptimizationData localData = data;      // 浅拷贝结构
        localData.m_spDetector = detector ? detector : data.m_spDetector;
        localData.externalDetectorVec = nullptr;    // 避免外部并发干扰
        localData.m_internalDetectors.clear();      // 不在岛内再分发
        
        // 为每个岛屿创建独立的机器人对象副本（避免多线程竞争，实现真正的并行）
        // 使用 robotModelPaths_ 重新加载机器人
        if (!data.robotModelPaths_.empty() && data.m_spScene) {
            localData.m_robots.clear();
            for (const std::string& fullpath : data.robotModelPaths_) {
                if (fullpath.empty()) continue;
                
                irobot_spointer robot = robot_factory::get_new_robot();
                if (!robot) continue;
                
                bool load_ok = false;
                if (fullpath.find(".xml") != std::string::npos) {
                    size_t pos = fullpath.find_last_of("/\\");
                    std::string folder = (pos != std::string::npos) ? fullpath.substr(0, pos) : ".";
                    std::string file = (pos != std::string::npos) ? fullpath.substr(pos + 1) : fullpath;
                    
                    iloader_spointer sp_loader = robot_factory::get_new_loader();
                    sp_loader->import(file.c_str(), folder.c_str());
                    std::vector<irobot_spointer> robots = sp_loader->compile_robot(data.m_spScene);
                    if (!robots.empty()) {
                        robot = robots[0];
                        load_ok = true;
                    }
                } else if (fullpath.find(".rbt") != std::string::npos) {
                    load_ok = robot->load(fullpath.c_str(), data.m_spScene);
                }
                
                if (load_ok) {
                    std::string robot_name = "Robot_" + std::to_string(localData.m_robots.size() + 1) + "_Island_" + std::to_string(seed);
                    robot->set_robot_name(robot_name);
                    robot->update_instance_name();
                    glm::mat4 base = glm::mat4(1.0f);
                    robot->set_base_mat(base);
                    localData.m_robots.push_back(robot);
                }
            }
            
            // 为每个岛屿创建新的碰撞检测器（使用新的机器人对象）
            if (!localData.m_robots.empty()) {
                // 创建新的检测器（因为检测器不支持清空，必须创建新的）
                localData.m_spDetector = icollision_detector::get_new_detector();
                if (localData.m_spDetector) {
                    // 收集所有机器人模型
                    std::vector<std::vector<imodel_spointer>> all_robot_models(localData.m_robots.size());
                    for (size_t i = 0; i < localData.m_robots.size(); ++i) {
                        CollisionDetection::collect_all_models(localData.m_robots[i], all_robot_models[i]);
                    }
                    
                    // 插入碰撞对到新检测器
                    for (size_t i = 0; i < all_robot_models.size(); ++i) {
                        for (size_t j = i + 1; j < all_robot_models.size(); ++j) {
                            for (auto& m1 : all_robot_models[i]) {
                                for (auto& m2 : all_robot_models[j]) {
                                    localData.m_spDetector->insert(m1, m2);
                                }
                            }
                        }
                    }
                }
            }
        }

        for (auto& ind : population) {
            ind.orient = goal_orient;
            ind.orient.x() += perturb(gen);
            ind.orient.y() += perturb(gen);
            ind.orient.z() += perturb(gen);
            
            // 限制角度不超过90度
            Eigen::Vector3d d_orient = ind.orient - goal_orient;
            for (int i = 0; i < 3; ++i) {
                while (d_orient[i] > M_PI) d_orient[i] -= 2.0 * M_PI;
                while (d_orient[i] < -M_PI) d_orient[i] += 2.0 * M_PI;
            }
            double max_angle_rad = 90.0 * M_PI / 180.0;
            if (d_orient.norm() > max_angle_rad) {
                // 如果超过90度，缩放到90度
                d_orient = d_orient.normalized() * max_angle_rad;
            }
            ind.orient = goal_orient + d_orient;
            
            if (enable_z_translation) {
                std::uniform_real_distribution<double> z_trans(-max_z_translation, max_z_translation);
                ind.z_translation = z_trans(gen);
            } else {
                ind.z_translation = 0.0;
            }
            evaluateIndividual(ind, localData);
        }

        for (int g = 0; g < num_generations; ++g) {
            std::vector<Individual> new_pop;
            new_pop.reserve(population_size);

            std::sort(population.begin(), population.end(),
                      [](const Individual& a, const Individual& b) {
                          if (a.is_valid != b.is_valid) return a.is_valid > b.is_valid;
                          return a.fitness < b.fitness;
                      });
            for (int i = 0; i < elite_count && i < static_cast<int>(population.size()); ++i) {
                if (population[i].is_valid) new_pop.push_back(population[i]);
            }

            while (static_cast<int>(new_pop.size()) < population_size) {
                Individual p1 = tournamentSelection(population, tournament_size, gen);
                Individual p2 = tournamentSelection(population, tournament_size, gen);
                Individual c1, c2;
                if (uni01(gen) < crossover_rate) crossover(p1, p2, c1, c2, gen);
                else { c1 = p1; c2 = p2; }
                if (uni01(gen) < mutation_rate) mutate(c1, gen);
                if (uni01(gen) < mutation_rate) mutate(c2, gen);
                evaluateIndividual(c1, localData);
                evaluateIndividual(c2, localData);
                new_pop.push_back(c1);
                if (static_cast<int>(new_pop.size()) < population_size) new_pop.push_back(c2);
            }

            population = new_pop;
        }

        // 取最佳
        std::sort(population.begin(), population.end(),
                  [](const Individual& a, const Individual& b) {
                      if (a.is_valid != b.is_valid) return a.is_valid > b.is_valid;
                      return a.fitness < b.fitness;
                  });
        return population.front();
    };

    // 并行跑 10 个岛（OpenMP 或 std::async）
    std::vector<Individual> island_bests(island_count);
    std::random_device rd;

#ifdef _OPENMP
    int detPoolSize = static_cast<int>(data.m_internalDetectors.size());
    int maxThreads = detPoolSize > 0 ? detPoolSize : 1;
    int threadsToUse = std::min(island_count, maxThreads);
#pragma omp parallel for num_threads(threadsToUse)
    for (int i = 0; i < island_count; ++i) {
        int detIdx = (detPoolSize > 0) ? (i % detPoolSize) : -1;
        icollis_spointer det = (detIdx >= 0) ? data.m_internalDetectors[detIdx] : data.m_spDetector;
        island_bests[i] = runIsland(static_cast<int>(rd()), det);
    }
#else
    std::vector<std::future<Individual>> futures;
    futures.reserve(island_count);
    int detPoolSize = static_cast<int>(data.m_internalDetectors.size());
    for (int i = 0; i < island_count; ++i) {
        int detIdx = (detPoolSize > 0) ? (i % detPoolSize) : -1;
        icollis_spointer det = (detIdx >= 0) ? data.m_internalDetectors[detIdx] : data.m_spDetector;
        futures.emplace_back(std::async(std::launch::async, runIsland, static_cast<int>(rd()), det));
    }
    for (int i = 0; i < island_count; ++i) {
        island_bests[i] = futures[i].get();
    }
#endif

    // Filter valid solutions
    island_bests.erase(
        std::remove_if(island_bests.begin(), island_bests.end(),
                       [](const Individual& ind) { return !ind.is_valid; }),
        island_bests.end());

    if (island_bests.empty()) return false;

    // Select global best
    std::sort(island_bests.begin(), island_bests.end(),
              [](const Individual& a, const Individual& b) { return a.fitness < b.fitness; });
    Individual best = island_bests.front();

    // Append each island's best solution to `solutions` (including global best)
    for (const auto& ind : island_bests) {
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(ind.orient.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(ind.orient.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(ind.orient.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d R = Rz * Rx * Ry;
        // [Core logic] Compute tool TCP by back-solving from fixed workpiece surface point (spray target)
        Eigen::Vector3d new_tool_z_axis = -R.col(2).normalized();
        // z_translation adjusts TCP-to-workpiece distance along new tool Z-axis, while workpiece point remains fixed
        double effective_spray_distance = spray_distance;
        if (enable_z_translation) {
            effective_spray_distance += ind.z_translation;
        }
        // Back-solve tool TCP from fixed workpiece point using new orientation Z-axis
        Eigen::Vector3d tool_tcp_pos = workpiece_surface_pos + new_tool_z_axis * effective_spray_distance;
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = tool_tcp_pos;
        
        // [Key] Re-validate collision: ensure no collision at tool TCP pose (required)
        glm::mat4 gpose = eigenMatrixToGlm(T);
        CollisionDetection::set_robot_pose_matrix(data, robot_index, gpose);
        if (CollisionDetection::is_collision(data)) {
            // If colliding, skip this solution
            continue;
        }
        
        solutions.push_back(T);
    }

    // Use best solutions as RRT seeds for local expansion
    std::vector<Eigen::Vector3d> base_positions;
    std::vector<Eigen::Vector3d> base_orients;
    for (const auto& ind : island_bests) {
        // [Correct logic] Compute tool TCP by back-solving from spray point using new orientation Z-axis
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(ind.orient.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(ind.orient.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(ind.orient.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d R = Rz * Rx * Ry;
        Eigen::Vector3d new_tool_z_axis = -R.col(2).normalized();
        // z_translation adjusts TCP-to-workpiece distance along new tool Z-axis, while workpiece point remains fixed
        double effective_spray_distance = spray_distance;
        if (enable_z_translation) {
            effective_spray_distance += ind.z_translation;
        }
        // Back-solve tool TCP from fixed workpiece point using new orientation Z-axis
        Eigen::Vector3d tool_tcp_pos = workpiece_surface_pos + new_tool_z_axis * effective_spray_distance;
        base_positions.push_back(tool_tcp_pos);
        base_orients.push_back(ind.orient);
    }

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rrt_nodes;
    runLocalRRT(data, robot_index, base_positions, base_orients,
                /*max_iterations*/200, /*step_angle_deg*/5.0, /*step_pos_mm*/10.0, rrt_nodes);

    for (const auto& node : rrt_nodes) {
        Eigen::Vector3d pos = node.first;
        Eigen::Vector3d orient = node.second;
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(orient.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(orient.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(orient.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d R = Rz * Rx * Ry;
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = pos;
        
        // [Key] Re-validate collision: ensure no collision at tool TCP pose (required)
        glm::mat4 gpose = eigenMatrixToGlm(T);
        CollisionDetection::set_robot_pose_matrix(data, robot_index, gpose);
        if (CollisionDetection::is_collision(data)) {
            // If colliding, skip this solution
            continue;
        }
        
        solutions.push_back(T);
    }

    return !solutions.empty();
}

// ============================================================================
// OptiAlgorithm 实现
// ============================================================================

OptiAlgorithm::OptiAlgorithm()
{
#ifdef _OPENMP
    qDebug() << "[OptiAlgorithm] OpenMP enabled, version:" << _OPENMP;
#else
    qDebug() << "[OptiAlgorithm] OpenMP NOT enabled";
#endif
}

OptiAlgorithm::~OptiAlgorithm()
{
}

bool OptiAlgorithm::selectContinuousSolution(
    const std::vector<Eigen::Matrix4d>& solutions,
    const Eigen::Matrix4d& originalPose,
    const Eigen::Matrix4d* previousPose,
    Eigen::Matrix4d& selectedPose)
{
    if (solutions.empty()) {
        return false;
    }

    const auto rotationDistance = [](const Eigen::Matrix4d& a, const Eigen::Matrix4d& b) {
        Eigen::Quaterniond qa(a.block<3, 3>(0, 0));
        Eigen::Quaterniond qb(b.block<3, 3>(0, 0));
        qa.normalize();
        qb.normalize();
        return 2.0 * std::acos(std::min(1.0, std::abs(qa.dot(qb))));
    };

    double bestCost = std::numeric_limits<double>::infinity();
    for (const Eigen::Matrix4d& candidate : solutions) {
        double cost = rotationDistance(candidate, originalPose) +
            0.02 * (candidate.block<3, 1>(0, 3) - originalPose.block<3, 1>(0, 3)).norm();
        if (previousPose) {
            cost += 2.0 * rotationDistance(candidate, *previousPose) +
                0.04 * (candidate.block<3, 1>(0, 3) - previousPose->block<3, 1>(0, 3)).norm();
        }
        if (cost < bestCost) {
            bestCost = cost;
            selectedPose = candidate;
        }
    }
    return true;
}

// ============================================================================
// Local RRT expansion algorithm
// ============================================================================

bool OptiAlgorithm::runLocalRRT(
    TrajOptimizationData& data,
    int robot_index,
    const std::vector<Eigen::Vector3d>& base_positions,
    const std::vector<Eigen::Vector3d>& base_orientations,
    int max_iterations,
    double step_angle_deg,
    double step_pos_mm,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& rrt_nodes)
{
    rrt_nodes.clear();
    
    if (base_orientations.empty()) {
        return false;
    }
    
    // Internal node structure
    struct Node {
        Eigen::Vector3d pos;
        Eigen::Vector3d orient;
    };
    std::vector<Node> nodes;
    
    // Initialization: use base solutions as root nodes
    for (size_t i = 0; i < base_orientations.size(); ++i) {
        Eigen::Vector3d p = (i < base_positions.size()) ? base_positions[i] : base_positions[0];
        nodes.push_back({p, base_orientations[i]});
        rrt_nodes.push_back({p, base_orientations[i]});
    }
    
    const double step_rad = step_angle_deg * M_PI / 180.0;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> d_angle(-step_rad, step_rad);
    std::uniform_real_distribution<double> d_pos(-step_pos_mm, step_pos_mm);
    std::uniform_int_distribution<int> pick_root(0, static_cast<int>(base_orientations.size()) - 1);
    
    // Local RRT expansion
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Pick one parent node from existing nodes (biased toward base solutions)
        int root_id = pick_root(gen);
        if (root_id >= static_cast<int>(nodes.size())) {
            root_id = root_id % static_cast<int>(nodes.size());
        }
        
        Eigen::Vector3d base_pos = nodes[root_id].pos;
        Eigen::Vector3d base_orient = nodes[root_id].orient;
        
        // Sample a small orientation perturbation
        Eigen::Vector3d target_orient = base_orient;
        target_orient.x() += d_angle(gen);
        target_orient.y() += d_angle(gen);
        target_orient.z() += d_angle(gen);
        
        // Normalize angles to [-pi, pi]
        for (int i = 0; i < 3; ++i) {
            while (target_orient(i) > M_PI) target_orient(i) -= 2.0 * M_PI;
            while (target_orient(i) < -M_PI) target_orient(i) += 2.0 * M_PI;
        }
        
        // Sample a small translation perturbation (world XYZ)
        Eigen::Vector3d target_pos = base_pos;
        target_pos.x() += d_pos(gen);
        target_pos.y() += d_pos(gen);
        target_pos.z() += d_pos(gen);
        
        // Build pose and run collision check
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(target_orient.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(target_orient.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(target_orient.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d R = Rz * Rx * Ry;
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = target_pos;
        
        glm::mat4 gpose = eigenMatrixToGlm(T);
        CollisionDetection::set_robot_pose_matrix(data, robot_index, gpose);
        
        if (CollisionDetection::is_collision(data)) {
            continue;
        }
        
        // Collision-free, add node to list
        nodes.push_back({target_pos, target_orient});
        rrt_nodes.push_back({target_pos, target_orient});
    }
    
    return !rrt_nodes.empty();
}

// ============================================================================
// Tolerance search methods (moved from CollisionDetection)
// ============================================================================

// Binary search: maximum collision-free translation distance
float OptiAlgorithm::searchLimit(
    TrajOptimizationData& data,
    int robot_index,
    const glm::mat4& base_pose,
    const glm::vec3& direction,
    float max_range,
    double tolerance,
    double spray_distance,
    const glm::vec3& tool_z_axis)
{
    float low = 0.0f;
    float high = max_range;
    float tol = static_cast<float>(tolerance);

    // Compute spray point position from trajectory point
    glm::vec3 tool_tcp_pos = glm::vec3(base_pose[3]);
    glm::vec3 workpiece_surface_pos = tool_tcp_pos - tool_z_axis * static_cast<float>(spray_distance);

    // Binary search: find maximum collision-free distance in [low, high]
    // Translate based on spray point, then back-solve trajectory-point position
    while (high - low > tol)
    {
        float mid = 0.5f * (low + high);
        
        // Adjust spray-point position (move on workpiece surface)
        glm::vec3 new_surface_pos = workpiece_surface_pos + direction * mid;
        
        // Back-solve trajectory-point position from new spray-point position
        // Need to preserve tool Z-axis direction, so extract rotation matrix from base_pose
        glm::mat3 base_rot = glm::mat3(base_pose);
        glm::vec3 new_tool_z_axis = -glm::normalize(glm::vec3(base_rot[2]));  // Tool Z-axis points to workpiece
        if (glm::length(new_tool_z_axis) < 1e-6f) {
            new_tool_z_axis = tool_z_axis;  // Fallback to provided tool_z_axis if computation fails
        }
        glm::vec3 new_tool_tcp_pos = new_surface_pos + new_tool_z_axis * static_cast<float>(spray_distance);
        
        // Build new trajectory-point pose
        glm::mat4 T = base_rot;
        T[3] = glm::vec4(new_tool_tcp_pos, 1.0f);

        CollisionDetection::set_robot_pose_matrix(data, robot_index, T);
        bool collided = CollisionDetection::is_collision(data);

        // Visualization support
        if (data.enableVisualization_ && data.onVisualizationRefresh) {
            // Update collision colors (red on collision, default otherwise)
            CollisionDetection::updateCollisionColors(data);
            
            data.onVisualizationRefresh();  // Refresh view
            
            // Delay
            if (data.visualizationDelayMs_ > 0) {
#ifdef _WIN32
                Sleep(data.visualizationDelayMs_);
#else
                usleep(data.visualizationDelayMs_ * 1000);
#endif
            }
        }

        if (!collided) low = mid;
        else high = mid;
    }

    return low;
}

// Binary search: maximum collision-free rotation angle
float OptiAlgorithm::searchRotLimit(
    TrajOptimizationData& data,
    int robot_index,
    const glm::mat4& base_pose,
    const glm::vec3& axis,
    float max_angle,
    double tolerance,
    double spray_distance,
    const glm::vec3& tool_z_axis)
{
    // Compute spray point position from trajectory point
    glm::vec3 tool_tcp_pos = glm::vec3(base_pose[3]);
    glm::mat3 base_rot = glm::mat3(base_pose);
    glm::vec3 current_tool_z = -glm::normalize(glm::vec3(base_rot[2]));  // Tool Z-axis points to workpiece
    if (glm::length(current_tool_z) < 1e-6f) {
        current_tool_z = tool_z_axis;  // Fallback to provided tool_z_axis if computation fails
    }
    glm::vec3 workpiece_surface_pos = tool_tcp_pos - current_tool_z * static_cast<float>(spray_distance);
    
    // Support positive and negative direction search
    float low = 0.0f;
    float high = max_angle;
    float tol = static_cast<float>(tolerance);
    
    // For negative-direction search, swap low/high semantics
    if (max_angle < 0) {
        low = max_angle;
        high = 0.0f;
    }

    // Binary search: find maximum collision-free angle in [low, high]
    // Rotate tool around spray point, then back-solve trajectory-point position
    while (std::abs(high - low) > tol)
    {
        float mid = 0.5f * (low + high);
        
        // Rotate tool around spray point
        // 1. Translate tool to spray point (use spray point as rotation center)
        glm::mat4 translate_to_surface = glm::translate(glm::mat4(1.0f), -workpiece_surface_pos);
        glm::mat4 translate_back = glm::translate(glm::mat4(1.0f), workpiece_surface_pos);
        
        // 2. Rotate tool around spray point
        glm::mat4 rotate_around_surface = glm::rotate(glm::mat4(1.0f), mid, axis);
        
        // 3. Compose transform: translate to spray point, rotate, translate back
        glm::mat4 R_full = translate_back * rotate_around_surface * translate_to_surface * base_pose;
        
        // 4. Extract new rotation matrix and position
        glm::mat3 new_rot = glm::mat3(R_full);
        glm::vec3 new_tool_tcp_pos = glm::vec3(R_full[3]);
        
        // 5. Validate new tool Z-axis direction (used in subsequent computations)
        glm::vec3 new_tool_z = -glm::normalize(glm::vec3(new_rot[2]));  // New tool Z-axis direction
        if (glm::length(new_tool_z) < 1e-6f) {
            new_tool_z = current_tool_z;  // Fallback to original direction if computation fails
        }
        
        // Build new trajectory-point pose
        glm::mat4 R = R_full;

        CollisionDetection::set_robot_pose_matrix(data, robot_index, R);
        bool collided = CollisionDetection::is_collision(data);
        
        // Visualization support
        if (data.enableVisualization_ && data.onVisualizationRefresh) {
            // Update collision colors (red on collision, default otherwise)
            CollisionDetection::updateCollisionColors(data);
            
            data.onVisualizationRefresh();  // Refresh view
            
            // Delay
            if (data.visualizationDelayMs_ > 0) {
#ifdef _WIN32
                Sleep(data.visualizationDelayMs_);
#else
                usleep(data.visualizationDelayMs_ * 1000);
#endif
            }
        }

        if (!collided) {
            // Collision-free, continue moving toward boundary
            if (max_angle >= 0) {
                low = mid;  // Positive direction: move lower bound up
            } else {
                high = mid; // Negative direction: move upper bound down
            }
        } else {
            // Collision detected, back off
            if (max_angle >= 0) {
                high = mid; // Positive direction: move upper bound down
            } else {
                low = mid;  // Negative direction: move lower bound up
            }
        }
    }

    // Return collision-free value closest to boundary
    return (max_angle >= 0) ? low : high;
}

// Step search: maximum collision-free translation distance
float OptiAlgorithm::searchLimitByStep(
    TrajOptimizationData& data,
    int robot_index,
    const glm::mat4& base_pose,
    const glm::vec3& direction,
    float max_range,
    double step_size,
    double spray_distance,
    const glm::vec3& tool_z_axis)
{
    // Compute spray point position from trajectory point
    glm::vec3 tool_tcp_pos = glm::vec3(base_pose[3]);
    glm::mat3 base_rot = glm::mat3(base_pose);
    glm::vec3 current_tool_z = -glm::normalize(glm::vec3(base_rot[2]));  // Tool Z-axis points to workpiece
    if (glm::length(current_tool_z) < 1e-6f) {
        current_tool_z = tool_z_axis;  // Fallback to provided tool_z_axis if computation fails
    }
    glm::vec3 workpiece_surface_pos = tool_tcp_pos - current_tool_z * static_cast<float>(spray_distance);
    
    float step = static_cast<float>(step_size);
    float best_safe = 0.0f;  // Maximum safe distance found
    bool found_any_safe = false;  // Whether any safe position was found
    int total_checks = 0;
    int collision_count = 0;
    
    // Start from 0 and scan full range (even if initial point collides)
    float current = 0.0f;
    
    while (current <= max_range)
    {
        // Adjust spray-point position (move on workpiece surface)
        glm::vec3 new_surface_pos = workpiece_surface_pos + direction * current;
        
        // Back-solve trajectory-point position from new spray point (keep tool Z-axis direction unchanged)
        glm::vec3 new_tool_tcp_pos = new_surface_pos + current_tool_z * static_cast<float>(spray_distance);
        
        // Build new trajectory-point pose
        glm::mat4 T = glm::mat4(base_rot);
        T[3] = glm::vec4(new_tool_tcp_pos, 1.0f);
        
        CollisionDetection::set_robot_pose_matrix(data, robot_index, T);
        bool collided = CollisionDetection::is_collision(data);
        
        total_checks++;
        if (collided) {
            collision_count++;
        }
        
        // Visualization support
        if (data.enableVisualization_ && data.onVisualizationRefresh) {
            CollisionDetection::updateCollisionColors(data);
            data.onVisualizationRefresh();
            
            if (data.visualizationDelayMs_ > 0) {
#ifdef _WIN32
                Sleep(data.visualizationDelayMs_);
#else
                usleep(data.visualizationDelayMs_ * 1000);
#endif
            }
        }
        
        if (!collided) {
            // Collision-free, update maximum safe distance
            best_safe = current;
            found_any_safe = true;
        }
        
        // Continue to next step (scan full range regardless of current collision state)
        current += step;
    }
    
    // Debug output
    static int call_count = 0;
    if (call_count < 5) {  // Only output first 5 calls
        qDebug() << "[searchLimitByStep] Checked" << total_checks << "positions, collisions:" 
                 << collision_count << ", found safe:" << found_any_safe << ", best_safe:" << best_safe;
        call_count++;
    }
    
    // Return maximum safe distance found (returns 0 if entire range collides)
    return best_safe;
}

// Step search: maximum collision-free rotation angle
float OptiAlgorithm::searchRotLimitByStep(
    TrajOptimizationData& data,
    int robot_index,
    const glm::mat4& base_pose,
    const glm::vec3& axis,
    float max_angle,
    double step_size,
    double spray_distance,
    const glm::vec3& tool_z_axis)
{
    // Compute spray point position from trajectory point
    glm::vec3 tool_tcp_pos = glm::vec3(base_pose[3]);
    glm::mat3 base_rot = glm::mat3(base_pose);
    glm::vec3 current_tool_z = -glm::normalize(glm::vec3(base_rot[2]));  // Tool Z-axis points to workpiece
    if (glm::length(current_tool_z) < 1e-6f) {
        current_tool_z = tool_z_axis;  // Fallback to provided tool_z_axis if computation fails
    }
    glm::vec3 workpiece_surface_pos = tool_tcp_pos - current_tool_z * static_cast<float>(spray_distance);
    
    float step = static_cast<float>(step_size);
    
    // Support positive and negative direction search
    bool is_negative = (max_angle < 0);
    float target = std::abs(max_angle);
    
    float best_safe = 0.0f;  // Maximum safe angle found
    bool found_any_safe = false;  // Whether any safe position was found
    int total_checks = 0;
    int collision_count = 0;
    
    // Start from 0 and scan full range (even if initial point collides)
    float current = 0.0f;
    
    while (current <= target)
    {
        float angle = is_negative ? -current : current;
        
        // Rotate tool around spray point
        // 1. Translate tool to spray point (use spray point as rotation center)
        glm::mat4 translate_to_surface = glm::translate(glm::mat4(1.0f), -workpiece_surface_pos);
        glm::mat4 translate_back = glm::translate(glm::mat4(1.0f), workpiece_surface_pos);
        
        // 2. Rotate tool around spray point
        glm::mat4 rotate_around_surface = glm::rotate(glm::mat4(1.0f), angle, axis);
        
        // 3. Compose transform: translate to spray point, rotate, translate back
        glm::mat4 R_full = translate_back * rotate_around_surface * translate_to_surface * base_pose;
        
        // 4. Extract new rotation matrix and position
        glm::mat3 new_rot = glm::mat3(R_full);
        glm::vec3 new_tool_tcp_pos = glm::vec3(R_full[3]);
        
        // 5. Validate new tool Z-axis direction (used in subsequent computations)
        glm::vec3 new_tool_z = -glm::normalize(glm::vec3(new_rot[2]));  // New tool Z-axis direction
        if (glm::length(new_tool_z) < 1e-6f) {
            new_tool_z = current_tool_z;  // Fallback to original direction if computation fails
        }
        
        // Build new trajectory-point pose
        glm::mat4 R = R_full;
        
        CollisionDetection::set_robot_pose_matrix(data, robot_index, R);
        bool collided = CollisionDetection::is_collision(data);
        
        total_checks++;
        if (collided) {
            collision_count++;
        }
        
        // Visualization support
        if (data.enableVisualization_ && data.onVisualizationRefresh) {
            CollisionDetection::updateCollisionColors(data);
            data.onVisualizationRefresh();
            
            if (data.visualizationDelayMs_ > 0) {
#ifdef _WIN32
                Sleep(data.visualizationDelayMs_);
#else
                usleep(data.visualizationDelayMs_ * 1000);
#endif
            }
        }
        
        if (!collided) {
            // Collision-free, update maximum safe angle
            best_safe = current;
            found_any_safe = true;
        }
        
        // Continue to next step (scan full range regardless of current collision state)
        current += step;
    }
    
    // Debug output
    static int call_count = 0;
    if (call_count < 5) {  // Only output first 5 calls
        qDebug() << "[searchRotLimitByStep] Checked" << total_checks << "angles, collisions:" 
                 << collision_count << ", found safe:" << found_any_safe << ", best_safe:" << best_safe;
        call_count++;
    }
    
    // Return maximum safe angle found (signed)
    return is_negative ? -best_safe : best_safe;
}

// Compute single-point tolerance
void OptiAlgorithm::computeSinglePointTolerance(
    TrajOptimizationData& data,
    int robot_index,
    const Eigen::Matrix4d& pose,
    Eigen::Matrix<double, 3, 2>& pos_tol,
    Eigen::Matrix<double, 3, 2>& ori_tol,
    const ToleranceLimitConfig& tol_limit,
    double spray_distance)
{
    glm::mat4 base_pose = eigenMatrixToGlm(pose);
    
    // Choose algorithm by flag
    bool useBinarySearch = data.useBinarySearchMethod_;

    // Extract tool Z-axis direction from trajectory-point pose (points to workpiece)
    glm::mat3 base_rot = glm::mat3(base_pose);
    glm::vec3 tool_z_axis = -glm::normalize(glm::vec3(base_rot[2]));  // Tool Z-axis points to workpiece
    if (glm::length(tool_z_axis) < 1e-6f) {
        tool_z_axis = glm::vec3(0, 0, -1);  // Default downward
    }

    // ========== Rotation tolerance (first, order: Z, X, Y) ==========
    const int rot_order[3] = {2, 0, 1};  // Z, X, Y
    const glm::vec3 rot_axes[3] = {
        {1,0,0},{0,1,0},{0,0,1}
    };

    for (int idx = 0; idx < 3; ++idx)
    {
        int i = rot_order[idx];
        float max_angle = static_cast<float>(tol_limit.max_rotation[i]);

        float pos, neg;
        if (useBinarySearch) {
            // Binary search
            pos = searchRotLimit(data, robot_index, base_pose, rot_axes[i], max_angle, tol_limit.rotation_tolerance, spray_distance, tool_z_axis);
            neg = searchRotLimit(data, robot_index, base_pose, rot_axes[i], -max_angle, tol_limit.rotation_tolerance, spray_distance, tool_z_axis);
        } else {
            // Step search
            pos = searchRotLimitByStep(data, robot_index, base_pose, rot_axes[i], max_angle, tol_limit.rotation_tolerance, spray_distance, tool_z_axis);
            neg = searchRotLimitByStep(data, robot_index, base_pose, rot_axes[i], -max_angle, tol_limit.rotation_tolerance, spray_distance, tool_z_axis);
        }

        ori_tol(i, 0) = neg; // min (already negative)
        ori_tol(i, 1) = pos; // max
    }

    // ========== Translation tolerance (second, order: Z, X, Y) ==========
    const int trans_order[3] = {2, 0, 1};  // Z, X, Y
    const glm::vec3 axis_dirs[3] = {
        {1,0,0},{0,1,0},{0,0,1}
    };

    for (int idx = 0; idx < 3; ++idx)
    {
        int i = trans_order[idx];
        float max_d = static_cast<float>(tol_limit.max_translation[i]);

        float pos, neg;
        if (useBinarySearch) {
            // Binary search
            pos = searchLimit(data, robot_index, base_pose, axis_dirs[i], max_d, tol_limit.position_tolerance, spray_distance, tool_z_axis);
            neg = searchLimit(data, robot_index, base_pose, -axis_dirs[i], max_d, tol_limit.position_tolerance, spray_distance, tool_z_axis);
        } else {
            // Step search
            pos = searchLimitByStep(data, robot_index, base_pose, axis_dirs[i], max_d, tol_limit.position_tolerance, spray_distance, tool_z_axis);
            neg = searchLimitByStep(data, robot_index, base_pose, -axis_dirs[i], max_d, tol_limit.position_tolerance, spray_distance, tool_z_axis);
        }

        pos_tol(i, 0) = -neg;  // min
        pos_tol(i, 1) = pos;  // max
    }
}

}