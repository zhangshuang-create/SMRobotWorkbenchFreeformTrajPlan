// CollisionDetection.cpp
// Include GLEW before other OpenGL headers, then Qt headers.
#include <GL/glew.h>  // GLEW must be included before other OpenGL headers
#include <CollisionDetection/CollisionDetection.h>
#include <QDebug>
#include <iostream>
#include <vector>
#include <data_path.h>
#include <algorithm>
#include <chrono>
#include <cmath>

#include <SMRobotTangQing/robot_factory.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Windows header for Sleep, etc.
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX  // Prevent Windows.h from defining min/max macros
#endif
#include <Windows.h>
#else
#include <unistd.h>
#endif

// Qt headers after OpenGL
#include <QDebug>
namespace ModelTrajPlan
{
    // ------------------------------------------------------------
    // Recursive traversal of SDK SMRobotNameTree (same pattern as external samples).
    // ------------------------------------------------------------
    static void collect_names_recursive(const SMRobotNameTree& node, std::vector<std::string>& names)
    {
        // Push non-empty node name, then recurse children / siblings
        if (!node.name.empty()) {
            names.push_back(node.name);
        }

        // Child nodes
        for (const auto& child : node.child_name_tree) {
            collect_names_recursive(child, names);
        }

        // Sibling nodes (next)
        for (const auto& next : node.next_name_tree) {
            collect_names_recursive(next, names);
        }
    }

    // ========== CollisionDetection implementation ==========
    // Methods take TrajOptimizationData& for shared state

    // ------------------------------------------------------------
    // collect_all_models: use SMRobotNameTree to collect all link models for a robot
    // ------------------------------------------------------------
    void CollisionDetection::collect_all_models(const irobot_spointer& robot, std::vector<imodel_spointer>& models)
    {
        if (!robot) return;

        SMRobotNameTree name_tree = robot->get_link_tree();
        std::vector<std::string> all_names;
        collect_names_recursive(name_tree, all_names);

        // Optional: print collected names for debugging
        // std::cout << "Collected names for robot '" << robot->get_robot_name() << "':\n";
        // for (auto& n : all_names) std::cout << "  " << n << std::endl;

        robot->get_models(all_names, models);
    }


    // ------------------------------------------------------------
    // initializeRobots: load robot models from paths (.xml / .rbt, etc.)
    // ------------------------------------------------------------
    bool CollisionDetection::initializeRobots(
        TrajOptimizationData& data,
        const std::vector<std::string>& robot_model_paths)
    {
        // Only create a new scene if one doesn't exist
        if (!data.m_spScene) {
            std::string error_info;
            data.m_spScene = iscene::get_new_scene(error_info);
            if (!data.m_spScene) {
                std::cerr << "Create scene failed: " << error_info << std::endl;
                return false;
            }

            data.m_spScene->init();
            data.m_spScene->set_camera_center(0, 0, -5000);
            data.m_spScene->set_trans_ratio(10);
            data.m_spScene->set_wheel_ratio(5);
            set_global_lights(data);
            set_global_materials(data);
        }

        data.m_robots.clear();

        for (size_t i = 0; i < robot_model_paths.size(); ++i)
        {
            const std::string& fullpath = robot_model_paths[i];
            if (fullpath.empty()) continue;

            std::cout << "Loading robot file: " << fullpath << std::endl;

            irobot_spointer robot = robot_factory::get_new_robot();
            if (!robot) continue;

            bool load_ok = false;

            if (fullpath.find(".xml") != std::string::npos)
            {
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
            }
            else if (fullpath.find(".rbt") != std::string::npos)
            {
                load_ok = robot->load(fullpath.c_str(), data.m_spScene);
            }

            if (!load_ok) {
                std::cerr << "Failed to load robot: " << fullpath << std::endl;
                continue;
            }

            std::string robot_name = "Robot_" + std::to_string(data.m_robots.size() + 1);
            robot->set_robot_name(robot_name);
            robot->update_instance_name();

            // Set robot to origin (identity matrix) - no automatic offset
            glm::mat4 base = glm::mat4(1.0f);
            robot->set_base_mat(base);
            std::cout << "[CollisionDetection] " << robot_name << " loaded at origin (0, 0, 0)" << std::endl;

            SMRobotNameTree tree = robot->get_link_tree();
            std::cout << "\n--- " << robot_name << " Link Tree ---\n";
            tree.print();

            data.m_robots.push_back(robot);

            // Notify host that a robot was loaded (optional callback)
            if (data.onRobotLoaded) {
                data.onRobotLoaded(robot);
            }
        }

        if (data.m_robots.empty()) {
            std::cerr << "No robot loaded!" << std::endl;
            return false;
        }

        if (!init_collision(data)) {
            std::cerr << "Collision initialization failed!" << std::endl;
            return false;
        }

        std::cout << "Successfully loaded " << data.m_robots.size() << " robot(s)!" << std::endl;
        return true;
    }


    bool CollisionDetection::draw(TrajOptimizationData& data)
    {
        for (size_t i = 0; i < data.m_robots.size(); ++i) {
            if (data.m_robots[i]) {
                data.m_robots[i]->forward_kinematic();
                data.m_robots[i]->draw();
            }
        }
        return true;
    }

    // ------------------------------------------------------------
    // resize_window (unchanged behavior)
    // ------------------------------------------------------------
    bool CollisionDetection::resize_window(TrajOptimizationData& data, int width, int height)
    {
        glViewport(0, 0, width, height);
        if (data.m_spScene) {
            data.m_spScene->set_window_size(width, height);
            data.m_spScene->set_vmatrix();
            glm::mat4 projection = glm::perspective(glm::radians(45.0f), (GLfloat)width / (GLfloat)height, 1.0f, 20000.0f);
            data.m_spScene->set_pmatrix(glm::value_ptr(projection));
        }
        return true;
    }

    // ------------------------------------------------------------
    // run_job1: placeholder
    // ------------------------------------------------------------
    bool CollisionDetection::run_job1(TrajOptimizationData& data)
    {
        return false;
    }

    // ------------------------------------------------------------
    // set_global_lights: scene lighting setup
    // ------------------------------------------------------------
    bool CollisionDetection::set_global_lights(TrajOptimizationData& data)
    {
        light_attr light;
        light.isEnabled = true;
        light.lightType = light_attr::PointLight;
        light.ambient = glm::vec4(0.2f, 0.2f, 0.2f, 1.0f);
        light.diffuse = glm::vec4(0.8f, 0.8f, 0.8f, 1.0f);
        light.specular = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f);
        light.direction = glm::vec3(0.0f, 0.0f, -1.0f);
        light.position = glm::vec3(0.0f, 0.0f, 3000.0f);
        light.constant = 1e-4f;
        light.linear = 1e-6f;
        light.quadratic = 1e-9f;
        light.cutOff = 0.8f;
        light.outerCutOff = 0.5f;
        data.m_spScene->set_global_light(light, 0);

        light.lightType = light_attr::DirectionLight;
        data.m_spScene->set_global_light(light, 1);

        light.lightType = light_attr::SpotLight;
        light.direction = glm::vec3(0.0f, -1.0f, 0.0f);
        light.position = glm::vec3(0.0f, 1850.0f, 500.0f);
        data.m_spScene->set_global_light(light, 2);
        return true;
    }

    // ------------------------------------------------------------
    // set_global_materials: material setup
    // ------------------------------------------------------------
    bool CollisionDetection::set_global_materials(TrajOptimizationData& data)
    {
        // Global materials commented out; use system defaults (see robot_system_demo)
        // material_attr red_material(material_attr::MaterialRed);
        // material_attr green_material(material_attr::MaterialGreen);
        // material_attr blue_material(material_attr::MaterialBlue);
        // data.m_spScene->set_global_material(red_material, 0);
        // data.m_spScene->set_global_material(green_material, 1);
        // data.m_spScene->set_global_material(blue_material, 2);
        return true;
    }

    // ------------------------------------------------------------
    // setCollisionVisualization: enable / disable collision coloring
    // ------------------------------------------------------------
    void CollisionDetection::setCollisionVisualization(
        TrajOptimizationData& data,
        bool enable)
    {
        data.m_collision_visualization_enabled = enable;

        if (enable) {
            // Cache per-robot models for visualization
            data.m_robot_models.clear();
            data.m_robot_models.resize(data.m_robots.size());

            for (size_t i = 0; i < data.m_robots.size(); ++i) {
                collect_all_models(data.m_robots[i], data.m_robot_models[i]);
            }

            // Seed last collision state from current scene
            data.m_last_collision_state = is_collision(data);

            std::cout << "[Collision Visualization] Enabled - " << data.m_robots.size() << " robots tracked" << std::endl;
            std::cout << "[Collision Visualization] Initial collision state: "
                << (data.m_last_collision_state ? "COLLISION" : "NO COLLISION") << std::endl;
        }
        else {
            // Restore global materials on all cached models
            for (size_t i = 0; i < data.m_robot_models.size(); ++i) {
                for (size_t j = 0; j < data.m_robot_models[i].size(); ++j) {
                    if (data.m_robot_models[i][j]) {
                        data.m_robot_models[i][j]->use_material(MATERIAL_TYPE::GLOBAL);
                    }
                }
            }

            data.m_robot_models.clear();
            data.m_last_collision_state = false;
            std::cout << "[Collision Visualization] Disabled" << std::endl;
        }
    }

    // ------------------------------------------------------------
    // updateCollisionColors: refresh model colors when collision state changes
    // ------------------------------------------------------------
    void CollisionDetection::updateCollisionColors(TrajOptimizationData& data)
    {
        if (!data.m_collision_visualization_enabled) return;

        if (data.m_robots.empty() || !data.m_spDetector) return;

        bool current_collision = is_collision(data);

        static int debug_counter = 0;
        if (++debug_counter % 100 == 0) {  // Log roughly every 100 frames
            std::cout << "[Debug] Collision state: " << (current_collision ? "TRUE" : "FALSE")
                << ", Last state: " << (data.m_last_collision_state ? "TRUE" : "FALSE") << std::endl;
        }

        if (current_collision != data.m_last_collision_state) {
            if (current_collision) {
                material_attr collision_material(material_attr::MaterialRed);
                collision_material.emission = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
                collision_material.ambient = glm::vec4(0.8f, 0.1f, 0.1f, 1.0f);
                collision_material.diffuse = glm::vec4(1.0f, 0.2f, 0.2f, 1.0f);

                for (size_t i = 0; i < data.m_robot_models.size(); ++i) {
                    for (size_t j = 0; j < data.m_robot_models[i].size(); ++j) {
                        if (data.m_robot_models[i][j]) {
                            data.m_robot_models[i][j]->set_local_material(collision_material);
                            data.m_robot_models[i][j]->use_material(MATERIAL_TYPE::LOCAL);
                        }
                    }
                }

                std::cout << "[Collision] DETECTED - Models changed to RED" << std::endl;
            }
            else {
                for (size_t i = 0; i < data.m_robot_models.size(); ++i) {
                    for (size_t j = 0; j < data.m_robot_models[i].size(); ++j) {
                        if (data.m_robot_models[i][j]) {
                            data.m_robot_models[i][j]->use_material(MATERIAL_TYPE::GLOBAL, material_attr::MaterialGreen);
                        }
                    }
                }

                std::cout << "[Collision] CLEARED - Models restored to original colors" << std::endl;
            }

            data.m_last_collision_state = current_collision;
        }
    }

    // ------------------------------------------------------------
    // init_collision: build collision pairs for all robots and internal detector pool
    // ------------------------------------------------------------
    bool CollisionDetection::init_collision(TrajOptimizationData& data)
    {
        if (data.m_robots.empty()) {
            qWarning() << "[init_collision] ERROR: No robots available for collision detection!";
            return false;
        }

        data.m_spDetector.reset();
        data.m_internalDetectors.clear();

        data.m_spDetector = icollision_detector::get_new_detector();
        if (!data.m_spDetector) {
            qWarning() << "[init_collision] ERROR: Failed to create main collision detector!";
            return false;
        }

        const int parallel_detector_count = 10;  // Pool size for parallel GA / OpenMP style use
        data.m_internalDetectors.reserve(parallel_detector_count);
        for (int i = 0; i < parallel_detector_count; ++i) {
            icollis_spointer det = icollision_detector::get_new_detector();
            if (det) {
                data.m_internalDetectors.push_back(det);
            }
        }

        if (data.m_internalDetectors.empty()) {
            qWarning() << "[init_collision] WARNING: Failed to create internal detector pool, parallel GA will fallback to single detector.";
        }

        std::vector<std::vector<imodel_spointer>> all_robot_models(data.m_robots.size());
        for (size_t i = 0; i < data.m_robots.size(); ++i) {
            collect_all_models(data.m_robots[i], all_robot_models[i]);

            if (all_robot_models[i].empty()) {
                qWarning() << "[init_collision] WARNING: Robot_" << (i + 1)
                    << "(" << QString::fromStdString(data.m_robots[i]->get_robot_name())
                    << ") has NO models!";
            }
        }

        int collision_pair_count = 0;

        std::vector<icollis_spointer> detectors_to_init;
        detectors_to_init.push_back(data.m_spDetector);
        for (auto& det : data.m_internalDetectors) {
            if (det) detectors_to_init.push_back(det);
        }

        for (size_t i = 0; i < all_robot_models.size(); ++i) {
            for (size_t j = i + 1; j < all_robot_models.size(); ++j) {
                for (auto& m1 : all_robot_models[i]) {
                    for (auto& m2 : all_robot_models[j]) {
                        for (auto& det : detectors_to_init) {
                            if (det) {
                                det->insert(m1, m2);
                            }
                        }
                        collision_pair_count++;
                    }
                }
            }
        }

        qDebug() << "[init_collision] Initialized with" << collision_pair_count
            << "collision pairs, detector pool size =" << detectors_to_init.size();

        return true;
    }

    // ------------------------------------------------------------
    // is_collision: query detector
    // ------------------------------------------------------------
    bool CollisionDetection::is_collision(TrajOptimizationData& data)
    {
        if (!data.m_spDetector) {
            static int error_count = 0;
            if (error_count < 5) {
                qWarning() << "[is_collision] ERROR: Detector is NULL! This should not happen. "
                    << "Please ensure collision detector is initialized before use. "
                    << "Returning true (collision) for safety.";
                error_count++;
            }
            return true;
        }

        auto startTime = std::chrono::high_resolution_clock::now();

        data.currentPointCollisionCheckCount_++;
        data.totalCollisionCheckCount_++;

        bool result = data.m_spDetector->is_collision();

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        double timeMs = duration.count() / 1000.0;
        data.totalCollisionCheckTime_ += timeMs;

        return result;
    }

    // ------------------------------------------------------------
    // get_scene
    // ------------------------------------------------------------
    iscene_spointer CollisionDetection::get_scene(TrajOptimizationData& data)
    {
        return data.m_spScene;
    }

    // ------------------------------------------------------------
    // robot joint helpers (wrappers for tests / UI)
    // ------------------------------------------------------------
    bool CollisionDetection::robot_joint_increase(
        TrajOptimizationData& data,
        unsigned int robot_index,
        unsigned int joint_index,
        float increment,
        AngleUnit unit)
    {
        if (robot_index >= data.m_robots.size()) {
            std::cerr << "[Error] Robot index " << robot_index << " out of range!" << std::endl;
            return false;
        }

        irobot_spointer robot = data.m_robots[robot_index];
        if (!robot) {
            std::cerr << "[Error] Robot pointer at index " << robot_index << " is null!" << std::endl;
            return false;
        }

        return robot->joint_increase(joint_index, increment, unit);
    }


    bool CollisionDetection::robot_joint_decrease(
        TrajOptimizationData& data,
        unsigned int robot_index,
        unsigned int joint_index,
        float decrement,
        AngleUnit unit)
    {
        if (robot_index >= data.m_robots.size()) {
            std::cerr << "[Error] Robot index " << robot_index << " out of range!" << std::endl;
            return false;
        }

        irobot_spointer robot = data.m_robots[robot_index];
        if (!robot) {
            std::cerr << "[Error] Robot pointer at index " << robot_index << " is null!" << std::endl;
            return false;
        }

        return robot->joint_decrease(joint_index, decrement, unit);
    }


    bool CollisionDetection::set_robot_pose(
        TrajOptimizationData& data,
        unsigned int robot_index,
        const glm::mat4& transform)
    {
        if (robot_index >= data.m_robots.size()) {
            std::cerr << "[Error] Robot index " << robot_index << " out of range!" << std::endl;
            return false;
        }

        irobot_spointer robot = data.m_robots[robot_index];
        if (!robot) {
            std::cerr << "[Error] Robot pointer at index " << robot_index << " is null!" << std::endl;
            return false;
        }

        robot->set_base_mat(transform);

        return robot->forward_kinematic();
    }



    bool CollisionDetection::move_robot_pose(
        TrajOptimizationData& data,
        unsigned int robot_index,
        const glm::vec3& translation_delta,
        const glm::vec3& rotation_delta_deg,
        bool use_local_coord)
    {
        if (robot_index >= data.m_robots.size()) {
            std::cerr << "[Error] robot_index out of range: " << robot_index << std::endl;
            return false;
        }

        irobot_spointer robot = data.m_robots[robot_index];
        if (!robot) {
            std::cerr << "[Error] Robot pointer is null!" << std::endl;
            return false;
        }

        glm::mat4 current_pose = robot->get_base_mat();

        glm::mat4 translation = glm::translate(glm::mat4(1.0f), translation_delta);
        glm::mat4 rotation = glm::mat4(1.0f);
        rotation = glm::rotate(rotation, glm::radians(rotation_delta_deg.x), glm::vec3(1, 0, 0));
        rotation = glm::rotate(rotation, glm::radians(rotation_delta_deg.y), glm::vec3(0, 1, 0));
        rotation = glm::rotate(rotation, glm::radians(rotation_delta_deg.z), glm::vec3(0, 0, 1));

        glm::mat4 new_pose;
        if (use_local_coord) {
            new_pose = current_pose * translation * rotation;
        }
        else {
            new_pose = translation * rotation * current_pose;
        }

        return set_robot_pose(data, robot_index, new_pose);
    }

    bool CollisionDetection::set_robot_pose_absolute(
        TrajOptimizationData& data,
        unsigned int robot_index,
        const glm::vec3& position,     // Base position
        const glm::vec3& euler_deg     // Euler angles (deg), XYZ order
    )
    {
        if (robot_index >= data.m_robots.size()) {
            std::cerr << "[Error] robot_index out of range: " << robot_index << std::endl;
            return false;
        }

        irobot_spointer robot = data.m_robots[robot_index];
        if (!robot) {
            std::cerr << "[Error] Robot pointer is null!" << std::endl;
            return false;
        }

        glm::mat4 T = glm::translate(glm::mat4(1.0f), position);

        glm::mat4 R = glm::mat4(1.0f);
        R = glm::rotate(R, glm::radians(euler_deg.x), glm::vec3(1, 0, 0));
        R = glm::rotate(R, glm::radians(euler_deg.y), glm::vec3(0, 1, 0));
        R = glm::rotate(R, glm::radians(euler_deg.z), glm::vec3(0, 0, 1));

        glm::mat4 absolute_pose = T * R;

        robot->set_base_mat(absolute_pose);

        return robot->forward_kinematic();
    }

    bool CollisionDetection::set_robot_pose_matrix(
        TrajOptimizationData& data,
        unsigned int robot_index,
        const glm::mat4& pose_matrix   // Full 4x4 base pose
    )
    {
        if (robot_index >= data.m_robots.size()) {
            std::cerr << "[Error] robot_index out of range: " << robot_index << std::endl;
            return false;
        }

        irobot_spointer robot = data.m_robots[robot_index];
        if (!robot) {
            std::cerr << "[Error] Robot pointer is null!" << std::endl;
            return false;
        }

        robot->set_base_mat(pose_matrix);

        return robot->forward_kinematic();
    }

    // computeToleranceForTrajectory: see TrajOptimizationAlgorithm

    // computeSinglePointTolerance: see OptiAlgorithm

    // searchLimit: see OptiAlgorithm

    // searchLimitByStep: see OptiAlgorithm

    // searchRotLimitByStep: see OptiAlgorithm

    // findCollisionFreePose: see OptiAlgorithm

    // searchRotLimit: see OptiAlgorithm

    glm::mat4 CollisionDetection::eigenMatrixToGlm(const Eigen::Matrix4d& m)
    {
        glm::mat4 out;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                out[c][r] = static_cast<float>(m(r, c)); // glm is column-major; transpose elements
            }
        }
        return out;
    }

    Eigen::Matrix4d CollisionDetection::glmMatrixToEigen(const glm::mat4& m)
    {
        Eigen::Matrix4d out;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                out(r, c) = static_cast<double>(m[c][r]); // glm column-major to Eigen row-major
            }
        }
        return out;
    }


}