#include <TrajOptimizationData/TrajOptimizationData.h>
#include <SMRobotTangQing/robot_factory.h>
#include <algorithm>
namespace ModelTrajPlan
{
    TrajOptimizationData::TrajOptimizationData()
        // --- Scene-related pointer initialization ---
        : m_spScene(nullptr)
        , sp_loader(nullptr)
        , workpieceLoader_(nullptr)
        , toolLoader_(nullptr)
        // --- Collision-detector-related pointers ---
        , m_spDetector(nullptr)
        , externalDetectorVec(nullptr)
        // --- m_robots container-related initialization ---
        // --- Reserved extension slot ---
        , m_collision_visualization_enabled(false)
        , m_last_collision_state(false)
        // --- m_robot_models model-related initialization ---
        // --- File-path-related variables ---
        // --- Reserved extension slot ---
        , workpieceRobot_(nullptr)
        , toolRobot_(nullptr)
        // --- Reserved extension slot ---
        // --- Task-related transformation matrix ---
        , transformMatrix_(Eigen::Matrix4d::Identity())
        // --- State flags ---
        , systemInitialized_(false)
        , hasOriginalModel_(false)
        , hasActualModel_(false)
        , collisionDetectionRunning_(false)
        , useBinarySearchMethod_(true)  // Default: use binary-search method
        , useParallelGAMethod_(false)   // Default: traditional GA
        , useACOMethod_(false)          // Default: non-ACO
        // --- Visualization-related configuration ---
        , enableVisualization_(false)
        , visualizationDelayMs_(0)
        // --- Collision-detection statistics ---
        , currentPointCollisionCheckCount_(0)
        , totalCollisionCheckTime_(0.0)
        , totalCollisionCheckCount_(0)
        // --- Reserved extension slot ---
    {
    }

    TrajOptimizationData::~TrajOptimizationData()
    {
    }

    // === Set scene ===
    void TrajOptimizationData::setScene(iscene_spointer scene)
    {
        m_spScene = scene;
    }

    // === Initialize scene and materials ===
    // This method initializes scene/material resources required by collision detection in trajectory optimization
    // The concrete implementation should be adjusted according to CollisionDetection interfaces
    void TrajOptimizationData::initializeSceneAndMaterials()
    {
        // Scene and material initialization logic (placeholder)
        // CollisionDetection::set_global_lights(*this);
        // CollisionDetection::set_global_materials(*this);
    }

    void TrajOptimizationData::initializeRobotsAndLoader()
    {
        workpieceRobot_ = robot_factory::get_new_robot();
        toolRobot_ = robot_factory::get_new_robot();
        sp_loader = robot_factory::get_new_loader();
        workpieceLoader_ = robot_factory::get_new_loader();
        toolLoader_ = robot_factory::get_new_loader();
    }

    bool TrajOptimizationData::loadWorkpieceModel(const std::string& filePath)
    {
        if (filePath.empty() || !m_spScene) {
            return false;
        }

        if (workpieceRobot_) {
            auto it = std::find(m_robots.begin(), m_robots.end(), workpieceRobot_);
            if (it != m_robots.end()) m_robots.erase(it);
        }

        workpieceRobot_ = robot_factory::get_new_robot();
        if (!workpieceRobot_) {
            return false;
        }

        const bool isXml = filePath.find(".xml") != std::string::npos || filePath.find(".XML") != std::string::npos;
        const bool isRbt = filePath.find(".rbt") != std::string::npos || filePath.find(".RBT") != std::string::npos;
        bool loaded = false;
        if (isXml) {
            workpieceLoader_ = robot_factory::get_new_loader();
            if (!workpieceLoader_) return false;
            const size_t pos = filePath.find_last_of("/\\");
            const std::string folder = pos != std::string::npos ? filePath.substr(0, pos) : ".";
            const std::string file = pos != std::string::npos ? filePath.substr(pos + 1) : filePath;
            if (workpieceLoader_->import(file.c_str(), folder.c_str())) {
                const std::vector<irobot_spointer> loadedRobots = workpieceLoader_->compile_robot(m_spScene);
                if (!loadedRobots.empty()) {
                    workpieceRobot_ = loadedRobots.front();
                    loaded = true;
                }
            }
        } else if (isRbt) {
            loaded = workpieceRobot_->load(filePath.c_str(), m_spScene);
        }
        if (!loaded) return false;

        workpieceRobot_->set_base_mat(glm::mat4(1.0f));
        workpieceRobot_->forward_kinematic();
        m_robots.push_back(workpieceRobot_);
        workpieceModelPath_ = filePath;
        originalModelPath_ = filePath;
        hasOriginalModel_ = true;
        systemInitialized_ = workpieceRobot_ && toolRobot_ && m_robots.size() >= 2;
        return true;
    }

    bool TrajOptimizationData::loadToolModel(const std::string& filePath)
    {
        if (filePath.empty() || !m_spScene) {
            return false;
        }

        if (toolRobot_) {
            auto it = std::find(m_robots.begin(), m_robots.end(), toolRobot_);
            if (it != m_robots.end()) m_robots.erase(it);
        }

        toolRobot_ = robot_factory::get_new_robot();
        if (!toolRobot_) {
            return false;
        }

        const bool isXml = filePath.find(".xml") != std::string::npos || filePath.find(".XML") != std::string::npos;
        const bool isRbt = filePath.find(".rbt") != std::string::npos || filePath.find(".RBT") != std::string::npos;
        bool loaded = false;
        if (isXml) {
            toolLoader_ = robot_factory::get_new_loader();
            if (!toolLoader_) return false;
            const size_t pos = filePath.find_last_of("/\\");
            const std::string folder = pos != std::string::npos ? filePath.substr(0, pos) : ".";
            const std::string file = pos != std::string::npos ? filePath.substr(pos + 1) : filePath;
            if (toolLoader_->import(file.c_str(), folder.c_str())) {
                const std::vector<irobot_spointer> loadedRobots = toolLoader_->compile_robot(m_spScene);
                if (!loadedRobots.empty()) {
                    toolRobot_ = loadedRobots.front();
                    loaded = true;
                }
            }
        } else if (isRbt) {
            loaded = toolRobot_->load(filePath.c_str(), m_spScene);
        }
        if (!loaded) return false;

        toolRobot_->set_base_mat(glm::mat4(1.0f));
        toolRobot_->forward_kinematic();
        m_robots.push_back(toolRobot_);
        actualModelPath_ = filePath;
        hasActualModel_ = true;
        systemInitialized_ = workpieceRobot_ && toolRobot_ && m_robots.size() >= 2;
        return true;
    }
}
