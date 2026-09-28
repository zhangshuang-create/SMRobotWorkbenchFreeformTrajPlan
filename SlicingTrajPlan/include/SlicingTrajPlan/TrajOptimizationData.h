#pragma once

#include "TrajOptimizationData_global.h"
#include <ModelTangQing/iscene.h>
#include <SMRobotTangQing/irobot.h>
#include <SMRobotTangQing/iloader.h>
#include <ModelTangQing/icollision_detector.h>
#include <ModelTangQing/imodel.h>
#include <GeneralData/GeneralData.h>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <functional>
namespace ModelTrajPlan
{
    // ========== Trajectory-optimization data class ==========
    // Manages all data used in trajectory optimization
    // Provides required getter/setter methods and initialization interfaces
    class TrajOptimizationData_SHAREDLIB TrajOptimizationData
    {
    public:
        TrajOptimizationData();
        ~TrajOptimizationData();

        // ========== Collision-detection-related members ==========

        // --- Scene and loader ---
        iscene_spointer m_spScene;                     // Scene pointer
        iloader_spointer sp_loader;                    // Model loader
        iloader_spointer workpieceLoader_;             // Loader dedicated to workpiece XML imports
        iloader_spointer toolLoader_;                  // Loader dedicated to tool/collision-model XML imports

        // --- Collision detectors ---
        icollis_spointer m_spDetector;                 // Main collision detector
        std::vector<icollis_spointer>* externalDetectorVec;  // External detector vector (from robot_system_demo)
        std::vector<icollis_spointer> m_internalDetectors;   // Internal detectors for parallel algorithms

        // --- Workpiece and tool robots ---
        irobot_spointer workpieceRobot_;               // Workpiece robot
        irobot_spointer toolRobot_;                    // Tool robot

        // --- Robot collection ---
        std::vector<irobot_spointer> m_robots;         // Collection of all robots (stored in vector)

        // --- Collision visualization ---
        bool m_collision_visualization_enabled;        // Whether collision visualization is enabled
        bool m_last_collision_state;                   // Last collision state
        std::vector<std::vector<imodel_spointer>> m_robot_models;  // Robot-model hierarchy

        // ========== Trajectory optimization ==========

        // === Model paths ===
        std::string originalModelPath_;                // Original model path
        std::string actualModelPath_;                  // Actual model path

        // === Trajectory data ===
        std::map<double, TolerancedTrajectoryPoint> originalTrajectory_;      // Original trajectory
        std::map<double, TolerancedTrajectoryPoint> transformedTrajectory_;   // Transformed trajectory
        std::map<double, TolerancedTrajectoryPoint> tolerancedTrajectory_;    // Toleranced trajectory

        // === Performance statistics ===
        std::map<double, double> pointCalculationTimes_;                      // Computation time per point (key: timestamp)
        std::map<double, int> pointCollisionCheckCounts_;                     // Collision-check count per point (key: timestamp)
        int currentPointCollisionCheckCount_;                                 // Current-point collision-check counter
        double totalCollisionCheckTime_;                                      // Total collision-check time
        int totalCollisionCheckCount_;                                        // Total collision-check count

        // === Transformation matrix ===
        Eigen::Matrix4d transformMatrix_;              // 4x4 transformation matrix

        // === System state ===
        bool systemInitialized_;                       // Whether system initialization is complete
        bool hasOriginalModel_;                        // Whether original model is loaded
        bool hasActualModel_;                          // Whether actual model is loaded
        bool collisionDetectionRunning_;               // Whether collision detection is running
        bool useBinarySearchMethod_;                   // Whether to use binary search (true=binary, false=traversal)
        bool useParallelGAMethod_;                     // Whether to use parallel island GA+RRT (true=parallel island GA, false=traditional GA)
        bool useACOMethod_;                            // Whether to use ACO (true=ACO)

        // === Visualization configuration ===
        bool enableVisualization_;                     // Whether visualization is enabled
        int visualizationDelayMs_;                     // Visualization delay (ms)

        // === Trajectory frame models ===
        std::vector<imodel_spointer> trajectoryFrameModels_;  // Trajectory-frame model collection
        MeshData trajectoryFrameData_;                      // Trajectory-frame model data (for creation and update)
        float trajectoryFrameScale_;                          // Trajectory-frame scale
        bool trajectoryFrameNeedsUpdate_;                     // Whether trajectory frame needs update
        bool trajectoryVisible_;                              // Whether trajectory is visible

        // === Collision-point records ===
        std::vector<int> unavoidableCollisionPoints_;         // Unavoidable collision-point indices

        // === RRT backup points ===
        std::map<double, std::vector<Eigen::Matrix4d>> rrtBackupPoints_;  // RRT backup points (key: timestamp)
        std::map<double, bool> rrtGeneratedPoints_;          // Whether RRT point is generated at each timestamp (key: timestamp)

        // === File-path configuration ===
        std::vector<std::string> robotModelPaths_;            // Robot-model file paths
        std::string workpieceModelPath_;                      // Workpiece-model file path
        std::string trajectoryFilePath_;                      // Trajectory file path

        // === Robot-index constants ===
        static const int ROBOT_INDEX_1 = 0;            // First robot index
        static const int ROBOT_INDEX_2 = 1;            // Second robot index

        // === Callback functions ===
        // Robot-loaded callback
        std::function<void(irobot_spointer)> onRobotLoaded;
        // Visualization refresh callback (for OpenGL UI refresh)
        std::function<void()> onVisualizationRefresh;

        // ========== Member functions ==========

        // === Scene setup (corresponds to settings in robot_system_demo::init) ===
        void setScene(iscene_spointer scene);

        // === Initialize scene and materials ===
        // Initialize required scene/material resources for collision detection
        // Shared by trajectory-optimization and collision-detection modules
        void initializeSceneAndMaterials();

        // === Initialize robots and loader ===
        // Corresponds to initialization logic in robot_system_demo::init
        // Initializes robot collection and loader for OpenGL visualization
        void initializeRobotsAndLoader();

        // === Load workpiece model ===
        // Load workpiece model from file path selected in dialog
        // Internally uses loader and robot import/compile_robot interfaces
        bool loadWorkpieceModel(const std::string& filePath);

        // === Load tool model ===
        // Load tool model from file path selected in dialog
        // Internally uses loader and robot import/compile_robot interfaces
        bool loadToolModel(const std::string& filePath);
    };

}
