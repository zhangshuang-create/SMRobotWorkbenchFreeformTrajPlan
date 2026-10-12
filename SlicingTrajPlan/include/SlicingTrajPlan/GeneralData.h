#pragma once
#include "GeneralData_global.h"

// Include required standard-library headers
#include <vector>
#include <string>
#include <Eigen/Dense>

// Include all data-structure headers
#include <SlicingTrajPlan/PModelData.h>
#include <SlicingTrajPlan/BoundaryData.h>
#include <SlicingTrajPlan/PTrajData.h>
#include <SlicingTrajPlan/SlicingData.h>
namespace ModelTrajPlan
{
    // Data-state enum
    enum class DataState {
        Empty = 0,           // Empty state, no data
        Loaded = 1,          // Data loaded
        Preprocessed = 2,    // Data preprocessed
        EdgeExtracted = 3,   // Edge extracted
        TrajectoryGenerated = 4  // Trajectory generated
    };


    // Unified data manager class - pure data container
    class GeneralData_SHAREDLIB GeneralData
    {
    public:
        GeneralData();
        ~GeneralData();

        // === Model data (from ModelData.h) ===
        ModelData  modelDataManager_;          // Model-data manager instance

        // === Boundary data (from BoundaryData.h) ===
        BoundaryData boundaryData_;                  // Boundary-data instance

        // === Trajectory data (from TrajData.h) ===
        TrajData trajData_;                          // Trajectory-data instance

        // === Slicing data (from SlicingData.h) ===
        SlicingData slicingData_;                    // Slicing-data instance

        bool hasExtractionResults_;                        // Whether extraction results exist

        // === Processing-state data (stored directly in GeneralData) ===
        std::vector<std::string> filePaths_;               // File-path collection
        DataState currentState_;                           // Current data-processing state
        bool hasOriginalData_;                             // Whether original data exists
        bool hasPreprocessedData_;                         // Whether preprocessed data exists

        // === Model-data collections ===
        std::vector<PrivateModel> originalData_;        // Original model data
        std::vector<PrivateModel> preprocessedData_;    // Preprocessed model data

        // === Processing-state flags (algorithm progress) ===
        struct ProcessingState {
            bool isSTLProcessed = false;           // Whether STL is processed
            bool isBoundaryDetected = false;       // Whether boundary is detected
            bool isSlicingCompleted = false;       // Whether slicing is completed
            bool isTrajectoryGenerated = false;    // Whether trajectory is generated
            bool isTrajectoryConnected = false;    // Whether trajectory is connected

            // Visibility control
            bool keepOriginalVisibleAfterPreprocess = false;  // Keep original data visible after preprocessing
            bool originalDataVisible = true;                   // Whether original data is visible
        } processingState_;

        // === Transformation matrix (for export and coordinate conversion) ===
        Eigen::Matrix4d transformMatrix_;  // 4x4 homogeneous transform matrix (for model/trajectory coordinate transform)

    private:

    };
}
