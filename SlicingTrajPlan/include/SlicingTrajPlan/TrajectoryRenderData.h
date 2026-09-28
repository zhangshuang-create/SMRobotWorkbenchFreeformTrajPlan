#pragma once
#include <RenderData/RenderData_global.h>
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/imodel.h>
#include <vector>
namespace ModelTrajPlan
{
    // Trajectory rendering data (full version)
    class RenderData_SHAREDLIB TrajectoryRenderData
    {
    public:
        // Main trajectory rendering model (trajectory-point coordinate frames)
        imodel_spointer trajectoryModel;
        iscene_spointer sp_scene = nullptr;
        // Main trajectory model data (persistent storage)
        MeshData trajectoryData;

        // Single coordinate-frame model (for batch creation)
        imodel_spointer frameModel;

        // Single coordinate-frame model data (persistent storage)
        MeshData frameData;

        // Batch coordinate-frame model collection (for independent frame visualization)
        std::vector<imodel_spointer> frameModels;

        // Coordinate-frame scale
        float frameScale = 30.0f;

        // Visibility control
        bool isVisible = true;
        bool isFrameVisible = true;  // Independent coordinate-frame visibility

        // Update flags
        bool needsUpdate = true;
        bool frameNeedsUpdate = true;  // Coordinate-frame update flag

        TrajectoryRenderData();
        ~TrajectoryRenderData();
    };
}