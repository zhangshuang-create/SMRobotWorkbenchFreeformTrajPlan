#pragma once
#include <RenderData/RenderData_global.h>
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/imodel.h>
#include <Eigen/Dense>
#include <vector>
namespace ModelTrajPlan
{
    // Connection rendering data (simplified)
    class RenderData_SHAREDLIB ConnectionRenderData
    {
    public:
        // Rendering model
        imodel_spointer connectionModel;

        // Model data (persistent storage)
        MeshData connectionData;

        // Color settings are defined in ConnectionSegment and are not needed here
        // Actual color is determined by the ConnectionSegment color field

        // Visibility control
        bool isVisible = true;

        // Update flags
        bool needsUpdate = true;

        ConnectionRenderData();
        ~ConnectionRenderData();
    };
}