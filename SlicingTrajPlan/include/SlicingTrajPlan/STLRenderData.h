#pragma once
#include <RenderData/RenderData_global.h>
#include <ModelTangQing/imodel.h>
namespace ModelTrajPlan
{
    // STL rendering data (simplified)
    class RenderData_SHAREDLIB STLRenderData
    {
    public:
        // Rendering model
        imodel_spointer meshModel;

        // Model data (persistent storage)
        MeshData meshData;

        // Visibility control
        bool isVisible = true;

        // Update flags
        bool needsUpdate = true;

        STLRenderData();
        ~STLRenderData();
    };
}