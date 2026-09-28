#pragma once
#include <RenderData/RenderData_global.h>
#include <ModelTangQing/imodel.h>
#include <Eigen/Dense>
namespace ModelTrajPlan
{
    // Boundary rendering data (simplified)
    class RenderData_SHAREDLIB BoundaryRenderData
    {
    public:
        // Rendering models
        imodel_spointer upperBoundary;
        imodel_spointer lowerBoundary;
        imodel_spointer upperBoundaryOffset;
        imodel_spointer lowerBoundaryOffset;
        imodel_spointer intersectionPoints;
        // Model data (persistent storage)
        MeshData upperBoundaryData;
        MeshData lowerBoundaryData;
        MeshData upperOffsetData;
        MeshData lowerOffsetData;
        MeshData intersectionPointsData_;
        // Color settings
        Eigen::Vector3f upperColor = Eigen::Vector3f(1.0f, 0.0f, 0.0f);  // Red
        Eigen::Vector3f lowerColor = Eigen::Vector3f(0.0f, 0.0f, 1.0f);  // Blue
        Eigen::Vector3f offsetColor = Eigen::Vector3f(1.0f, 1.0f, 0.0f); // Yellow
        float lineWidth = 20.0f;

        // Visibility control
        bool isVisible = true;
        bool isOffsetVisible = true;

        // Update flags
        bool needsUpdate = true;

        BoundaryRenderData();
        ~BoundaryRenderData();
    };
}