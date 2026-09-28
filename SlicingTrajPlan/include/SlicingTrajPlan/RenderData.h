#pragma once
#include <memory>
#include <RenderData/RenderData_global.h>
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/imodel.h>
#include <Eigen/Dense>
#include <vector>
// Include concrete rendering-data class definitions
#include "STLRenderData.h"
#include "BoundaryRenderData.h"
#include "SlicingRenderData.h"
#include "TrajectoryRenderData.h"
#include "ConnectionRenderData.h"
namespace ModelTrajPlan
{
    // Unified rendering-data manager class (similar to GeneralData design)
    class RenderData_SHAREDLIB RenderData
    {
    public:
        RenderData();
        ~RenderData();

        // Scene management (centralized management to avoid duplication)
        void setScene(iscene_spointer scene);
        iscene_spointer getScene() const { return scene_; }

        // Rendering-data accessors
        STLRenderData& stl();
        const STLRenderData& stl() const;

        BoundaryRenderData& boundary();
        const BoundaryRenderData& boundary() const;

        SlicingRenderData& slicing();
        const SlicingRenderData& slicing() const;

        TrajectoryRenderData& trajectory();
        const TrajectoryRenderData& trajectory() const;

        ConnectionRenderData& connection();
        const ConnectionRenderData& connection() const;

        // Initialize all rendering models
        void initializeAllModels();

    private:
        iscene_spointer scene_;  // Unified scene pointer

        // Manage internal data using smart pointers
        std::unique_ptr<STLRenderData> stlData_;
        std::unique_ptr<BoundaryRenderData> boundaryData_;
        std::unique_ptr<SlicingRenderData> slicingData_;
        std::unique_ptr<TrajectoryRenderData> trajectoryData_;
        std::unique_ptr<ConnectionRenderData> connectionData_;
    };
}