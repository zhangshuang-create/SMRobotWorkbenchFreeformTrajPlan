#include "UvPlanningSession.h"

namespace SprayTrajPlan
{
    bool UvPlanningSession::startUvMap(TrajPlanPipeline& pipeline, const std::string& source, const TrajectoryParams& params)
    {
        if (!canMap()) return false;
        running_ = true;
        const bool ok = pipeline.startUvMap(source, params);
        running_ = false;
        if (ok) state_ = UvPlanningState::UVMapped;
        return ok;
    }

    bool UvPlanningSession::startGetTraj(TrajPlanPipeline& pipeline, const TrajectoryParams& params)
    {
        if (!canGenerateTrajectory()) return false;
        running_ = true;
        const bool ok = pipeline.startGetTraj(params);
        running_ = false;
        if (ok) state_ = UvPlanningState::TrajectoryGenerated;
        return ok;
    }

    bool UvPlanningSession::startOptimize(TrajPlanPipeline& pipeline, const TrajectoryParams& params)
    {
        if (!canOptimize()) return false;
        running_ = true;
        const bool ok = pipeline.startOptimize(params);
        running_ = false;
        if (ok) state_ = UvPlanningState::Optimized;
        return ok;
    }

    bool UvPlanningSession::exportPaq(TrajPlanPipeline& pipeline, const std::string& paqPath, const TrajectoryParams& params)
    {
        if (!canExport()) return false;
        running_ = true;
        const bool ok = pipeline.exportPaq(paqPath, params);
        running_ = false;
        if (ok) state_ = UvPlanningState::Exported;
        return ok;
    }

    void UvPlanningSession::clearModel(TrajPlanPipeline& pipeline)
    {
        pipeline.clearModel();
        state_ = UvPlanningState::Empty;
    }

    void UvPlanningSession::clearTrajectory(TrajPlanPipeline& pipeline)
    {
        pipeline.clearTrajectory();
        if (state_ != UvPlanningState::Empty) state_ = UvPlanningState::UVMapped;
    }

    void UvPlanningSession::clearConnection(TrajPlanPipeline& pipeline)
    {
        pipeline.clearConnection();
    }
}
