#pragma once

#include "TrajPlanPipeline.h"

namespace SprayTrajPlan
{
    enum class UvPlanningState { Empty, UVMapped, TrajectoryGenerated, Optimized, Exported };

    // Owns the valid UV planning stages. Failed operations leave the previous state intact.
    class UvPlanningSession
    {
    public:
        UvPlanningState state() const { return state_; }
        bool isRunning() const { return running_; }
        bool canMap() const { return !running_; }
        // Re-running a completed stage replaces its output and invalidates only
        // the stages after it, as required by the ribbon workflow.
        bool canGenerateTrajectory() const { return !running_ && state_ != UvPlanningState::Empty; }
        bool canOptimize() const { return !running_ && (state_ == UvPlanningState::TrajectoryGenerated || state_ == UvPlanningState::Optimized || state_ == UvPlanningState::Exported); }
        // The raw result is a valid trajectory and may be exported directly
        // after Start Get Traj. Once Start Optimize succeeds, the pipeline
        // replaces final_trajectory_ with the optimized result, so this same
        // command exports the newer trajectory without a separate mode.
        bool canExport() const {
            return !running_ && (state_ == UvPlanningState::TrajectoryGenerated ||
                state_ == UvPlanningState::Optimized || state_ == UvPlanningState::Exported);
        }

        bool startUvMap(TrajPlanPipeline& pipeline, const std::string& source, const TrajectoryParams& params);
        bool startGetTraj(TrajPlanPipeline& pipeline, const TrajectoryParams& params);
        bool startOptimize(TrajPlanPipeline& pipeline, const TrajectoryParams& params);
        bool exportPaq(TrajPlanPipeline& pipeline, const std::string& paqPath, const TrajectoryParams& params);
        void clearModel(TrajPlanPipeline& pipeline);
        void clearTrajectory(TrajPlanPipeline& pipeline);
        void clearConnection(TrajPlanPipeline& pipeline);

    private:
        UvPlanningState state_ = UvPlanningState::Empty;
        bool running_ = false;
    };
}
