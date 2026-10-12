#pragma once

#include "CommonDataTypes.h"
#include "OptCutsRunner.h"
#include "TrajectoryGenerator.h"
#include "TrajectoryExporter.h"
#include <functional>
#include <string>
#include <memory>

namespace SprayTrajPlan
{
    // Main pipeline orchestrator
    class TrajPlanPipeline
    {
    public:
        using OptimizationProgressCallback = std::function<void(int, const std::string&,
            const std::vector<TrajectorySegment>&)>;
        TrajPlanPipeline();
        ~TrajPlanPipeline();

        // Initialize the pipeline
        bool initialize();

        // Set the workspace used for transient OptCuts input and output files.
        void setOptCutsRoot(const std::string& root_path);

        // Independently callable UI stages. Each preserves the previous valid result on failure.
        bool startUvMap(const std::string& input_mesh_path, const TrajectoryParams& params);
        bool startGetTraj(const TrajectoryParams& params);
        bool startOptimize(const TrajectoryParams& params);
        void setOptimizationProgressCallback(OptimizationProgressCallback callback) {
            optimization_progress_callback_ = std::move(callback);
        }
        bool exportPaq(const std::string& paq_path, const TrajectoryParams& params);
        void clearModel();
        void clearTrajectory();
        void clearConnection();

        // Run the complete pipeline
        bool runPipeline(
            const std::string& input_mesh_path,
            const std::string& output_csv_path,
            const std::string& output_metadata_path,
            const std::string& output_paq_path,
            const TrajectoryParams& params
        );

        // Get the last generated trajectory
        const std::vector<TrajectorySegment>& getTrajectory() const {
            return final_trajectory_;
        }

        // Get OptCuts result
        const OptCutsResult& getOptCutsResult() const {
            return optcuts_result_;
        }

        // Get metadata
        const TrajectoryMetadata& getMetadata() const {
            return metadata_;
        }

    private:
        std::unique_ptr<OptCutsRunner> optcuts_runner_;
        std::unique_ptr<TrajectoryGenerator> traj_generator_;

        OptCutsResult optcuts_result_;
        std::vector<TrajectorySegment> final_trajectory_;
        TrajectoryMetadata metadata_;
        PlanningFrame planning_frame_;
        std::vector<TrajectorySegment> generated_trajectory_;
        std::string uv_source_path_;
        OptimizationProgressCallback optimization_progress_callback_;

        void reportOptimizationProgress(int percent, const std::string& stage,
            const std::vector<TrajectorySegment>& trajectory) const;

        // Pipeline stages
        bool runOptCutsStage(const std::string& input_mesh);
        bool runTrajectoryGeneration(const TrajectoryParams& params);
        bool runOptimization(const TrajectoryParams& params);
        bool runExport(
            const std::string& csv_path,
            const std::string& metadata_path,
            const std::string& paq_path,
            const TrajectoryParams& params
        );
    };

} // namespace SprayTrajPlan
