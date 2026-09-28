#include "TrajPlanPipeline.h"
#include "GeometryUtils.h"
#include "TrajectoryCore.h"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace SprayTrajPlan
{
    namespace
    {
        bool uvEndpointTouchesBoundary(const Eigen::Vector2d& point,
            const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& boundaryEdges,
            double tolerance)
        {
            if (!point.allFinite()) return false;
            const double toleranceSquared = tolerance * tolerance;
            for (const auto& [start, end] : boundaryEdges) {
                    const Eigen::Vector2d edge = end - start;
                    const double edgeSquared = edge.squaredNorm();
                    const double t = edgeSquared > EPS
                        ? std::clamp((point - start).dot(edge) / edgeSquared, 0.0, 1.0)
                        : 0.0;
                    if ((point - (start + t * edge)).squaredNorm() <= toleranceSquared) return true;
            }
            return false;
        }

        std::vector<TrajectorySegment> filterInvalidBoundaryEndpointTracks(
            const std::vector<TrajectorySegment>& trajectory,
            const ObjUVData& uvData,
            double overtravel)
        {
            std::map<std::pair<int, int>, int> edgeCounts;
            for (const auto& face : uvData.uv_faces) {
                for (int corner = 0; corner < 3; ++corner) {
                    const int first = face[corner];
                    const int second = face[(corner + 1) % 3];
                    if (first < 0 || second < 0
                        || static_cast<size_t>(first) >= uvData.uv.size()
                        || static_cast<size_t>(second) >= uvData.uv.size()) continue;
                    ++edgeCounts[{std::min(first, second), std::max(first, second)}];
                }
            }
            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> boundaryEdges;
            double uvExtent = 0.0;
            for (const auto& [edge, count] : edgeCounts) {
                if (count != 1) continue;
                boundaryEdges.emplace_back(uvData.uv[edge.first], uvData.uv[edge.second]);
                uvExtent = std::max(uvExtent, (uvData.uv[edge.first] - uvData.uv[edge.second]).norm());
            }
            if (boundaryEdges.empty()) return trajectory;
            const double uvTolerance = std::max(1.0e-8, uvExtent * 1.0e-6);

            std::vector<bool> removed(trajectory.size(), false);
            std::vector<size_t> centerIndices;
            std::vector<bool> centerInvalid;
            size_t scannedTracks = 0;
            for (size_t index = 0; index < trajectory.size(); ++index) {
                const auto& segment = trajectory[index];
                if (segment.kind != "SURFACE_SCAN") continue;
                ++scannedTracks;
                const bool startOnBoundary = segment.uv.size() >= 2
                    && uvEndpointTouchesBoundary(segment.uv.front(), boundaryEdges, uvTolerance);
                const bool endOnBoundary = segment.uv.size() >= 2
                    && uvEndpointTouchesBoundary(segment.uv.back(), boundaryEdges, uvTolerance);
                const bool invalid = !startOnBoundary || !endOnBoundary;
                if (segment.region == "CENTER") {
                    centerIndices.push_back(index);
                    centerInvalid.push_back(invalid);
                    continue;
                }
                if ((segment.region == "LOWER" || segment.region == "UPPER") && invalid) {
                    removed[index] = true;
                    std::cout << "[Track removed] " << segment.region
                        << " UV endpoint misses boundary: start=" << startOnBoundary
                        << ", end=" << endOnBoundary << std::endl;
                }
            }

            // Python keeps CENTER count intact.  The requested extension removes an
            // invalid CENTER path together with its mirrored CENTER path.
            for (size_t i = 0; i < centerIndices.size(); ++i) {
                if (!centerInvalid[i]) continue;
                const size_t mirrored = centerIndices.size() - 1 - i;
                removed[centerIndices[i]] = true;
                removed[centerIndices[mirrored]] = true;
                std::cout << "[Track removed] CENTER mirrored pair=" << i << ',' << mirrored
                    << " UV endpoint misses boundary" << std::endl;
            }

            const size_t removedCount = static_cast<size_t>(std::count(removed.begin(), removed.end(), true));
            if (removedCount == 0) {
                std::cout << "[Boundary endpoint filter] scans=" << scannedTracks
                    << ", boundary_edges=" << boundaryEdges.size() << ", removed=0" << std::endl;
                return trajectory;
            }

            std::vector<TrajectorySegment> result;
            result.reserve(trajectory.size());
            for (size_t index = 0; index < trajectory.size(); ++index) {
                const auto& segment = trajectory[index];
                const bool invalidConnector = segment.kind == "OVERTRAVEL"
                    && ((index > 0 && removed[index - 1])
                        || (index + 1 < removed.size() && removed[index + 1]));
                if (removed[index] || invalidConnector) continue;
                if (segment.kind == "SURFACE_SCAN" && !result.empty()
                    && result.back().kind == "SURFACE_SCAN") {
                    result.push_back(SeamHandler::createOvertravelSegment(
                        result.back(), segment, std::max(overtravel, 0.0)));
                }
                result.push_back(segment);
            }
            std::cout << "[Boundary endpoint filter] scans=" << scannedTracks
                << ", boundary_edges=" << boundaryEdges.size() << ", removed=" << removedCount << std::endl;
            return result;
        }

        void projectSurfaceScansToMesh(
            std::vector<TrajectorySegment>& trajectory,
            const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<Eigen::Vector3i>& faces)
        {
            if (vertices.empty() || faces.empty()) return;
            for (auto& segment : trajectory) {
                if (segment.kind != "SURFACE_SCAN" || segment.xyz.empty()) continue;
                std::vector<Eigen::Vector3d> projected;
                std::vector<double> distances;
                std::vector<int> faceIndices;
                GeometryUtils::closestPointsRobust(
                    vertices, faces, segment.xyz, projected, distances, faceIndices);
                if (projected.size() == segment.xyz.size()) segment.xyz = std::move(projected);
            }
        }
    }

    TrajPlanPipeline::TrajPlanPipeline()
        : optcuts_runner_(std::make_unique<OptCutsRunner>()),
          traj_generator_(std::make_unique<TrajectoryGenerator>())
    {
    }

    TrajPlanPipeline::~TrajPlanPipeline()
    {
    }

    bool TrajPlanPipeline::initialize()
    {
        // Initialize components
        std::cout << "Initializing trajectory planning pipeline..." << std::endl;
        return true;
    }

    void TrajPlanPipeline::setOptCutsExecutable(const std::string& exe_path)
    {
        optcuts_runner_->setOptCutsExecutable(exe_path);
    }

    void TrajPlanPipeline::setOptCutsRoot(const std::string& root_path)
    {
        optcuts_runner_->setOptCutsRoot(root_path);
    }

    bool TrajPlanPipeline::startUvMap(const std::string& input_mesh_path, const TrajectoryParams&)
    {
        OptCutsResult candidate;
        if (!optcuts_runner_->loadOrRunOptCuts(input_mesh_path, candidate)) return false;
        optcuts_result_ = std::move(candidate);
        uv_source_path_ = input_mesh_path;
        clearTrajectory();
        return true;
    }

    bool TrajPlanPipeline::startGetTraj(const TrajectoryParams& params)
    {
        if (!optcuts_result_.uv_data || !optcuts_result_.precut_seam) return false;
        std::vector<TrajectorySegment> candidate;
        PlanningFrame candidateFrame;
        TrajectoryMetadata candidateMetadata;
        const auto started = std::chrono::steady_clock::now();
        if (!traj_generator_->generateSeamAwareTrajectory(*optcuts_result_.uv_data, *optcuts_result_.precut_seam,
                params, candidate, candidateFrame, candidateMetadata)) return false;
        // TS.py displays this exact raw output. Do not alter it here with
        // arc fitting, boundary filtering, mesh projection, or a seam path.
        // Those are optimization-stage operations.
        candidateMetadata.mesh_path = optcuts_result_.mesh_path;
        candidateMetadata.optcuts_result_path = optcuts_result_.result_obj_path;
        candidateMetadata.source_type = std::filesystem::path(uv_source_path_).extension() == ".json" ? "cache" : "mesh";
        candidateMetadata.cache_format = candidateMetadata.source_type == "cache" ? "optcuts-cache-v2" : "";
        candidateMetadata.parameter_snapshot = params;
        generated_trajectory_ = candidate;
        final_trajectory_ = std::move(candidate);
        planning_frame_ = candidateFrame;
        metadata_ = candidateMetadata;
        size_t points = 0;
        for (const auto& segment : final_trajectory_) points += segment.xyz.size();
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        std::cout << "[UV timing] raw trajectory generation: " << std::fixed << std::setprecision(3) << seconds
            << " s, segments=" << final_trajectory_.size() << ", scan_points=" << points << std::endl;
        return true;
    }

    bool TrajPlanPipeline::startOptimize(const TrajectoryParams& params)
    {
        if (generated_trajectory_.empty()) return false;
        final_trajectory_ = generated_trajectory_;
        reportOptimizationProgress(0, "Preparing trajectory", final_trajectory_);
        return runOptimization(params);
    }

    void TrajPlanPipeline::reportOptimizationProgress(int percent, const std::string& stage,
        const std::vector<TrajectorySegment>& trajectory) const
    {
        if (optimization_progress_callback_) optimization_progress_callback_(percent, stage, trajectory);
    }

    bool TrajPlanPipeline::exportPaq(const std::string& paq_path, const TrajectoryParams& params)
    {
        if (final_trajectory_.empty()) return false;
        return TrajectoryExporter::exportPAQTrajectory(paq_path, optcuts_result_.mesh_vertices,
            optcuts_result_.mesh_faces, final_trajectory_, params.trajectory_speed,
            params.spray_distance, params.arc_speed_loss_pct);
    }

    void TrajPlanPipeline::clearModel()
    {
        optcuts_result_ = OptCutsResult{};
        uv_source_path_.clear();
        clearTrajectory();
    }

    void TrajPlanPipeline::clearTrajectory()
    {
        generated_trajectory_.clear();
        final_trajectory_.clear();
        metadata_ = TrajectoryMetadata{};
    }

    void TrajPlanPipeline::clearConnection()
    {
        const auto isConnection = [](const TrajectorySegment& segment) {
            return segment.kind == "TRANSITION" || segment.kind == "CONNECTION" ||
                segment.kind == "OVERTRAVEL" || segment.kind == "SEAM_TRANSITION";
        };
        final_trajectory_.erase(std::remove_if(final_trajectory_.begin(), final_trajectory_.end(), isConnection), final_trajectory_.end());
        generated_trajectory_.erase(std::remove_if(generated_trajectory_.begin(), generated_trajectory_.end(), isConnection), generated_trajectory_.end());
    }

    bool TrajPlanPipeline::runPipeline(
        const std::string& input_mesh_path,
        const std::string& output_csv_path,
        const std::string& output_metadata_path,
        const std::string& output_paq_path,
        const TrajectoryParams& params)
    {
        std::cout << "\n=== Starting Trajectory Planning Pipeline ===" << std::endl;
        std::cout << "Input mesh: " << input_mesh_path << std::endl;

        // Stage 1: Run OptCuts
        if (!runOptCutsStage(input_mesh_path)) {
            std::cerr << "OptCuts stage failed" << std::endl;
            return false;
        }

        // Stage 2: Generate trajectory
        if (!runTrajectoryGeneration(params)) {
            std::cerr << "Trajectory generation stage failed" << std::endl;
            return false;
        }

        // Stage 3: Optimize trajectory
        if (!runOptimization(params)) {
            std::cerr << "Trajectory optimization stage failed" << std::endl;
            return false;
        }

        // Stage 4: Export results
        if (!runExport(output_csv_path, output_metadata_path, output_paq_path, params)) {
            std::cerr << "Export stage failed" << std::endl;
            return false;
        }

        std::cout << "\n=== Pipeline Completed Successfully ===" << std::endl;
        std::cout << "Total trajectory segments: " << final_trajectory_.size() << std::endl;
        std::cout << "Total length: " << metadata_.total_length << " mm" << std::endl;

        return true;
    }

    bool TrajPlanPipeline::runOptCutsStage(const std::string& input_mesh)
    {
        std::cout << "\n[Stage 1/4] Running OptCuts..." << std::endl;

        if (!optcuts_runner_->loadOrRunOptCuts(input_mesh, optcuts_result_)) {
            return false;
        }
        uv_source_path_ = input_mesh;

        std::cout << "  - UV data loaded: " << optcuts_result_.uv_data->xyz.size() << " vertices" << std::endl;
        std::cout << "  - Precut seam: " << optcuts_result_.precut_seam->xyz.size() << " points" << std::endl;

        return true;
    }

    bool TrajPlanPipeline::runTrajectoryGeneration(const TrajectoryParams& params)
    {
        std::cout << "\n[Stage 2/4] Generating seam-aware trajectory..." << std::endl;
        std::cout << "  - Target spacing: " << params.target_spacing << " mm" << std::endl;
        std::cout << "  - Sample step: " << params.sample_step << " mm" << std::endl;

        if (!startGetTraj(params)) return false;
        std::cout << "  - Prepared " << final_trajectory_.size()
            << " trajectory segments after boundary filtering and arc fitting" << std::endl;
        return true;
    }

    bool TrajPlanPipeline::runOptimization(const TrajectoryParams& params)
    {
        std::cout << "\n[Stage 3/4] Optimizing trajectory..." << std::endl;
        const auto optimizationStarted = std::chrono::steady_clock::now();
        GeometryUtils::resetProjectionStatistics();
        size_t inputPoints = 0;
        for (const auto& segment : final_trajectory_) inputPoints += segment.xyz.size();
        std::cout << "[UV workload] vertices=" << optcuts_result_.mesh_vertices.size()
            << ", faces=" << optcuts_result_.mesh_faces.size()
            << ", segments=" << final_trajectory_.size()
            << ", trajectory_points=" << inputPoints << std::endl;

        // TS.py keeps Start Get Traj raw. Apply all geometric post-processing
        // only after the user explicitly starts optimization.
        std::vector<TrajectorySegment> preparedTrajectory;
        if (!traj_generator_->optimizeTrajectoryArcs(
                final_trajectory_, optcuts_result_.mesh_vertices, optcuts_result_.mesh_faces,
                params, preparedTrajectory)) return false;
        preparedTrajectory = filterInvalidBoundaryEndpointTracks(
            preparedTrajectory, *optcuts_result_.uv_data, 2.0 * params.target_spacing);
        projectSurfaceScansToMesh(
            preparedTrajectory, optcuts_result_.mesh_vertices, optcuts_result_.mesh_faces);
        const double sampleStep = params.sample_step > 0.0 ? params.sample_step : params.target_spacing / 4.0;
        if (!traj_generator_->appendSeamTransition(
                preparedTrajectory, *optcuts_result_.precut_seam, sampleStep)) return false;
        final_trajectory_ = std::move(preparedTrajectory);
        reportOptimizationProgress(0, "Preparing geometric trajectory", final_trajectory_);
        std::cout << "  - Optimizing trajectory energy..." << std::endl;
        std::vector<TrajectorySegment> relaxed_trajectory;

        GeometryUtils::ProjectionStatistics previousProjectionStats;
        double previousIterationSeconds = 0.0;
        if (!traj_generator_->optimizeCenterTrajectories(
                final_trajectory_,
                optcuts_result_.mesh_vertices,
                optcuts_result_.mesh_faces,
                params,
                relaxed_trajectory,
                [this, optimizationStarted, previousProjectionStats, previousIterationSeconds](int iteration, int total,
                    const std::vector<TrajectorySegment>& snapshot) mutable {
                    const int percent = (70 * iteration) / std::max(total, 1);
                    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - optimizationStarted).count();
                    const auto current = GeometryUtils::projectionStatistics();
                    const std::uint64_t cpuQueries = current.cpu_queries - previousProjectionStats.cpu_queries;
                    const std::uint64_t gpuQueries = current.gpu_queries - previousProjectionStats.gpu_queries;
                    const double projectionMs = (current.cpu_milliseconds - previousProjectionStats.cpu_milliseconds) +
                        (current.gpu_milliseconds - previousProjectionStats.gpu_milliseconds);
                    std::ostringstream stage;
                    stage << "Energy " << iteration << '/' << total << ": " << std::fixed << std::setprecision(2)
                        << (elapsed - previousIterationSeconds) << " s, projection " << projectionMs << " ms, CPU "
                        << cpuQueries << ", GPU " << gpuQueries << " queries";
                    std::cout << "[UV energy] iteration=" << iteration << '/' << total << ", iteration_time="
                        << std::fixed << std::setprecision(3) << (elapsed - previousIterationSeconds)
                        << " s, projection_time=" << projectionMs << " ms, cpu_queries=" << cpuQueries
                        << ", gpu_queries=" << gpuQueries << std::endl;
                    previousProjectionStats = current;
                    previousIterationSeconds = elapsed;
                    reportOptimizationProgress(percent, stage.str(), snapshot);
                })) {
            return false;
        }
        reportOptimizationProgress(70, "Optimizing center coverage", relaxed_trajectory);

        // B-spline fitting
        std::cout << "  - Fitting quintic B-splines..." << std::endl;
        std::vector<TrajectorySegment> smooth_trajectory;
        if (!traj_generator_->fitQuinticBSpline(
                relaxed_trajectory,
                optcuts_result_.mesh_vertices,
                optcuts_result_.mesh_faces,
                params,
                false,
                smooth_trajectory)) {
            return false;
        }
        reportOptimizationProgress(80, "Fitting quintic B-splines", smooth_trajectory);

        final_trajectory_ = smooth_trajectory;
        metadata_.total_segments = static_cast<int>(final_trajectory_.size());
        reportOptimizationProgress(100, "Finalizing optimized trajectory", final_trajectory_);

        const auto stats = GeometryUtils::projectionStatistics();
        const double totalSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - optimizationStarted).count();
        std::cout << "[UV timing] optimization=" << std::fixed << std::setprecision(3) << totalSeconds
            << " s; projection CPU batches=" << stats.cpu_batches << ", queries=" << stats.cpu_queries
            << ", time=" << stats.cpu_milliseconds << " ms; GPU batches=" << stats.gpu_batches
            << ", queries=" << stats.gpu_queries << ", time=" << stats.gpu_milliseconds << " ms" << std::endl;
        std::cout << "  - Optimization complete" << std::endl;

        return true;
    }

    bool TrajPlanPipeline::runExport(
        const std::string& csv_path,
        const std::string& metadata_path,
        const std::string& paq_path,
        const TrajectoryParams& params)
    {
        std::cout << "\n[Stage 4/4] Exporting results..." << std::endl;

        // Export CSV
        std::cout << "  - Exporting CSV: " << csv_path << std::endl;
        if (!TrajectoryExporter::exportTrajectoryCSV(csv_path, final_trajectory_, metadata_)) {
            std::cerr << "Failed to export CSV" << std::endl;
            return false;
        }

        // Export metadata JSON
        std::cout << "  - Exporting metadata: " << metadata_path << std::endl;
        if (!TrajectoryExporter::exportMetadataJSON(metadata_path, metadata_)) {
            std::cerr << "Failed to export metadata" << std::endl;
            return false;
        }

        // Export PAQ format
        std::cout << "  - Exporting PAQ format: " << paq_path << std::endl;
        if (!TrajectoryExporter::exportPAQTrajectory(
                paq_path,
                optcuts_result_.mesh_vertices,
                optcuts_result_.mesh_faces,
                final_trajectory_,
                params.trajectory_speed,
                params.spray_distance,
                0.0)) {
            std::cerr << "Failed to export PAQ" << std::endl;
            return false;
        }

        // Export matrix format
        std::string matrix_path = paq_path;
        size_t ext_pos = matrix_path.find_last_of('.');
        if (ext_pos != std::string::npos) {
            matrix_path = matrix_path.substr(0, ext_pos) + "_matrix.txt";
        } else {
            matrix_path += "_matrix.txt";
        }

        std::cout << "  - Exporting matrix format: " << matrix_path << std::endl;
        if (!TrajectoryExporter::exportMatrixFormat(
                matrix_path,
                final_trajectory_,
                params.trajectory_speed)) {
            std::cerr << "Failed to export matrix format" << std::endl;
            return false;
        }

        std::cout << "  - Export complete" << std::endl;

        return true;
    }

} // namespace SprayTrajPlan
