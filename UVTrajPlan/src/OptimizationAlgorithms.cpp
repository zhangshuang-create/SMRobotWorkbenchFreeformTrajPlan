#include "OptimizationAlgorithms.h"
#include "GeometryUtils.h"
#include "GpuMeshProjection.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>

namespace SprayTrajPlan
{
    namespace
    {
        double adaptiveVCutoff(double spacing)
        {
            const double d = std::max(spacing, EPS);
            const double ratio = std::clamp(0.5 + 2.5 * std::pow(10.0 / d, 2.32), 0.5, 5.0);
            return ratio * d;
        }

        std::pair<double, double> superellipse(double x, double a, double b, double n)
        {
            const double u = std::clamp(std::abs(x - a) / std::max(a, EPS), 1.0e-7, 1.0 - 1.0e-5);
            const double inside = std::max(0.0, 1.0 - std::pow(u, n));
            const double y = b * std::pow(inside, 1.0 / n);
            const double sign = x >= a ? 1.0 : -1.0;
            const double derivative = -(b / std::max(a, EPS)) * sign * std::pow(u, n - 1.0) *
                std::pow(std::max(inside, EPS), 1.0 / n - 1.0);
            return {y, derivative};
        }

        void projectToMesh(std::vector<Eigen::Vector3d>& points, const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<Eigen::Vector3i>& faces)
        {
            std::vector<Eigen::Vector3d> projected;
            std::vector<double> distances;
            std::vector<int> faceIds;
            GeometryUtils::closestPointsRobust(vertices, faces, points, projected, distances, faceIds);
            if (projected.size() == points.size()) points = std::move(projected);
        }

        std::vector<Eigen::Vector3d> pruneVSharpApex(const std::vector<Eigen::Vector3d>& xyz,
            double cutoff, const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& faces)
        {
            if (xyz.size() < 8 || cutoff <= EPS) return xyz;
            const Eigen::Vector3d chord = xyz.back() - xyz.front();
            if (chord.norm() <= EPS) return xyz;
            const Eigen::Vector3d direction = chord.normalized();
            size_t tip = 0;
            double greatest = -1.0;
            for (size_t i = 0; i < xyz.size(); ++i) {
                const Eigen::Vector3d lateral = xyz[i] - xyz.front() - (xyz[i] - xyz.front()).dot(direction) * direction;
                if (lateral.squaredNorm() > greatest) { greatest = lateral.squaredNorm(); tip = i; }
            }
            if (tip <= 2 || tip + 3 >= xyz.size()) return xyz;
            size_t first = tip, last = tip;
            bool found = false;
            for (size_t i = 0; i <= tip; ++i) for (size_t j = tip; j < xyz.size(); ++j) {
                if ((xyz[i] - xyz[j]).norm() < cutoff) { first = std::min(first, i); last = std::max(last, j); found = true; }
            }
            if (!found || first == 0 || last + 1 >= xyz.size()) return xyz;
            first = first > 0 ? first - 1 : first;
            last = std::min(last + 1, xyz.size() - 1);
            std::vector<Eigen::Vector3d> result;
            result.insert(result.end(), xyz.begin(), xyz.begin() + first + 1);
            result.push_back(0.5 * (xyz[first] + xyz[last]));
            result.insert(result.end(), xyz.begin() + last, xyz.end());
            if (result.size() > first + 1) {
                std::vector<Eigen::Vector3d> midpoint{result[first + 1]};
                projectToMesh(midpoint, vertices, faces);
                result[first + 1] = midpoint.front();
            }
            return result;
        }

        struct DepositionSample { Eigen::Vector3d point; Eigen::Vector3d normal; };

        std::vector<DepositionSample> surfaceSamples(const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<Eigen::Vector3i>& faces, size_t maximum = 1200)
        {
            std::vector<DepositionSample> samples;
            const size_t stride = std::max<size_t>(1, faces.size() / std::max<size_t>(1, maximum));
            for (size_t i = 0; i < faces.size(); i += stride) {
                const auto& f = faces[i];
                if (f.minCoeff() < 0 || f.maxCoeff() >= static_cast<int>(vertices.size())) continue;
                const Eigen::Vector3d normal = (vertices[f[1]] - vertices[f[0]]).cross(vertices[f[2]] - vertices[f[0]]);
                if (normal.norm() <= EPS) continue;
                samples.push_back({(vertices[f[0]] + vertices[f[1]] + vertices[f[2]]) / 3.0, normal.normalized()});
            }
            return samples;
        }

    }
    // ============================================================================
    // EnergyOptimizer Implementation
    // ============================================================================

    std::vector<std::map<std::string, std::vector<Eigen::Vector3d>>>
    EnergyOptimizer::extractLogicalSurfaceTracks(
        const std::vector<TrajectorySegment>& trajectory)
    {
        std::vector<std::map<std::string, std::vector<Eigen::Vector3d>>> tracks;

        for (const auto& segment : trajectory) {
            if (segment.kind != "SURFACE_SCAN" || segment.xyz.size() < 2) {
                continue;
            }

            std::map<std::string, std::vector<Eigen::Vector3d>> track;
            track["xyz"] = segment.xyz;
            track["region"] = {Eigen::Vector3d::Zero()};  // Store region as metadata

            tracks.push_back(track);
        }

        return tracks;
    }

    double EnergyOptimizer::computeBlankEnergy(
        const std::vector<std::vector<Eigen::Vector3d>>& tracks,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        double spacing)
    {
        if (tracks.size() < 2) return 0.0;

        double total_energy = 0.0;
        int sample_count = 0;

        // Sample points between adjacent tracks
        for (size_t i = 0; i < tracks.size() - 1; ++i) {
            const auto& track1 = tracks[i];
            const auto& track2 = tracks[i + 1];

            size_t min_size = std::min(track1.size(), track2.size());

            for (size_t j = 0; j < min_size; ++j) {
                // Midpoint between tracks
                Eigen::Vector3d midpoint = (track1[j] + track2[j]) * 0.5;

                // Find distance to nearest track point
                double min_dist = std::numeric_limits<double>::max();
                for (const auto& track : tracks) {
                    for (const auto& pt : track) {
                        double dist = (pt - midpoint).norm();
                        min_dist = std::min(min_dist, dist);
                    }
                }

                // Energy penalty for gaps larger than spacing
                double gap_ratio = min_dist / spacing;
                if (gap_ratio > 0.5) {
                    total_energy += std::pow(gap_ratio - 0.5, 2);
                }

                sample_count++;
            }
        }

        return sample_count > 0 ? total_energy / sample_count : 0.0;
    }

    double EnergyOptimizer::computeSmoothnessEnergy(
        const std::vector<std::vector<Eigen::Vector3d>>& tracks)
    {
        double total_energy = 0.0;
        int sample_count = 0;

        for (const auto& track : tracks) {
            if (track.size() < 3) continue;

            for (size_t i = 1; i < track.size() - 1; ++i) {
                // Compute curvature approximation
                Eigen::Vector3d v1 = track[i] - track[i - 1];
                Eigen::Vector3d v2 = track[i + 1] - track[i];

                double len1 = v1.norm();
                double len2 = v2.norm();

                if (len1 > EPS && len2 > EPS) {
                    v1 /= len1;
                    v2 /= len2;

                    double angle = std::acos(std::clamp(v1.dot(v2), -1.0, 1.0));
                    total_energy += angle * angle;
                    sample_count++;
                }
            }
        }

        return sample_count > 0 ? total_energy / sample_count : 0.0;
    }

    std::vector<Eigen::Vector3d> EnergyOptimizer::optimizeTrackPosition(
        const std::vector<Eigen::Vector3d>& track,
        const std::vector<std::vector<Eigen::Vector3d>>& neighbor_tracks,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const OptimizationParams& params)
    {
        std::vector<Eigen::Vector3d> optimized = track;

        // Simple gradient descent for each point
        for (size_t i = 1; i < optimized.size() - 1; ++i) {
            Eigen::Vector3d gradient = Eigen::Vector3d::Zero();

            // Smoothness gradient
            Eigen::Vector3d prev = optimized[i - 1];
            Eigen::Vector3d curr = optimized[i];
            Eigen::Vector3d next = optimized[i + 1];

            gradient += params.smoothness_weight * ((prev + next) - 2.0 * curr);

            // Blank energy gradient (push towards neighbors)
            for (const auto& neighbor : neighbor_tracks) {
                if (i < neighbor.size()) {
                    Eigen::Vector3d to_neighbor = neighbor[i] - curr;
                    double dist = to_neighbor.norm();
                    if (dist > params.spacing * 0.6) {
                        gradient += params.blank_weight * to_neighbor.normalized() * 0.1;
                    }
                }
            }

            // Update position
            optimized[i] += gradient * params.sample_step * 0.01;
        }

        return optimized;
    }

    bool EnergyOptimizer::optimizeCenterTrajectoriesDualBlankEnergy(
        const std::vector<TrajectorySegment>& input_trajectory,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const OptimizationParams& params,
        std::vector<TrajectorySegment>& output_trajectory,
        const std::function<void(int, int, const std::vector<TrajectorySegment>&)>& progress_callback)
    {
        // Structural port of OptAeigenTest.optimize_center_trajectories_dual_blank_energy:
        // each iteration runs independent CENTER, LOWER and UPPER force passes.
        // The non-moving passes form the fixed boundary field for the active pass.
        output_trajectory = input_trajectory;
        const double d = std::max(params.spacing, EPS);
        // Match OptAeigenTest: energy relaxation operates on one control node
        // per nominal track spacing, not the dense d/4 export samples.
        const double sampleStep = std::max(params.sample_step, d);
        const int iterations = std::max(1, params.max_iterations);

        auto resample = [&](const std::vector<Eigen::Vector3d>& line) {
            if (line.size() < 2) return line;
            std::vector<double> lengths(line.size(), 0.0);
            for (size_t i = 1; i < line.size(); ++i) lengths[i] = lengths[i - 1] + (line[i] - line[i - 1]).norm();
            const double total = lengths.back();
            if (total <= EPS) return line;
            const int count = std::max(2, static_cast<int>(std::ceil(total / sampleStep)) + 1);
            std::vector<Eigen::Vector3d> out; out.reserve(count);
            size_t segment = 1;
            for (int i = 0; i < count; ++i) {
                const double s = total * i / static_cast<double>(count - 1);
                while (segment + 1 < lengths.size() && lengths[segment] < s) ++segment;
                const double span = std::max(lengths[segment] - lengths[segment - 1], EPS);
                out.push_back(line[segment - 1] + ((s - lengths[segment - 1]) / span) * (line[segment] - line[segment - 1]));
            }
            out.front() = line.front(); out.back() = line.back();
            return out;
        };
        auto nearest = [](const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& points) {
            size_t best = 0; double best2 = std::numeric_limits<double>::max();
            for (size_t i = 0; i < points.size(); ++i) { const double q = (point - points[i]).squaredNorm(); if (q < best2) { best2 = q; best = i; } }
            return std::pair<size_t, double>{best, std::sqrt(best2)};
        };
        auto indicesFor = [](const std::vector<TrajectorySegment>& trajectory, const std::string& region) {
            std::vector<size_t> out;
            for (size_t i = 0; i < trajectory.size(); ++i)
                if (trajectory[i].kind == "SURFACE_SCAN" && trajectory[i].region == region && trajectory[i].xyz.size() >= 2) out.push_back(i);
            return out;
        };
        auto allPoints = [&](const std::vector<TrajectorySegment>& trajectory, const std::vector<size_t>& excluded) {
            std::vector<Eigen::Vector3d> out;
            for (size_t i = 0; i < trajectory.size(); ++i) {
                const bool isFixedBoundary = trajectory[i].kind == "SURFACE_SCAN" || trajectory[i].kind == "SEAM_TRANSITION";
                if (std::find(excluded.begin(), excluded.end(), i) != excluded.end() || !isFixedBoundary) continue;
                const auto sampled = resample(trajectory[i].xyz); out.insert(out.end(), sampled.begin(), sampled.end());
            }
            return out;
        };
        const auto triangleSamples = surfaceSamples(mesh_vertices, mesh_faces, 3000);
        std::vector<Eigen::Vector3d> triangleSamplePoints;
        triangleSamplePoints.reserve(triangleSamples.size());
        for (const auto& sample : triangleSamples) triangleSamplePoints.push_back(sample.point);

        auto updateRegion = [&](std::vector<TrajectorySegment>& trajectory, const std::string& region) {
            const auto moving = indicesFor(trajectory, region);
            if (moving.empty()) return;
            std::vector<std::vector<Eigen::Vector3d>> lines; lines.reserve(moving.size());
            for (size_t index : moving) lines.push_back(resample(trajectory[index].xyz));
            const auto fixed = allPoints(trajectory, moving);
            std::vector<Eigen::Vector3d> seamPoints;
            for (const auto& segment : trajectory) {
                if (segment.kind != "SEAM_TRANSITION" || segment.xyz.size() < 2) continue;
                const auto sampled = resample(segment.xyz);
                seamPoints.insert(seamPoints.end(), sampled.begin(), sampled.end());
            }

            // LOWER/UPPER only relax their paths nearest CENTER, matching the
            // Python red-boundary selection.  Other paths remain fixed leaders.
            std::vector<bool> movable(lines.size(), true);
            if (region != "CENTER" && lines.size() > 1) {
                std::vector<Eigen::Vector3d> centers;
                for (size_t ci : indicesFor(trajectory, "CENTER")) { const auto s = resample(trajectory[ci].xyz); centers.insert(centers.end(), s.begin(), s.end()); }
                std::vector<std::pair<double, size_t>> rank;
                for (size_t i = 0; i < lines.size(); ++i) {
                    double gap = std::numeric_limits<double>::max();
                    for (const auto& p : lines[i]) if (!centers.empty()) gap = std::min(gap, nearest(p, centers).second);
                    rank.emplace_back(gap, i);
                }
                std::sort(rank.begin(), rank.end());
                std::fill(movable.begin(), movable.end(), false);
                for (size_t i = 0; i < (rank.size() + 1) / 2; ++i) movable[rank[i].second] = true;
            }

            std::vector<std::vector<Eigen::Vector3d>> gpuNext;
            if (relaxEnergyRegionCuda(lines, movable, fixed, seamPoints,
                    triangleSamplePoints, region == "CENTER" ? 0 : 1, d, gpuNext)) {
                for (size_t li = 0; li < gpuNext.size(); ++li) {
                    const Eigen::Vector3d first = lines[li].front(), last = lines[li].back();
                    projectToMesh(gpuNext[li], mesh_vertices, mesh_faces);
                    gpuNext[li].front() = first;
                    gpuNext[li].back() = last;
                    trajectory[moving[li]].xyz = std::move(gpuNext[li]);
                }
                return;
            }

            // _blank_guidance_force in the Python implementation is evaluated
            // from a small set of blank centres, not once per line point per
            // mesh sample.  Derive the current dominant uncovered centre once
            // for this regional pass, then reuse it for all CENTER samples.
            Eigen::Vector3d blankCenter = Eigen::Vector3d::Zero();
            double blankCoverage = 0.0;
            if (region == "CENTER") {
                for (const auto& sample : triangleSamples) {
                    double coverage = std::numeric_limits<double>::max();
                    for (const auto& f : fixed) coverage = std::min(coverage, (sample.point - f).norm());
                    for (const auto& line : lines) for (const auto& q : line) coverage = std::min(coverage, (sample.point - q).norm());
                    if (coverage > blankCoverage) { blankCoverage = coverage; blankCenter = sample.point; }
                }
            }

            std::vector<std::vector<Eigen::Vector3d>> next = lines;
            for (size_t li = 0; li < lines.size(); ++li) {
                if (!movable[li]) continue;
                for (size_t pi = 1; pi + 1 < lines[li].size(); ++pi) {
                    const Eigen::Vector3d p = lines[li][pi];
                    Eigen::Vector3d force = (region == "CENTER" ? 0.80 : 2.40) * (lines[li][pi - 1] - 2.0 * p + lines[li][pi + 1]);

                    // Python's same-line non-local self-repulsion (radius 1.8d).
                    for (size_t qi = 0; qi < lines[li].size(); ++qi) {
                        if (std::abs(static_cast<long long>(qi) - static_cast<long long>(pi)) <= 6) continue;
                        const Eigen::Vector3d delta = p - lines[li][qi]; const double distance = delta.norm();
                        if (distance > EPS && distance < 1.8 * d) force += 0.30 * (1.0 - distance / (1.8 * d)) * delta / distance;
                    }

                    // Directed d-equilibrium springs.  Only this follower moves.
                    auto directed = [&](const std::vector<Eigen::Vector3d>& leader, double target, double gain) {
                        if (leader.empty()) return;
                        const auto [ni, distance] = nearest(p, leader);
                        if (distance <= EPS) return;
                        const double error = (distance - target) / target;
                        const double deadT = std::clamp(std::abs(error) / 0.015, 0.0, 1.0);
                        const double dead = deadT * deadT * (3.0 - 2.0 * deadT);
                        const double mag = (distance < target ? 6.0 * std::min(target / distance - 1.0, 4.0) : -6.0 * std::min(distance / target - 1.0, 1.0)) * dead * gain * 0.5;
                        force += (p - leader[ni]) / distance * mag;
                    };
                    if (region == "CENTER") {
                        if (li > 0) directed(lines[li - 1], d, 0.680);
                        if (li + 1 < lines.size()) directed(lines[li + 1], d, 0.680);
                        // The pre-cut seam is a fixed leader for C0/C_last.
                        // It must be present during energy optimization, not
                        // appended only after every center path has converged.
                        if (!seamPoints.empty() && (li == 0 || li + 1 == lines.size())) directed(seamPoints, d, 3.0 * 0.680);
                        else if (lines.size() > 1 && li == 0) directed(lines.back(), 2.0 * d, 0.680);
                        else if (lines.size() > 1 && li + 1 == lines.size()) directed(lines.front(), 2.0 * d, 0.680);
                    } else {
                        if (li > 0) directed(lines[li - 1], d, 0.680);
                        else directed(fixed, d, 0.680);
                    }

                    // Coverage guidance is the closest uncovered triangle-centre
                    // direction, with the same 0.5d residual threshold as Python.
                    if (region == "CENTER" && blankCoverage > 0.5 * d) {
                        const Eigen::Vector3d delta = blankCenter - p; const double distance = delta.norm();
                        // Same smooth dead zone as _blank_guidance_force:
                        // no attraction within 0.5d and full response at 1.5d.
                        const double t = std::clamp((distance - 0.5 * d) / d, 0.0, 1.0);
                        if (distance > EPS) force += 0.05 * t * t * (3.0 - 2.0 * t) * delta / distance;
                    }
                    const double norm = force.norm();
                    if (norm > EPS) next[li][pi] += std::min(0.12 * d, norm) * force / norm;
                }
            }
            for (size_t li = 0; li < next.size(); ++li) {
                const Eigen::Vector3d first = lines[li].front(), last = lines[li].back();
                projectToMesh(next[li], mesh_vertices, mesh_faces);
                next[li].front() = first; next[li].back() = last;
                trajectory[moving[li]].xyz = std::move(next[li]);
            }
        };

        for (int iteration = 0; iteration < iterations; ++iteration) {
            updateRegion(output_trajectory, "CENTER");
            updateRegion(output_trajectory, "LOWER");
            updateRegion(output_trajectory, "UPPER");
            // Python emits on_iteration after every full coupled pass.  Always
            // invoke this callback, including a geometrically stable pass.
            if (progress_callback) progress_callback(iteration + 1, iterations, output_trajectory);
        }
        return true;
    }

    // ============================================================================
    // ArcOptimizer Implementation
    // ============================================================================

    std::vector<int> ArcOptimizer::detectSharpCorners(
        const std::vector<Eigen::Vector3d>& points,
        double angle_threshold)
    {
        std::vector<int> corners;

        if (points.size() < 3) return corners;

        for (size_t i = 1; i < points.size() - 1; ++i) {
            Eigen::Vector3d v1 = (points[i] - points[i - 1]).normalized();
            Eigen::Vector3d v2 = (points[i + 1] - points[i]).normalized();

            double angle = std::acos(std::clamp(v1.dot(v2), -1.0, 1.0));

            if (angle > angle_threshold) {
                corners.push_back(static_cast<int>(i));
            }
        }

        return corners;
    }

    std::vector<Eigen::Vector3d> ArcOptimizer::fitCircularArc(
        const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1,
        const Eigen::Vector3d& p2,
        double max_deviation)
    {
        // Simple 3-point arc approximation
        std::vector<Eigen::Vector3d> arc_points;

        Eigen::Vector3d v1 = p1 - p0;
        Eigen::Vector3d v2 = p2 - p1;

        double len1 = v1.norm();
        double len2 = v2.norm();

        if (len1 < EPS || len2 < EPS) {
            return {p0, p1, p2};
        }

        // Compute arc center (simplified)
        double blend = std::min(max_deviation / len1, 0.5);

        Eigen::Vector3d arc_start = p1 - v1.normalized() * len1 * blend;
        Eigen::Vector3d arc_end = p1 + v2.normalized() * len2 * blend;

        // Sample arc
        int n_samples = 5;
        for (int i = 0; i <= n_samples; ++i) {
            double t = static_cast<double>(i) / n_samples;
            // Quadratic Bezier
            Eigen::Vector3d pt = (1 - t) * (1 - t) * arc_start +
                                2 * (1 - t) * t * p1 +
                                t * t * arc_end;
            arc_points.push_back(pt);
        }

        return arc_points;
    }

    bool ArcOptimizer::checkKinematicConstraints(
        const std::vector<Eigen::Vector3d>& points,
        double speed,
        double a_max,
        double j_max)
    {
        if (points.size() < 2) return true;

        // Check maximum acceleration needed
        for (size_t i = 1; i < points.size() - 1; ++i) {
            Eigen::Vector3d v1 = points[i] - points[i - 1];
            Eigen::Vector3d v2 = points[i + 1] - points[i];

            double dt1 = v1.norm() / speed;
            double dt2 = v2.norm() / speed;

            if (dt1 < EPS || dt2 < EPS) continue;

            Eigen::Vector3d vel1 = v1 / dt1;
            Eigen::Vector3d vel2 = v2 / dt2;
            Eigen::Vector3d accel = (vel2 - vel1) / ((dt1 + dt2) / 2.0);

            if (accel.norm() > a_max * 1.1) {
                return false;
            }
        }

        return true;
    }

    bool ArcOptimizer::fitSuperellipseTrajectories(
        const std::vector<TrajectorySegment>& input_trajectory,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const ArcParams& params,
        std::vector<TrajectorySegment>& output_trajectory)
    {
        output_trajectory = input_trajectory;
        const double step = params.sample_step > EPS ? params.sample_step : std::max(params.spacing / 4.0, 1.0);
        const double cutoff = adaptiveVCutoff(params.spacing);
        int fitted = 0;

        for (auto& segment : output_trajectory) {
            if (segment.kind != "SURFACE_SCAN" || (segment.region != "LOWER" && segment.region != "UPPER") ||
                segment.xyz.size() < 5) continue;
            const auto raw = pruneVSharpApex(segment.xyz, cutoff, mesh_vertices, mesh_faces);
            // Python discards boundary V paths that collapse during the cutoff.
            if (raw.size() < 5) {
                segment = TrajectorySegment{};
                segment.kind = "REMOVED";
                continue;
            }
            const Eigen::Vector3d p1 = raw.front(), p3 = raw.back();
            const Eigen::Vector3d chord = p3 - p1;
            const double chordLength = chord.norm();
            if (chordLength <= EPS) continue;
            const Eigen::Vector3d xDir = chord / chordLength;
            size_t tip = 0;
            double maxDistance = -1.0;
            for (size_t i = 0; i < raw.size(); ++i) {
                const Eigen::Vector3d lateral = raw[i] - p1 - (raw[i] - p1).dot(xDir) * xDir;
                if (lateral.squaredNorm() > maxDistance) { maxDistance = lateral.squaredNorm(); tip = i; }
            }
            if (tip < 2 || tip + 2 >= raw.size()) continue;
            const Eigen::Vector3d midpoint = 0.5 * (p1 + p3);
            Eigen::Vector3d yAxis = raw[tip] - midpoint - (raw[tip] - midpoint).dot(xDir) * xDir;
            const double height = yAxis.norm();
            if (height <= EPS) continue;
            yAxis /= height;
            const double a = 0.5 * chordLength;

            // Bounded grid search is deterministic and mirrors Python's bounded n/b fit.
            double bestN = 1.0, bestB = height, bestLoss = std::numeric_limits<double>::max();
            const double minB = std::max(0.3 * height, height - 2.0 * params.spacing);
            for (int ni = 0; ni <= 39; ++ni) for (int bi = 0; bi <= 23; ++bi) {
                const double n = 0.05 + (1.95 * ni / 39.0);
                const double b = minB + ((height - minB) * bi / 23.0);
                double loss = 0.0;
                for (const auto& point : raw) {
                    const Eigen::Vector3d relative = point - midpoint;
                    const double x = relative.dot(xDir) + a;
                    const double y = relative.dot(yAxis);
                    const double residual = superellipse(x, a, b, n).first - y;
                    loss += residual * residual * (std::abs(x - a) < 0.6 * a ? 2.5 : 1.0);
                }
                if (loss < bestLoss) { bestLoss = loss; bestN = n; bestB = b; }
            }

            const int count = std::max(32, static_cast<int>(std::ceil((2.0 * a) / step)));
            std::vector<Eigen::Vector3d> curve;
            curve.reserve(count);
            for (int i = 0; i < count; ++i) {
                const double x = 2.0 * a * i / static_cast<double>(count - 1);
                const double y = superellipse(x, a, bestB, bestN).first;
                curve.push_back(midpoint + (x - a) * xDir + y * yAxis);
            }
            curve.front() = p1;
            curve.back() = p3;
            projectToMesh(curve, mesh_vertices, mesh_faces);
            curve.front() = p1;
            curve.back() = p3;
            const Eigen::Vector2d startUv = segment.uv.empty()
                ? Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()) : segment.uv.front();
            const Eigen::Vector2d endUv = segment.uv.empty()
                ? Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()) : segment.uv.back();
            segment.xyz = std::move(curve);
            segment.uv.assign(segment.xyz.size(), Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()));
            segment.uv.front() = startUv;
            segment.uv.back() = endUv;
            segment.phases.assign(segment.xyz.size(), 1);
            std::fill(segment.phases.begin(), segment.phases.begin() + segment.phases.size() / 2, 0);
            segment.geometry_meta = std::map<std::string, double>{{"superellipse", 1.0},
                {"superellipse_a", a}, {"superellipse_b", bestB}, {"superellipse_n", bestN},
                {"v_cutoff", cutoff}};
            segment.note += " [Raw Superellipse]";
            ++fitted;
        }
        output_trajectory.erase(std::remove_if(output_trajectory.begin(), output_trajectory.end(),
            [](const TrajectorySegment& segment) { return segment.kind == "REMOVED"; }), output_trajectory.end());
        std::cout << "Superellipse fitting: fitted " << fitted << " V trajectories" << std::endl;
        return true;
    }

    bool ArcOptimizer::applyTangentCircleFillets(
        const std::vector<TrajectorySegment>& input_trajectory,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const ArcParams& params,
        std::vector<TrajectorySegment>& output_trajectory)
    {
        output_trajectory = input_trajectory;
        const double step = params.sample_step > EPS ? params.sample_step : std::max(params.spacing / 4.0, 1.0);
        const double radius = std::max(params.speed * params.speed / std::max(params.a_max, EPS),
            std::sqrt(params.speed * params.speed * params.speed / std::max(params.j_max, EPS)));
        const double cutoff = adaptiveVCutoff(params.spacing);
        int fitted = 0;

        for (auto& segment : output_trajectory) {
            if (segment.kind != "SURFACE_SCAN" || (segment.region != "LOWER" && segment.region != "UPPER") ||
                !segment.geometry_meta || !segment.geometry_meta->count("superellipse")) continue;
            const auto raw = segment.xyz;
            const Eigen::Vector3d p1 = raw.front(), p3 = raw.back();
            const Eigen::Vector3d chord = p3 - p1;
            const double chordLength = chord.norm();
            if (chordLength <= EPS) continue;
            const Eigen::Vector3d xDir = chord / chordLength;
            size_t tip = 0;
            double maxDistance = -1.0;
            for (size_t i = 0; i < raw.size(); ++i) {
                const Eigen::Vector3d lateral = raw[i] - p1 - (raw[i] - p1).dot(xDir) * xDir;
                if (lateral.squaredNorm() > maxDistance) { maxDistance = lateral.squaredNorm(); tip = i; }
            }
            if (tip < 2 || tip + 2 >= raw.size()) continue;
            const Eigen::Vector3d midpoint = 0.5 * (p1 + p3);
            Eigen::Vector3d yAxis = raw[tip] - midpoint - (raw[tip] - midpoint).dot(xDir) * xDir;
            const double height = yAxis.norm();
            if (height <= EPS) continue;
            yAxis /= height;
            const double a = 0.5 * chordLength;
            const double bestN = segment.geometry_meta->at("superellipse_n");
            const double bestB = segment.geometry_meta->at("superellipse_b");

            double low = std::max(0.01 * a, 0.1), high = a - 1.0e-3;
            const auto tangentResidual = [&](double x) {
                const auto [y, dy] = superellipse(x, a, bestB, bestN);
                if (std::abs(dy) <= 1.0e-6) return -radius;
                const double centerY = y - (a - x) / dy;
                return std::hypot(x - a, y - centerY) - radius;
            };
            double rootLow = low, rootHigh = high;
            bool bracketed = false;
            double previousX = low, previousF = tangentResidual(low);
            double minResidual = std::isfinite(previousF)
                ? previousF : std::numeric_limits<double>::infinity();
            for (int i = 1; i <= 256; ++i) {
                const double x = low + (high - low) * i / 256.0;
                const double f = tangentResidual(x);
                if (std::isfinite(f)) minResidual = std::min(minResidual, f);
                if (std::isfinite(f) && std::isfinite(previousF) && previousF * f <= 0.0) {
                    rootLow = previousX; rootHigh = x; bracketed = true; break;
                }
                previousX = x; previousF = f;
            }
            // Python condition C: when every analytic tangent radius already
            // exceeds R_min, retain the superellipse.  Forcing the spatial B
            // arc here is what folded otherwise valid neighbouring tracks
            // together in the UV/3D preview.
            const double radiusTolerance = std::max(1.0e-6, 1.0e-4 * std::max(radius, params.spacing));
            if (std::isfinite(minResidual) && minResidual > radiusTolerance) {
                std::cout << "[Retained superellipse] region=" << segment.region
                    << ", min_tangent_radius=" << radius + minResidual
                    << ", R_min=" << radius << std::endl;
                continue;
            }
            if (!bracketed) {
                // Python B branch: a spatial minimum-radius arc is used only
                // when the surface-superellipse tangent solve has no solution.
                const Eigen::Vector3d apex = raw[tip];
                Eigen::Vector3d arm1 = p1 - apex, arm2 = p3 - apex;
                if (arm1.norm() <= EPS || arm2.norm() <= EPS) continue;
                const Eigen::Vector3d u1 = arm1.normalized(), u2 = arm2.normalized();
                const double angle = std::clamp(std::acos(std::clamp(u1.dot(u2), -1.0, 1.0)),
                    2.0 * 3.14159265358979323846 / 180.0, 178.0 * 3.14159265358979323846 / 180.0);
                const double tangentDistance = radius / std::max(std::tan(0.5 * angle), 1.0e-4);
                const double centerDistance = radius / std::max(std::sin(0.5 * angle), 1.0e-4);
                Eigen::Vector3d bisector = u1 + u2;
                if (bisector.norm() <= EPS) bisector = yAxis; else bisector.normalize();
                const Eigen::Vector3d center = apex + centerDistance * bisector;
                std::vector<Eigen::Vector3d> centerQuery{center}, centerProjected;
                std::vector<double> centerDistances; std::vector<int> centerFaces;
                GeometryUtils::closestPointsRobust(mesh_vertices, mesh_faces, centerQuery, centerProjected, centerDistances, centerFaces);
                const double centerLimit = params.center_boundary_limit > EPS ? params.center_boundary_limit : radius + 2.0 * params.spacing;
                if (centerDistances.empty() || centerDistances.front() > centerLimit) continue;
                const Eigen::Vector3d t1 = apex + tangentDistance * u1, t2 = apex + tangentDistance * u2;
                Eigen::Vector3d start = (t1 - center).normalized(), finish = (t2 - center).normalized();
                Eigen::Vector3d axis = start.cross(finish);
                if (axis.norm() <= EPS) axis = start.cross(bisector);
                if (axis.norm() <= EPS) continue;
                axis.normalize();
                const double sweep = 3.14159265358979323846 - angle;
                const double extension = 0.5 * angle;
                const int before = std::max(2, static_cast<int>(std::ceil(radius * extension / step)) + 1);
                const int between = std::max(3, static_cast<int>(std::ceil(radius * sweep / step)) + 1);
                std::vector<Eigen::Vector3d> curve;
                curve.reserve(2 * before + between - 2);
                for (int i = 0; i < 2 * before + between - 2; ++i) {
                    const double phase = i < before ? -extension + extension * i / (before - 1.0) :
                        (i < before + between - 1 ? sweep * (i - before + 1) / (between - 1.0) :
                        sweep + extension * (i - before - between + 2) / (before - 1.0));
                    const Eigen::Vector3d radial = start * std::cos(phase) + axis.cross(start) * std::sin(phase) +
                        axis * axis.dot(start) * (1.0 - std::cos(phase));
                    curve.push_back(center + radius * radial);
                }
                if (curve.size() < 2) continue;
                curve[before - 1] = t1;
                curve[before + between - 2] = t2;
                const double chordMidDistance = (center - midpoint).norm();
                if (chordMidDistance <= radius - 2.0 * params.spacing) {
                    std::vector<Eigen::Vector3d> interior(curve.begin() + before - 1, curve.begin() + before + between - 1);
                    projectToMesh(interior, mesh_vertices, mesh_faces);
                    std::copy(interior.begin(), interior.end(), curve.begin() + before - 1);
                    curve[before - 1] = t1; curve[before + between - 2] = t2;
                }
                const Eigen::Vector2d startUv = segment.uv.empty()
                    ? Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()) : segment.uv.front();
                const Eigen::Vector2d endUv = segment.uv.empty()
                    ? Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()) : segment.uv.back();
                segment.xyz = std::move(curve);
                segment.uv.assign(segment.xyz.size(), Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()));
                segment.uv.front() = startUv;
                segment.uv.back() = endUv;
                segment.phases.assign(segment.xyz.size(), 1);
                std::fill(segment.phases.begin(), segment.phases.begin() + segment.phases.size() / 2, 0);
                segment.geometry_meta = std::map<std::string, double>{{"arc_locked", 1.0}, {"arc_radius", radius},
                    {"superellipse_a", a}, {"superellipse_b", bestB}, {"superellipse_n", bestN}, {"v_cutoff", cutoff}, {"spatial_arc", 1.0}};
                segment.note += " [Spatial minimum-radius arc]";
                ++fitted;
                continue;
            }
            for (int i = 0; i < 48; ++i) {
                const double mid = 0.5 * (rootLow + rootHigh);
                if (tangentResidual(rootLow) * tangentResidual(mid) <= 0.0) rootHigh = mid; else rootLow = mid;
            }
            const double xTangent = 0.5 * (rootLow + rootHigh);
            const auto [yTangent, derivative] = superellipse(xTangent, a, bestB, bestN);
            if (std::abs(derivative) <= EPS) continue;
            const double centerY = yTangent - (a - xTangent) / derivative;
            const double xRight = 2.0 * a - xTangent;
            const double phi0 = std::atan2(yTangent - centerY, xTangent - a);
            const double phi1 = std::atan2(yTangent - centerY, xRight - a);
            const int count = std::max(16, static_cast<int>(std::ceil((2.0 * a + radius * std::abs(phi1 - phi0)) / step)));
            std::vector<Eigen::Vector3d> curve;
            curve.reserve(count);
            for (int i = 0; i < count; ++i) {
                const double x = 2.0 * a * i / static_cast<double>(count - 1);
                double y = superellipse(x, a, bestB, bestN).first;
                if (x >= xTangent && x <= xRight) {
                    const double ratio = (x - xTangent) / std::max(xRight - xTangent, EPS);
                    const double phi = phi0 + ratio * (phi1 - phi0);
                    y = centerY + radius * std::sin(phi);
                }
                curve.push_back(midpoint + (x - a) * xDir + y * yAxis);
            }
            curve.front() = p1; curve.back() = p3;
            projectToMesh(curve, mesh_vertices, mesh_faces);
            curve.front() = p1; curve.back() = p3;
            const Eigen::Vector2d startUv = segment.uv.empty()
                ? Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()) : segment.uv.front();
            const Eigen::Vector2d endUv = segment.uv.empty()
                ? Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()) : segment.uv.back();
            segment.xyz = std::move(curve);
            segment.uv.assign(segment.xyz.size(), Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()));
            segment.uv.front() = startUv;
            segment.uv.back() = endUv;
            segment.phases.assign(segment.xyz.size(), 1);
            std::fill(segment.phases.begin(), segment.phases.begin() + segment.phases.size() / 2, 0);
            segment.geometry_meta = std::map<std::string, double>{{"arc_locked", 1.0}, {"arc_radius", radius},
                {"superellipse_a", a}, {"superellipse_b", bestB}, {"superellipse_n", bestN}, {"v_cutoff", cutoff}};
            segment.note += " [Superellipse tangent fillet]";
            ++fitted;
        }
        std::cout << "Arc optimization: fitted " << fitted << " analytic superellipse fillets" << std::endl;
        return true;
    }

    bool ArcOptimizer::optimizeTrajectoryArcs(
        const std::vector<TrajectorySegment>& input_trajectory,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const ArcParams& params,
        std::vector<TrajectorySegment>& output_trajectory)
    {
        std::vector<TrajectorySegment> superellipseTrajectory;
        if (!fitSuperellipseTrajectories(input_trajectory, mesh_vertices, mesh_faces, params, superellipseTrajectory)) return false;
        return applyTangentCircleFillets(superellipseTrajectory, mesh_vertices, mesh_faces, params, output_trajectory);
    }

    // B-spline implementation continues...
    // (Implementation is quite involved, using simplified version for now)

    bool BSplineFitter::fitQuinticBSpline(
        const std::vector<TrajectorySegment>& input_trajectory,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const SplineParams& params,
        std::vector<TrajectorySegment>& output_trajectory)
    {
        output_trajectory = input_trajectory;

        // Simplified B-spline smoothing
        for (auto& segment : output_trajectory) {
            if (segment.kind != "SURFACE_SCAN" || segment.xyz.size() < 6) {
                continue;
            }
            if (segment.geometry_meta && segment.geometry_meta->count("arc_locked") &&
                segment.geometry_meta->at("arc_locked") > 0.5) {
                continue;
            }

            // Apply moving average smoothing as simplified B-spline
            std::vector<Eigen::Vector3d> smoothed = segment.xyz;
            int window = 3;

            for (size_t i = window; i < segment.xyz.size() - window; ++i) {
                Eigen::Vector3d avg = Eigen::Vector3d::Zero();
                for (int j = -window; j <= window; ++j) {
                    avg += segment.xyz[i + j];
                }
                smoothed[i] = avg / (2 * window + 1);
            }

            segment.xyz = smoothed;
        }

        return true;
    }

} // namespace SprayTrajPlan
