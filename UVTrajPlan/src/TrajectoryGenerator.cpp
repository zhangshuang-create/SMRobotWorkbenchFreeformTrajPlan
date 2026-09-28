#include "TrajectoryGenerator.h"
#include "TrajectoryCore.h"
#include "GeometryUtils.h"
#include "OptimizationAlgorithms.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace SprayTrajPlan
{
    TrajectoryGenerator::TrajectoryGenerator()
    {
    }

    TrajectoryGenerator::~TrajectoryGenerator()
    {
    }

    namespace {
        struct UvFrame {
            Eigen::Vector2d origin = Eigen::Vector2d::Zero();
            Eigen::Vector2d xAxis = Eigen::Vector2d::UnitX();
            Eigen::Vector2d yAxis = Eigen::Vector2d::UnitY();
            double waistWidth = 0.0;
            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> seamPairs;
        };

        bool makeUvFrame(const ObjUVData& data, const PrecutSeam& seam, UvFrame& frame)
        {
            std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> edgeToUv;
            for (size_t faceIndex = 0; faceIndex < data.xyz_faces.size() && faceIndex < data.uv_faces.size(); ++faceIndex) {
                const auto& xyzFace = data.xyz_faces[faceIndex];
                const auto& uvFace = data.uv_faces[faceIndex];
                for (int local = 0; local < 3; ++local) {
                    const int a = xyzFace[local], b = xyzFace[(local + 1) % 3];
                    const int ua = uvFace[local], ub = uvFace[(local + 1) % 3];
                    if (a < 0 || b < 0 || ua < 0 || ub < 0 || ua >= static_cast<int>(data.uv.size()) || ub >= static_cast<int>(data.uv.size())) continue;
                    edgeToUv[{std::min(a, b), std::max(a, b)}].emplace_back(a < b ? ua : ub, a < b ? ub : ua);
                }
            }
            struct Pair { Eigen::Vector2d a; Eigen::Vector2d b; };
            std::vector<Pair> samples;
            for (size_t index = 0; index + 1 < seam.cut_vertex_pairs.size(); ++index) {
                const auto [a0, b0] = seam.cut_vertex_pairs[index];
                const auto [a1, b1] = seam.cut_vertex_pairs[index + 1];
                const auto lookup = [&edgeToUv, &data](int first, int second, Eigen::Vector2d& start, Eigen::Vector2d& end) {
                    const auto found = edgeToUv.find({std::min(first, second), std::max(first, second)});
                    if (found == edgeToUv.end() || found->second.empty()) return false;
                    const auto chosen = *std::max_element(found->second.begin(), found->second.end(), [&data](const auto& left, const auto& right) {
                        return (data.uv[left.second] - data.uv[left.first]).squaredNorm() < (data.uv[right.second] - data.uv[right.first]).squaredNorm();
                    });
                    const auto oriented = first < second ? chosen : std::pair<int, int>{chosen.second, chosen.first};
                    start = data.uv[oriented.first]; end = data.uv[oriented.second];
                    return true;
                };
                Eigen::Vector2d aa0, aa1, bb0, bb1;
                if (!lookup(a0, a1, aa0, aa1) || !lookup(b0, b1, bb0, bb1)) continue;
                const Eigen::Vector2d difference0 = bb0 - aa0;
                const Eigen::Vector2d differenceStep = (bb1 - bb0) - (aa1 - aa0);
                const double denominator = differenceStep.squaredNorm();
                const double ratio = denominator <= EPS ? 0.0 : std::clamp(-difference0.dot(differenceStep) / denominator, 0.0, 1.0);
                samples.push_back({aa0 + ratio * (aa1 - aa0), bb0 + ratio * (bb1 - bb0)});
            }
            if (samples.empty() && seam.xyz.size() >= 2) {
                // cache-v1 does not serialize cut_vertex_pairs. Recover the
                // two UV copies of every pre-cut seam point from repeated OBJ
                // geometry, which is the same relation Python reconstructs
                // before it plans tracks.
                double scale = 1.0;
                for (const auto& point : data.xyz) scale = std::max(scale, point.cwiseAbs().maxCoeff());
                const double matchTolerance = 1.0e-6 * scale;
                for (const auto& seamPoint : seam.xyz) {
                    std::vector<int> candidates;
                    for (size_t vertex = 0; vertex < data.xyz.size(); ++vertex) {
                        if ((data.xyz[vertex] - seamPoint).norm() <= matchTolerance) candidates.push_back(static_cast<int>(vertex));
                    }
                    Eigen::Vector2d first, second;
                    double widest = -1.0;
                    for (size_t left = 0; left < candidates.size(); ++left) {
                        for (size_t right = left + 1; right < candidates.size(); ++right) {
                            const double width = (data.uv[candidates[left]] - data.uv[candidates[right]]).squaredNorm();
                            if (width > widest) { widest = width; first = data.uv[candidates[left]]; second = data.uv[candidates[right]]; }
                        }
                    }
                    if (widest > EPS * EPS) samples.push_back({first, second});
                }
            }
            if (samples.empty()) {
                // Last-resort compatibility path for malformed legacy caches.
                if (data.uv.size() < 3) return false;
                Eigen::Vector2d center = Eigen::Vector2d::Zero();
                for (const auto& uv : data.uv) center += uv;
                center /= static_cast<double>(data.uv.size());
                Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
                for (const auto& uv : data.uv) { const Eigen::Vector2d delta = uv - center; covariance += delta * delta.transpose(); }
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
                if (solver.info() != Eigen::Success) return false;
                frame.xAxis = solver.eigenvectors().col(1).normalized();
                frame.yAxis = Eigen::Vector2d(-frame.xAxis.y(), frame.xAxis.x());
                double minX = std::numeric_limits<double>::max(), minY = minX, maxX = -minX;
                for (const auto& uv : data.uv) {
                    const Eigen::Vector2d delta = uv - center;
                    minX = std::min(minX, delta.dot(frame.xAxis)); maxX = std::max(maxX, delta.dot(frame.xAxis));
                    minY = std::min(minY, delta.dot(frame.yAxis));
                }
                frame.origin = center + minX * frame.xAxis + minY * frame.yAxis;
                frame.waistWidth = maxX - minX;
                return frame.waistWidth > EPS;
            }
            // Keep side A/B consistent along the seam. Recovered OBJ copies
            // have no intrinsic ordering, so choose the orientation closest to
            // the preceding pair before applying Python's counterpart mapping.
            for (size_t index = 1; index < samples.size(); ++index) {
                const double direct = (samples[index].a - samples[index - 1].a).squaredNorm() +
                    (samples[index].b - samples[index - 1].b).squaredNorm();
                const double swapped = (samples[index].b - samples[index - 1].a).squaredNorm() +
                    (samples[index].a - samples[index - 1].b).squaredNorm();
                if (swapped < direct) std::swap(samples[index].a, samples[index].b);
            }
            frame.seamPairs.clear();
            for (const auto& sample : samples) frame.seamPairs.emplace_back(sample.a, sample.b);
            const auto narrowest = std::min_element(samples.begin(), samples.end(), [](const Pair& left, const Pair& right) {
                return (left.b - left.a).squaredNorm() < (right.b - right.a).squaredNorm();
            });
            const Eigen::Vector2d axis = narrowest->b - narrowest->a;
            if (axis.norm() <= EPS) return false;
            frame.origin = narrowest->a;
            frame.xAxis = axis.normalized();
            frame.yAxis = Eigen::Vector2d(-frame.xAxis.y(), frame.xAxis.x());
            frame.waistWidth = axis.norm();
            const Eigen::Vector2d longitudinal = samples.back().a + samples.back().b - samples.front().a - samples.front().b;
            if (longitudinal.dot(frame.yAxis) < 0.0) frame.yAxis = -frame.yAxis;
            return true;
        }

        Eigen::Vector2d toLocal(const Eigen::Vector2d& point, const UvFrame& frame)
        {
            const Eigen::Vector2d delta = point - frame.origin;
            return {delta.dot(frame.xAxis), delta.dot(frame.yAxis)};
        }

        Eigen::Vector2d fromLocal(const Eigen::Vector2d& point, const UvFrame& frame)
        {
            return frame.origin + point.x() * frame.xAxis + point.y() * frame.yAxis;
        }

        bool seamProjectionAndCounterpart(const Eigen::Vector2d& point, const UvFrame& frame,
            Eigen::Vector2d& source, Eigen::Vector2d& counterpart)
        {
            if (frame.seamPairs.size() < 2) return false;
            double bestDistance = std::numeric_limits<double>::max();
            for (int side = 0; side < 2; ++side) {
                for (size_t index = 0; index + 1 < frame.seamPairs.size(); ++index) {
                    const Eigen::Vector2d start = side == 0 ? frame.seamPairs[index].first : frame.seamPairs[index].second;
                    const Eigen::Vector2d end = side == 0 ? frame.seamPairs[index + 1].first : frame.seamPairs[index + 1].second;
                    const Eigen::Vector2d targetStart = side == 0 ? frame.seamPairs[index].second : frame.seamPairs[index].first;
                    const Eigen::Vector2d targetEnd = side == 0 ? frame.seamPairs[index + 1].second : frame.seamPairs[index + 1].first;
                    const Eigen::Vector2d direction = end - start;
                    const double lengthSquared = direction.squaredNorm();
                    const double ratio = lengthSquared <= EPS ? 0.0 : std::clamp(direction.dot(point - start) / lengthSquared, 0.0, 1.0);
                    const Eigen::Vector2d projected = start + ratio * direction;
                    const double distance = (point - projected).squaredNorm();
                    if (distance < bestDistance) {
                        bestDistance = distance;
                        source = projected;
                        counterpart = targetStart + ratio * (targetEnd - targetStart);
                    }
                }
            }
            return std::isfinite(bestDistance);
        }

        std::vector<Eigen::Vector3d> smoothVTrajectory(const std::vector<Eigen::Vector3d>& points,
            double minimumDistance = 0.2, int iterations = 2, double weight = 0.25)
        {
            if (points.size() < 3) return points;
            std::vector<Eigen::Vector3d> cleaned;
            cleaned.reserve(points.size());
            cleaned.push_back(points.front());
            for (size_t index = 1; index < points.size(); ++index) {
                if ((points[index] - cleaned.back()).norm() > minimumDistance || index + 1 == points.size()) {
                    cleaned.push_back(points[index]);
                }
            }
            if (cleaned.size() < 3) return cleaned;
            for (int iteration = 0; iteration < iterations; ++iteration) {
                const std::vector<Eigen::Vector3d> previous = cleaned;
                for (size_t index = 1; index + 1 < cleaned.size(); ++index) {
                    const Eigen::Vector3d laplacian = 0.5 * (previous[index - 1] + previous[index + 1]) - previous[index];
                    cleaned[index] = previous[index] + weight * laplacian;
                }
            }
            return cleaned;
        }

        std::vector<std::pair<double, double>> lineDomainIntervals(const std::vector<Eigen::Vector2d>& localUv,
            const std::vector<Eigen::Vector3i>& faces, double x, double tolerance)
        {
            std::vector<std::pair<double, double>> raw;
            for (const auto& face : faces) {
                if (face.minCoeff() < 0 || face.maxCoeff() >= static_cast<int>(localUv.size())) continue;
                const Eigen::Vector2d triangle[3] = {localUv[face[0]], localUv[face[1]], localUv[face[2]]};
                double minX = triangle[0].x(), maxX = minX;
                for (const auto& vertex : triangle) { minX = std::min(minX, vertex.x()); maxX = std::max(maxX, vertex.x()); }
                if (x < minX - tolerance || x > maxX + tolerance) continue;
                std::vector<double> intersections;
                for (int edge = 0; edge < 3; ++edge) {
                    const Eigen::Vector2d& a = triangle[edge];
                    const Eigen::Vector2d& b = triangle[(edge + 1) % 3];
                    const double dx = b.x() - a.x();
                    if (std::abs(dx) <= tolerance) {
                        if (std::abs(x - a.x()) <= tolerance) { intersections.push_back(a.y()); intersections.push_back(b.y()); }
                        continue;
                    }
                    const double ratio = (x - a.x()) / dx;
                    if (ratio >= -tolerance && ratio <= 1.0 + tolerance) intersections.push_back(a.y() + ratio * (b.y() - a.y()));
                }
                if (intersections.size() >= 2) {
                    const auto bounds = std::minmax_element(intersections.begin(), intersections.end());
                    if (*bounds.second - *bounds.first > tolerance) raw.emplace_back(*bounds.first, *bounds.second);
                }
            }
            if (raw.empty()) return raw;
            std::sort(raw.begin(), raw.end());
            std::vector<std::pair<double, double>> merged{raw.front()};
            for (size_t i = 1; i < raw.size(); ++i) {
                if (raw[i].first <= merged.back().second + 10.0 * tolerance) merged.back().second = std::max(merged.back().second, raw[i].second);
                else merged.push_back(raw[i]);
            }
            return merged;
        }

        std::optional<std::pair<double, double>> regionInterval(const std::vector<Eigen::Vector2d>& localUv,
            const std::vector<Eigen::Vector3i>& faces, double x, const std::string& region, double tolerance)
        {
            const auto intervals = lineDomainIntervals(localUv, faces, x, tolerance);
            double low = std::numeric_limits<double>::max(), high = -low;
            for (const auto& interval : intervals) {
                const double midpoint = 0.5 * (interval.first + interval.second);
                if ((region == "LOWER" && midpoint > tolerance) || (region == "UPPER" && midpoint < -tolerance)) continue;
                low = std::min(low, interval.first); high = std::max(high, interval.second);
            }
            if (low > high) return std::nullopt;
            return std::make_pair(low, high);
        }

        // Helper: Extract waist dimensions from UV data
        Eigen::Vector3d extractWaistDimensions(const ObjUVData& uv_data, const PlanningFrame& frame) {
            // Find waist width (center), upper and lower extents
            double min_y = std::numeric_limits<double>::max();
            double max_y = -std::numeric_limits<double>::max();

            for (const auto& uv : uv_data.uv) {
                min_y = std::min(min_y, uv.y());
                max_y = std::max(max_y, uv.y());
            }

            double center_width = max_y - min_y;

            // Simplified: assume symmetric
            double upper_ref = center_width * 0.3;
            double lower_ref = center_width * 0.3;

            return Eigen::Vector3d(center_width, upper_ref, lower_ref);
        }

        // Helper: Generate scan lines in UV space
        std::vector<std::vector<Eigen::Vector2d>> generateScanLines(
            const ObjUVData& uv_data,
            const PlanningFrame& frame,
            const std::vector<double>& x_positions,
            double sample_step,
            const std::string& region)
        {
            std::vector<std::vector<Eigen::Vector2d>> scan_lines;

            // Find Y bounds
            double min_y = std::numeric_limits<double>::max();
            double max_y = -std::numeric_limits<double>::max();

            for (const auto& uv : uv_data.uv) {
                min_y = std::min(min_y, uv.y());
                max_y = std::max(max_y, uv.y());
            }

            bool upward = true;
            for (double x : x_positions) {
                auto scan = TrajectorySampler::sampleScanUV(
                    x,
                    {min_y, max_y},
                    upward,
                    sample_step,
                    frame
                );
                scan_lines.push_back(scan);
                upward = !upward;  // Alternate direction
            }

            return scan_lines;
        }
    }

    bool TrajectoryGenerator::generateSeamAwareTrajectory(
        const ObjUVData& uv_data,
        const PrecutSeam& precut_seam,
        const TrajectoryParams& params,
        std::vector<TrajectorySegment>& trajectory,
        PlanningFrame& planning_frame,
        TrajectoryMetadata& metadata)
    {
        trajectory.clear();

        // TS.py emits the raw displayed trajectory at the requested sample
        // pitch (normally d/4), before any optimization-stage resampling.
        const double requested_step = params.sample_step > 0.0 ?
            params.sample_step : params.target_spacing / 4.0;
        const double sample_step = requested_step;
        UvFrame uvFrame;
        if (!makeUvFrame(uv_data, precut_seam, uvFrame)) {
            std::cerr << "Unable to derive the UV planning frame from the pre-cut seam" << std::endl;
            return false;
        }
        planning_frame.origin = Eigen::Vector3d(uvFrame.origin.x(), uvFrame.origin.y(), 0.0);
        planning_frame.x_axis = Eigen::Vector3d(uvFrame.xAxis.x(), uvFrame.xAxis.y(), 0.0);
        planning_frame.y_axis = Eigen::Vector3d(uvFrame.yAxis.x(), uvFrame.yAxis.y(), 0.0);
        planning_frame.waist_width = uvFrame.waistWidth;

        std::vector<Eigen::Vector2d> localUv;
        localUv.reserve(uv_data.uv.size());
        for (const auto& uv : uv_data.uv) localUv.push_back(toLocal(uv, uvFrame));
        double minX = std::numeric_limits<double>::max(), maxX = -minX;
        for (const auto& uv : localUv) { minX = std::min(minX, uv.x()); maxX = std::max(maxX, uv.x()); }
        const double tolerance = 1.0e-9 * std::max({std::abs(minX), std::abs(maxX), uvFrame.waistWidth, 1.0});
        double lowerLeft = 0.0, lowerRight = 0.0, upperLeft = 0.0, upperRight = 0.0;
        for (const auto& uv : localUv) {
            if (uv.y() <= tolerance) { lowerLeft = std::max(lowerLeft, -uv.x()); lowerRight = std::max(lowerRight, uv.x() - uvFrame.waistWidth); }
            if (uv.y() >= -tolerance) { upperLeft = std::max(upperLeft, -uv.x()); upperRight = std::max(upperRight, uv.x() - uvFrame.waistWidth); }
        }
        const Eigen::Vector3d dimensions(uvFrame.waistWidth, std::min(upperLeft, upperRight), std::min(lowerLeft, lowerRight));
        const SpacingOptimum optimum = SpacingOptimizer::optimizeSpacing(params.target_spacing, dimensions);
        std::cout << "Symmetric spacing: " << optimum.spacing << " mm, counts " << optimum.counts.transpose() << std::endl;
        UVToXYZMapper mapper(uv_data);
        const auto makeTrack = [&](double x, const std::string& region, bool upward) -> std::optional<TrajectorySegment> {
            const auto interval = regionInterval(localUv, uv_data.uv_faces, x, region, tolerance);
            if (!interval || interval->second - interval->first < 2.0 * optimum.spacing) return std::nullopt;
            const double start = upward ? interval->first : interval->second;
            const double stop = upward ? interval->second : interval->first;
            const int count = std::max(2, static_cast<int>(std::ceil(std::abs(stop - start) / sample_step)) + 1);
            TrajectorySegment segment;
            segment.kind = "SURFACE_SCAN";
            segment.region = region;
            segment.spray_on = true;
            segment.note = region + " clipped scan";
            segment.uv.reserve(count);
            for (int i = 0; i < count; ++i) {
                const double ratio = static_cast<double>(i) / static_cast<double>(count - 1);
                segment.uv.push_back(fromLocal({x, start + ratio * (stop - start)}, uvFrame));
                segment.phases.push_back(i < count / 2 ? 0 : 1);
            }
            try { segment.xyz = mapper.map(segment.uv, 0.5 * sample_step, true); }
            catch (const std::exception&) { return std::nullopt; }
            return segment.xyz.size() >= 2 ? std::optional<TrajectorySegment>(std::move(segment)) : std::nullopt;
        };

        std::optional<size_t> lastSurfaceIndex;
        const auto appendSurface = [&](TrajectorySegment&& segment) {
            if (lastSurfaceIndex) {
                trajectory.push_back(SeamHandler::createOvertravelSegment(trajectory[*lastSurfaceIndex], segment, 2.0 * optimum.spacing));
            }
            trajectory.push_back(std::move(segment));
            lastSurfaceIndex = trajectory.size() - 1;
        };
        const auto pairedScans = [&](double extent, int count, const std::string& region, bool masterUpward) {
            std::vector<TrajectorySegment> left, right;
            const auto [leftPositions, ignoredLeftSpacing] = SpacingOptimizer::balancedOuterPositions(extent, count, 0.0, -1.0);
            const auto [rightPositions, ignoredRightSpacing] = SpacingOptimizer::balancedOuterPositions(extent, count, uvFrame.waistWidth, 1.0);
            const bool innerToOuter = region == "LOWER";
            const auto collectTracks = [&](const std::vector<double>& positions, std::vector<TrajectorySegment>& target) {
                if (innerToOuter) {
                    for (auto it = positions.rbegin(); it != positions.rend(); ++it) {
                        if (auto scan = makeTrack(*it, region, masterUpward)) target.push_back(std::move(*scan));
                    }
                }
                else {
                    for (const double position : positions) {
                        if (auto scan = makeTrack(position, region, masterUpward)) target.push_back(std::move(*scan));
                    }
                }
            };
            collectTracks(leftPositions, left);
            collectTracks(rightPositions, right);
            const size_t pairs = std::min(left.size(), right.size());
            for (size_t index = 0; index < pairs; ++index) {
                // TS.py chooses the nearer side independently for every pair.
                // Do not impose an extra alternating-side policy here.
                bool masterLeft = true;
                if (lastSurfaceIndex) {
                    const auto& lastSurface = trajectory[*lastSurfaceIndex];
                    const double leftGap = (lastSurface.xyz.back() - left[index].xyz.front()).norm();
                    const double rightGap = (lastSurface.xyz.back() - right[index].xyz.front()).norm();
                    masterLeft = leftGap <= rightGap;
                }
                auto first = masterLeft ? left[index] : right[index];
                auto second = masterLeft ? makeTrack(toLocal(right[index].uv.front(), uvFrame).x(), region, !masterUpward)
                    : makeTrack(toLocal(left[index].uv.front(), uvFrame).x(), region, !masterUpward);
                if (!second) continue;
                // Project the end of the master pass onto its actual UV seam
                // copy and start the slave pass at the matching copy.  The two
                // copies are distinct in UV but coincide on the surface.
                Eigen::Vector2d seamSource, seamCounterpart;
                if (seamProjectionAndCounterpart(first.uv.back(), uvFrame, seamSource, seamCounterpart)) {
                    first.uv.back() = seamSource;
                    second->uv.front() = seamCounterpart;
                    try {
                        first.xyz = mapper.map(first.uv, 0.5 * sample_step, true);
                        second->xyz = mapper.map(second->uv, 0.5 * sample_step, true);
                    }
                    catch (const std::exception&) { continue; }
                    if (first.xyz.size() < 2 || second->xyz.size() < 2) continue;
                }
                // Both halves meet at the physical cut. Keeping them in one
                // scan segment matches Python's phase-aware paired passes.
                first.note = region + (masterLeft ? " left/right paired scan" : " right/left paired scan");
                first.uv.insert(first.uv.end(), second->uv.begin(), second->uv.end());
                first.xyz.insert(first.xyz.end(), second->xyz.begin(), second->xyz.end());
                first.phases.assign(first.xyz.size(), 1);
                const size_t split = first.xyz.size() - second->xyz.size();
                std::fill(first.phases.begin(), first.phases.begin() + split, 0);
                if (region == "LOWER") {
                    const double splitRatio = static_cast<double>(split) / static_cast<double>(first.xyz.size());
                    first.xyz = smoothVTrajectory(first.xyz);
                    first.phases.assign(first.xyz.size(), 1);
                    const size_t smoothedSplit = static_cast<size_t>(std::ceil(splitRatio * first.xyz.size()));
                    std::fill(first.phases.begin(), first.phases.begin() + std::min(smoothedSplit, first.phases.size()), 0);
                }
                appendSurface(std::move(first));
            }
        };
        pairedScans(dimensions[2], optimum.counts[2], "LOWER", true);
        const auto [centerPositions, unusedCenterSpacing] = SpacingOptimizer::balancedCenterPositions(uvFrame.waistWidth, optimum.counts[0]);
        std::vector<double> orderedCenterPositions = centerPositions;
        if (lastSurfaceIndex && centerPositions.size() > 1) {
            const auto forward = makeTrack(centerPositions.front(), "CENTER", true);
            const auto reverse = makeTrack(centerPositions.back(), "CENTER", true);
            const auto& lastSurface = trajectory[*lastSurfaceIndex];
            if (forward && reverse && (lastSurface.xyz.back() - reverse->xyz.front()).norm() <
                (lastSurface.xyz.back() - forward->xyz.front()).norm()) {
                std::reverse(orderedCenterPositions.begin(), orderedCenterPositions.end());
            }
        }
        for (size_t index = 0; index < orderedCenterPositions.size(); ++index) {
            if (auto scan = makeTrack(orderedCenterPositions[index], "CENTER", index % 2 == 0)) appendSurface(std::move(*scan));
        }
        pairedScans(dimensions[1], optimum.counts[1], "UPPER", false);
        if (trajectory.empty()) return false;

        // Fill metadata
        metadata.spacing = optimum.spacing;
        metadata.speed = params.trajectory_speed;
        metadata.spray_distance = params.spray_distance;
        metadata.total_segments = static_cast<int>(trajectory.size());

        // Compute total length
        double total_length = 0.0;
        for (const auto& seg : trajectory) {
            for (size_t i = 1; i < seg.xyz.size(); ++i) {
                total_length += (seg.xyz[i] - seg.xyz[i-1]).norm();
            }
        }
        metadata.total_length = total_length;

        return true;
    }

    bool TrajectoryGenerator::generateBasicTrajectory(
        const ObjUVData& uv_data,
        double target_spacing,
        double sample_step,
        std::vector<TrajectorySegment>& trajectory)
    {
        trajectory.clear();

        // Compute planning frame first
        PlanningFrame temp_frame;
        if (!computePlanningFrame(uv_data, temp_frame)) {
            std::cerr << "Failed to compute planning frame" << std::endl;
            return false;
        }

        // Extract dimensions
        Eigen::Vector3d dimensions = extractWaistDimensions(uv_data, temp_frame);

        // Optimize spacing
        auto spacing_result = SpacingOptimizer::optimizeSpacing(target_spacing, dimensions);
        std::cout << "Optimized spacing: " << spacing_result.spacing << " mm" << std::endl;
        std::cout << "Center count: " << spacing_result.counts[0] << std::endl;
        std::cout << "Upper count: " << spacing_result.counts[1] << std::endl;
        std::cout << "Lower count: " << spacing_result.counts[2] << std::endl;

        // Generate center positions
        auto [center_positions, center_spacing] = SpacingOptimizer::balancedCenterPositions(
            dimensions[0], spacing_result.counts[0]
        );

        // Find waist X position (middle of UV domain)
        double min_x = std::numeric_limits<double>::max();
        double max_x = -std::numeric_limits<double>::max();
        for (const auto& uv : uv_data.uv) {
            min_x = std::min(min_x, uv.x());
            max_x = std::max(max_x, uv.x());
        }
        double waist_x = (min_x + max_x) / 2.0;

        // Create UV to XYZ mapper
        UVToXYZMapper mapper(uv_data);

        // Generate CENTER scans
        auto center_scan_lines = generateScanLines(
            uv_data, temp_frame, center_positions, sample_step, "CENTER"
        );

        for (size_t i = 0; i < center_scan_lines.size(); ++i) {
            TrajectorySegment seg;
            seg.kind = "SURFACE_SCAN";
            seg.region = "CENTER";
            seg.spray_on = true;
            seg.note = "Center scan " + std::to_string(i);
            seg.uv = center_scan_lines[i];

            try {
                seg.xyz = mapper.map(seg.uv, sample_step * 0.5, true);
            } catch (const std::exception& e) {
                std::cerr << "Warning: UV mapping failed for center scan " << i
                         << ": " << e.what() << std::endl;
                continue;
            }

            if (!seg.xyz.empty()) {
                trajectory.push_back(seg);
            }
        }

        // Generate UPPER scans if needed
        if (spacing_result.counts[1] > 0) {
            auto [upper_positions, upper_spacing] = SpacingOptimizer::balancedOuterPositions(
                dimensions[1], spacing_result.counts[1], waist_x, 1.0
            );

            auto upper_scan_lines = generateScanLines(
                uv_data, temp_frame, upper_positions, sample_step, "UPPER"
            );

            for (size_t i = 0; i < upper_scan_lines.size(); ++i) {
                TrajectorySegment seg;
                seg.kind = "SURFACE_SCAN";
                seg.region = "UPPER";
                seg.spray_on = true;
                seg.note = "Upper scan " + std::to_string(i);
                seg.uv = upper_scan_lines[i];

                try {
                    seg.xyz = mapper.map(seg.uv, sample_step * 0.5, true);
                } catch (const std::exception& e) {
                    std::cerr << "Warning: UV mapping failed for upper scan " << i << std::endl;
                    continue;
                }

                if (!seg.xyz.empty()) {
                    trajectory.push_back(seg);
                }
            }
        }

        // Generate LOWER scans if needed
        if (spacing_result.counts[2] > 0) {
            auto [lower_positions, lower_spacing] = SpacingOptimizer::balancedOuterPositions(
                dimensions[2], spacing_result.counts[2], waist_x, -1.0
            );

            auto lower_scan_lines = generateScanLines(
                uv_data, temp_frame, lower_positions, sample_step, "LOWER"
            );

            for (size_t i = 0; i < lower_scan_lines.size(); ++i) {
                TrajectorySegment seg;
                seg.kind = "SURFACE_SCAN";
                seg.region = "LOWER";
                seg.spray_on = true;
                seg.note = "Lower scan " + std::to_string(i);
                seg.uv = lower_scan_lines[i];

                try {
                    seg.xyz = mapper.map(seg.uv, sample_step * 0.5, true);
                } catch (const std::exception& e) {
                    std::cerr << "Warning: UV mapping failed for lower scan " << i << std::endl;
                    continue;
                }

                if (!seg.xyz.empty()) {
                    trajectory.push_back(seg);
                }
            }
        }

        // Add overtravel connectors between scans
        std::vector<TrajectorySegment> with_connectors;
        for (size_t i = 0; i < trajectory.size(); ++i) {
            with_connectors.push_back(trajectory[i]);

            if (i < trajectory.size() - 1) {
                // Add connector
                double overtravel = sample_step * 2.0;
                try {
                    auto connector = SeamHandler::createOvertravelSegment(
                        trajectory[i], trajectory[i + 1], overtravel
                    );
                    with_connectors.push_back(connector);
                } catch (const std::exception& e) {
                    std::cerr << "Warning: Failed to create connector: " << e.what() << std::endl;
                }
            }
        }

        trajectory = with_connectors;

        std::cout << "Generated " << trajectory.size() << " trajectory segments" << std::endl;
        return !trajectory.empty();
    }

    bool TrajectoryGenerator::computePlanningFrame(
        const ObjUVData& uv_data,
        PlanningFrame& frame)
    {
        if (uv_data.xyz.empty()) {
            return false;
        }

        // Compute centroid as origin
        frame.origin = GeometryUtils::closedPolylineCenter(uv_data.xyz);

        // Compute principal axes using PCA
        Eigen::MatrixXd points(uv_data.xyz.size(), 3);
        for (size_t i = 0; i < uv_data.xyz.size(); ++i) {
            points.row(i) = uv_data.xyz[i];
        }

        Eigen::MatrixXd centered = points.rowwise() - frame.origin.transpose();
        Eigen::MatrixXd covariance = (centered.transpose() * centered) / double(points.rows() - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);
        if (solver.info() != Eigen::Success) {
            return false;
        }

        // Extract principal axes (sorted by eigenvalue)
        frame.x_axis = solver.eigenvectors().col(2).normalized();
        frame.y_axis = solver.eigenvectors().col(1).normalized();

        frame.waist_width = 0.0;
        frame.waist_pair_index = 0;

        return true;
    }

    bool TrajectoryGenerator::optimizeTrajectoryArcs(
        const std::vector<TrajectorySegment>& input_trajectory,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const TrajectoryParams& params,
        std::vector<TrajectorySegment>& output_trajectory)
    {
        ArcOptimizer::ArcParams arc_params;
        arc_params.speed = params.trajectory_speed;
        arc_params.a_max = params.a_max;
        arc_params.j_max = params.j_max;
        arc_params.spacing = params.target_spacing;
        arc_params.sample_step = params.sample_step;
        arc_params.center_boundary_limit = params.arc_center_limit;
        arc_params.arc_lock_guard_d = params.arc_lock_guard_d;

        return ArcOptimizer::optimizeTrajectoryArcs(
            input_trajectory,
            mesh_vertices,
            mesh_faces,
            arc_params,
            output_trajectory
        );
    }

    bool TrajectoryGenerator::optimizeCenterTrajectories(
        const std::vector<TrajectorySegment>& input_trajectory,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const TrajectoryParams& params,
        std::vector<TrajectorySegment>& output_trajectory,
        const std::function<void(int, int, const std::vector<TrajectorySegment>&)>& progress_callback)
    {
        EnergyOptimizer::OptimizationParams opt_params;
        opt_params.spacing = params.target_spacing;
        opt_params.sample_step = params.sample_step > EPS ? params.sample_step : params.target_spacing / 4.0;
        opt_params.speed = params.trajectory_speed;
        opt_params.spray_distance = params.spray_distance;
        // Match optimize_center_trajectories_dual_blank_energy in
        // OptAeigenTest.py.  Do not shorten this here: the UI receives a
        // snapshot after every complete coupled iteration.
        opt_params.max_iterations = 140;
        opt_params.convergence_threshold = 1e-4;
        opt_params.blank_weight = 1.0;
        opt_params.smoothness_weight = 0.5;

        return EnergyOptimizer::optimizeCenterTrajectoriesDualBlankEnergy(
            input_trajectory,
            mesh_vertices,
            mesh_faces,
            opt_params,
            output_trajectory,
            progress_callback
        );
    }

    bool TrajectoryGenerator::fitQuinticBSpline(
        const std::vector<TrajectorySegment>& input_trajectory,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const TrajectoryParams& params,
        bool project_to_mesh,
        std::vector<TrajectorySegment>& output_trajectory)
    {
        BSplineFitter::SplineParams spline_params;
        spline_params.degree = 5;
        spline_params.smoothness = 0.1;
        spline_params.project_to_mesh = project_to_mesh;

        return BSplineFitter::fitQuinticBSpline(
            input_trajectory,
            mesh_vertices,
            mesh_faces,
            spline_params,
            output_trajectory
        );
    }

    bool TrajectoryGenerator::appendSeamTransition(
        std::vector<TrajectorySegment>& trajectory,
        const PrecutSeam& precut_seam,
        double sample_step)
    {
        if (precut_seam.xyz.empty()) {
            return true;  // No seam to append
        }

        // Create seam transition segment
        TrajectorySegment seam_seg;
        seam_seg.kind = "SEAM_TRANSITION";
        seam_seg.xyz = precut_seam.xyz;
        seam_seg.spray_on = true;
        seam_seg.note = "Seam closure";
        seam_seg.region = "SEAM";

        trajectory.push_back(seam_seg);
        return true;
    }

} // namespace SprayTrajPlan
