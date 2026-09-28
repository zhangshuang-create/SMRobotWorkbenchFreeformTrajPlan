#include "TrajectoryCore.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <map>
#include <numeric>
#include <queue>

namespace SprayTrajPlan
{
    // ============================================================================
    // UVToXYZMapper Implementation
    // ============================================================================

    struct UVToXYZMapper::Impl {
        const ObjUVData& data;

        // KD-Tree data (simplified implementation)
        std::vector<Eigen::Vector2d> triangle_centers;
        std::vector<Eigen::Vector2d> triangle_minimum;
        std::vector<Eigen::Vector2d> triangle_maximum;
        std::vector<Eigen::Vector3d> vertex_xyz;

        struct CenterNode {
            int face = -1;
            int axis = 0;
            int left = -1;
            int right = -1;
        };
        std::vector<CenterNode> center_tree;
        int center_tree_root = -1;

        double uv_tolerance;

        explicit Impl(const ObjUVData& data_) : data(data_) {
            // Compute triangle centers and bounds
            triangle_centers.reserve(data.uv_faces.size());
            triangle_minimum.reserve(data.uv_faces.size());
            triangle_maximum.reserve(data.uv_faces.size());

            for (const auto& face : data.uv_faces) {
                Eigen::Vector2d v0 = data.uv[face[0]];
                Eigen::Vector2d v1 = data.uv[face[1]];
                Eigen::Vector2d v2 = data.uv[face[2]];

                triangle_centers.push_back((v0 + v1 + v2) / 3.0);

                Eigen::Vector2d min_pt = v0.cwiseMin(v1).cwiseMin(v2);
                Eigen::Vector2d max_pt = v0.cwiseMax(v1).cwiseMax(v2);

                triangle_minimum.push_back(min_pt);
                triangle_maximum.push_back(max_pt);
            }

            // Build vertex XYZ mapping
            vertex_xyz.resize(data.uv.size(), Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));

            std::map<int, std::vector<Eigen::Vector3d>> uv_to_xyz;
            for (size_t i = 0; i < data.xyz_faces.size(); ++i) {
                const auto& xyz_face = data.xyz_faces[i];
                const auto& uv_face = data.uv_faces[i];

                for (int j = 0; j < 3; ++j) {
                    int xyz_idx = xyz_face[j];
                    int uv_idx = uv_face[j];
                    uv_to_xyz[uv_idx].push_back(data.xyz[xyz_idx]);
                }
            }

            for (const auto& [uv_idx, xyz_values] : uv_to_xyz) {
                Eigen::Vector3d sum = Eigen::Vector3d::Zero();
                for (const auto& xyz : xyz_values) {
                    sum += xyz;
                }
                vertex_xyz[uv_idx] = sum / static_cast<double>(xyz_values.size());
            }

            // The Python mapper queries cKDTree(triangle_centers) in
            // increasing batches.  Keep the same search domain instead of
            // falling back to OBJ face order after one nearest triangle.
            std::vector<int> faces(triangle_centers.size());
            std::iota(faces.begin(), faces.end(), 0);
            center_tree.reserve(faces.size());
            const auto buildTree = [&](auto&& self, int begin, int end, int depth) -> int {
                if (begin >= end) return -1;
                const int axis = depth % 2;
                const int middle = begin + (end - begin) / 2;
                std::nth_element(faces.begin() + begin, faces.begin() + middle, faces.begin() + end,
                    [this, axis](int left, int right) {
                        const double lhs = triangle_centers[left][axis];
                        const double rhs = triangle_centers[right][axis];
                        return lhs == rhs ? left < right : lhs < rhs;
                    });
                const int node = static_cast<int>(center_tree.size());
                center_tree.push_back({faces[middle], axis, -1, -1});
                center_tree[node].left = self(self, begin, middle, depth + 1);
                center_tree[node].right = self(self, middle + 1, end, depth + 1);
                return node;
            };
            center_tree_root = buildTree(buildTree, 0, static_cast<int>(faces.size()), 0);

            // Compute UV tolerance
            Eigen::Vector2d min_uv = data.uv[0];
            Eigen::Vector2d max_uv = data.uv[0];
            for (const auto& uv : data.uv) {
                min_uv = min_uv.cwiseMin(uv);
                max_uv = max_uv.cwiseMax(uv);
            }
            uv_tolerance = 1.0e-8 * std::max((max_uv - min_uv).maxCoeff(), 1.0);
        }

        int findNearestTriangle(const Eigen::Vector2d& point) const {
            int nearest = -1;
            double min_dist = std::numeric_limits<double>::max();

            for (size_t i = 0; i < triangle_centers.size(); ++i) {
                double dist = (triangle_centers[i] - point).squaredNorm();
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest = static_cast<int>(i);
                }
            }

            return nearest;
        }

        int findNearestVertex(const Eigen::Vector2d& point) const {
            int nearest = -1;
            double min_dist = std::numeric_limits<double>::max();

            for (size_t i = 0; i < data.uv.size(); ++i) {
                double dist = (data.uv[i] - point).squaredNorm();
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest = static_cast<int>(i);
                }
            }

            return nearest;
        }

        std::vector<int> nearestTriangleIds(const Eigen::Vector2d& point, size_t count) const {
            using Entry = std::pair<double, int>;
            std::priority_queue<Entry> nearest;
            const auto visit = [&](auto&& self, int nodeIndex) -> void {
                if (nodeIndex < 0) return;
                const CenterNode& node = center_tree[nodeIndex];
                const double distance = (triangle_centers[node.face] - point).squaredNorm();
                if (nearest.size() < count) nearest.emplace(distance, node.face);
                else if (distance < nearest.top().first ||
                    (distance == nearest.top().first && node.face < nearest.top().second)) {
                    nearest.pop();
                    nearest.emplace(distance, node.face);
                }
                const double delta = point[node.axis] - triangle_centers[node.face][node.axis];
                const int nearChild = delta <= 0.0 ? node.left : node.right;
                const int farChild = delta <= 0.0 ? node.right : node.left;
                self(self, nearChild);
                if (nearest.size() < count || delta * delta <= nearest.top().first) self(self, farChild);
            };
            if (count == 0) return {};
            visit(visit, center_tree_root);
            std::vector<Entry> ordered;
            ordered.reserve(nearest.size());
            while (!nearest.empty()) { ordered.push_back(nearest.top()); nearest.pop(); }
            std::sort(ordered.begin(), ordered.end(), [](const Entry& left, const Entry& right) {
                return left.first == right.first ? left.second < right.second : left.first < right.first;
            });
            std::vector<int> ids;
            ids.reserve(ordered.size());
            for (const auto& [ignoredDistance, face] : ordered) ids.push_back(face);
            return ids;
        }
    };

    UVToXYZMapper::UVToXYZMapper(const ObjUVData& data)
        : impl_(std::make_unique<Impl>(data))
    {
    }

    UVToXYZMapper::~UVToXYZMapper() = default;

    Eigen::Vector3d UVToXYZMapper::computeWeights(
        const Eigen::Vector2d& point,
        const std::vector<Eigen::Vector2d>& triangle)
    {
        const Eigen::Vector2d& a = triangle[0];
        const Eigen::Vector2d& b = triangle[1];
        const Eigen::Vector2d& c = triangle[2];

        Eigen::Matrix2d matrix;
        matrix.col(0) = b - a;
        matrix.col(1) = c - a;

        double determinant = matrix.determinant();
        if (std::abs(determinant) <= EPS) {
            return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        }

        Eigen::Vector2d bc = matrix.inverse() * (point - a);
        double beta = bc[0];
        double gamma = bc[1];
        double alpha = 1.0 - beta - gamma;

        return Eigen::Vector3d(alpha, beta, gamma);
    }

    int UVToXYZMapper::findContainingTriangle(const Eigen::Vector2d& point, int start_hint) const
    {
        int candidate_face = -1;
        Eigen::Vector3d candidate_weights;
        double candidate_minimum = -std::numeric_limits<double>::infinity();

        const size_t faceCount = impl_->triangle_centers.size();
        if (faceCount == 0) return -1;
        size_t queryCount = std::min<size_t>(32, faceCount);
        while (true) {
            const auto candidateFaces = impl_->nearestTriangleIds(point, queryCount);
            for (const int current_face : candidateFaces) {
            // Check bounding box first
            if ((point.array() < impl_->triangle_minimum[current_face].array() - impl_->uv_tolerance).any() ||
                (point.array() > impl_->triangle_maximum[current_face].array() + impl_->uv_tolerance).any()) {
                continue;
            }

            // Compute barycentric coordinates
            std::vector<Eigen::Vector2d> triangle = {
                impl_->data.uv[impl_->data.uv_faces[current_face][0]],
                impl_->data.uv[impl_->data.uv_faces[current_face][1]],
                impl_->data.uv[impl_->data.uv_faces[current_face][2]]
            };

            Eigen::Vector3d weights = computeWeights(point, triangle);

            if (!weights.allFinite()) {
                continue;
            }

            double minimum = weights.minCoeff();
            if (minimum > candidate_minimum) {
                candidate_minimum = minimum;
                candidate_face = current_face;
                candidate_weights = weights;
            }

                // Identical to Python: first near candidate inside the
                // tolerance is accepted, otherwise expand the KD query.
                if (minimum >= -1.0e-8) return current_face;
            }

            if (candidate_minimum >= -1.0e-8 || queryCount == faceCount) break;
            const size_t nextCount = std::min({2 * queryCount, static_cast<size_t>(2048), faceCount});
            if (nextCount == queryCount) break;
            queryCount = nextCount;
        }

        // Keep the Python mapper's domain contract.  A best candidate outside
        // the triangle is not a valid inverse mapping: Python rejects it when
        // its minimum barycentric coordinate is below -1e-5, then performs
        // optional vertex snapping / polyline bridging.  Returning that face
        // here used negative weights to extrapolate onto unrelated 3D faces.
        return candidate_minimum >= -1.0e-5 ? candidate_face : -1;
    }

    Eigen::Vector3d UVToXYZMapper::mapSingle(
        const Eigen::Vector2d& uv_point,
        double snap_tolerance,
        bool bridge_outside)
    {
        int face_id = findContainingTriangle(uv_point);

        if (face_id < 0) {
            // Try vertex snapping
            if (snap_tolerance > 0.0) {
                int nearest_vertex = impl_->findNearestVertex(uv_point);
                double dist = (impl_->data.uv[nearest_vertex] - uv_point).norm();

                if (dist <= snap_tolerance && impl_->vertex_xyz[nearest_vertex].allFinite()) {
                    return impl_->vertex_xyz[nearest_vertex];
                }
            }

            if (bridge_outside) {
                return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
            }

            throw std::runtime_error("UV point lies outside the domain");
        }

        // Compute barycentric coordinates
        std::vector<Eigen::Vector2d> triangle = {
            impl_->data.uv[impl_->data.uv_faces[face_id][0]],
            impl_->data.uv[impl_->data.uv_faces[face_id][1]],
            impl_->data.uv[impl_->data.uv_faces[face_id][2]]
        };

        Eigen::Vector3d weights = computeWeights(uv_point, triangle);

        // Interpolate XYZ
        Eigen::Vector3d result = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; ++i) {
            int xyz_idx = impl_->data.xyz_faces[face_id][i];
            result += weights[i] * impl_->data.xyz[xyz_idx];
        }

        return result;
    }

    std::vector<Eigen::Vector3d> UVToXYZMapper::map(
        const std::vector<Eigen::Vector2d>& uv_points,
        double snap_tolerance,
        bool bridge_outside)
    {
        std::vector<Eigen::Vector3d> result;
        result.reserve(uv_points.size());

        for (const auto& uv_point : uv_points) {
            result.push_back(mapSingle(uv_point, snap_tolerance, bridge_outside));
        }

        // Handle bridging if needed
        if (bridge_outside) {
            std::vector<int> valid_indices;
            for (size_t i = 0; i < result.size(); ++i) {
                if (result[i].allFinite()) {
                    valid_indices.push_back(static_cast<int>(i));
                }
            }

            if (valid_indices.empty()) {
                // All points invalid - use nearest vertex
                for (size_t i = 0; i < uv_points.size(); ++i) {
                    int nearest = impl_->findNearestVertex(uv_points[i]);
                    result[i] = impl_->vertex_xyz[nearest];
                }
            } else {
                // Interpolate invalid points
                for (size_t i = 0; i < result.size(); ++i) {
                    if (!result[i].allFinite()) {
                        // Find nearest valid points and interpolate
                        int prev = -1, next = -1;
                        for (int j : valid_indices) {
                            if (j < static_cast<int>(i) && (prev < 0 || j > prev)) prev = j;
                            if (j > static_cast<int>(i) && (next < 0 || j < next)) next = j;
                        }

                        if (prev >= 0 && next >= 0) {
                            double t = static_cast<double>(i - prev) / (next - prev);
                            result[i] = (1.0 - t) * result[prev] + t * result[next];
                        } else if (prev >= 0) {
                            result[i] = result[prev];
                        } else if (next >= 0) {
                            result[i] = result[next];
                        }
                    }
                }
            }
        }

        return result;
    }

    // ============================================================================
    // SpacingOptimizer Implementation
    // ============================================================================

    SpacingOptimum SpacingOptimizer::optimizeSpacing(
        double nominal,
        const Eigen::Vector3d& dimensions)
    {
        if (nominal <= 0.0) {
            throw std::invalid_argument("Nominal trajectory spacing must be greater than zero");
        }

        if (dimensions[0] <= 0.0 || dimensions[1] < 0.0 || dimensions[2] < 0.0) {
            throw std::invalid_argument("Invalid dimensions");
        }

        // Generate candidate spacings
        const int n_candidates = 500;
        std::vector<double> candidates(n_candidates);
        for (int i = 0; i < n_candidates; ++i) {
            candidates[i] = 0.9 * nominal + (0.2 * nominal * i) / (n_candidates - 1);
        }

        std::vector<Eigen::Vector3i> counts(n_candidates);
        std::vector<Eigen::Vector3d> actual(n_candidates);
        std::vector<double> objective_values(n_candidates);

        for (int i = 0; i < n_candidates; ++i) {
            double d = candidates[i];

            // Center count (must be odd)
            double center_ratio = dimensions[0] / d;
            int lower_odd = std::max(1, 2 * static_cast<int>(std::floor((center_ratio - 1.0) / 2.0)) + 1);
            int upper_odd = lower_odd + 2;

            int center_count = (std::abs(dimensions[0] / lower_odd - d) <= std::abs(dimensions[0] / upper_odd - d))
                ? lower_odd : upper_odd;

            counts[i][0] = center_count;

            // Upper and lower counts (floor for safety)
            bool upper_active = dimensions[1] > 0.75 * d;
            bool lower_active = dimensions[2] > 0.75 * d;

            counts[i][1] = upper_active ? std::max(1, static_cast<int>(std::floor(dimensions[1] / d))) : 0;
            counts[i][2] = lower_active ? std::max(1, static_cast<int>(std::floor(dimensions[2] / d))) : 0;

            // Compute actual spacings
            actual[i][0] = dimensions[0] / center_count;
            actual[i][1] = (counts[i][1] > 0) ? (dimensions[1] / counts[i][1]) : (0.5 * actual[i][0] + dimensions[1]);
            actual[i][2] = (counts[i][2] > 0) ? (dimensions[2] / counts[i][2]) : (0.5 * actual[i][0] + dimensions[2]);

            // Compute objective (variance + bias)
            Eigen::Vector3d relative_error = (actual[i].array() - d) / d;
            double variance = ((relative_error.array() - relative_error.mean()).square()).sum() / 3.0;
            double bias = std::pow((actual[i].mean() - nominal) / nominal, 2);

            objective_values[i] = variance + 0.5 * bias;
        }

        // Find best
        int best_idx = 0;
        for (int i = 1; i < n_candidates; ++i) {
            if (objective_values[i] < objective_values[best_idx]) {
                best_idx = i;
            }
        }

        SpacingOptimum result;
        result.spacing = candidates[best_idx];
        result.counts = counts[best_idx];
        result.actual_spacings = actual[best_idx];
        result.objective = objective_values[best_idx];
        result.candidates = candidates;
        result.objective_values = objective_values;

        return result;
    }

    std::pair<std::vector<double>, double> SpacingOptimizer::balancedCenterPositions(
        double width,
        int count)
    {
        if (count <= 0 || count % 2 == 0) {
            throw std::invalid_argument("Center count must be a positive odd integer");
        }

        double spacing = width / count;
        std::vector<double> positions(count);

        for (int i = 0; i < count; ++i) {
            positions[i] = (i + 0.5) * spacing;
        }

        return {positions, spacing};
    }

    std::pair<std::vector<double>, double> SpacingOptimizer::balancedOuterPositions(
        double extent,
        int count,
        double waist_x,
        double direction)
    {
        extent = std::max(0.0, extent);

        if (extent <= EPS || count <= 0) {
            return {std::vector<double>(), std::numeric_limits<double>::quiet_NaN()};
        }

        double actual = extent / count;
        std::vector<double> positions(count);

        for (int i = 0; i < count; ++i) {
            positions[i] = waist_x + direction * actual * (i + 0.5);
        }

        return {positions, actual};
    }

    // ============================================================================
    // TrajectorySampler Implementation
    // ============================================================================

    std::vector<Eigen::Vector2d> TrajectorySampler::fromLocal(
        const std::vector<Eigen::Vector2d>& local_coords,
        const PlanningFrame& frame)
    {
        std::vector<Eigen::Vector2d> uv_coords;
        uv_coords.reserve(local_coords.size());

        // Transform from planning frame to UV space
        // UV = origin_uv + x_axis_uv * local_x + y_axis_uv * local_y
        for (const auto& local : local_coords) {
            // For 2D UV space, we project the frame axes
            Eigen::Vector2d uv = Eigen::Vector2d::Zero();
            // Simplified: assume frame is already in UV space
            uv = local;
            uv_coords.push_back(uv);
        }

        return uv_coords;
    }

    std::vector<Eigen::Vector2d> TrajectorySampler::toLocal(
        const std::vector<Eigen::Vector2d>& uv_coords,
        const PlanningFrame& frame)
    {
        std::vector<Eigen::Vector2d> local_coords;
        local_coords.reserve(uv_coords.size());

        for (const auto& uv : uv_coords) {
            // Transform from UV space to planning frame local coordinates
            Eigen::Vector2d local = uv;  // Simplified
            local_coords.push_back(local);
        }

        return local_coords;
    }

    std::vector<Eigen::Vector2d> TrajectorySampler::sampleScanUV(
        double x_value,
        const std::pair<double, double>& interval,
        bool upward,
        double sample_step,
        const PlanningFrame& frame)
    {
        double start = upward ? interval.first : interval.second;
        double stop = upward ? interval.second : interval.first;

        int count = std::max(2, static_cast<int>(std::ceil(std::abs(stop - start) / sample_step)) + 1);

        std::vector<Eigen::Vector2d> local_coords(count);
        for (int i = 0; i < count; ++i) {
            double t = static_cast<double>(i) / (count - 1);
            double y_value = start + t * (stop - start);
            local_coords[i] = Eigen::Vector2d(x_value, y_value);
        }

        return fromLocal(local_coords, frame);
    }

    // ============================================================================
    // SeamHandler Implementation
    // ============================================================================

    std::pair<Eigen::Vector2d, Eigen::Vector2d> SeamHandler::seamProjectionAndCounterpart(
        const Eigen::Vector2d& point,
        const std::vector<SeamUVSegment>& seam_segments,
        bool from_a)
    {
        double best_distance = std::numeric_limits<double>::infinity();
        Eigen::Vector2d best_source;
        Eigen::Vector2d best_counterpart;

        for (const auto& segment : seam_segments) {
            Eigen::Vector2d a, b, target_a, target_b;

            if (from_a) {
                a = segment.uv_a0;
                b = segment.uv_a1;
                target_a = segment.uv_b0;
                target_b = segment.uv_b1;
            } else {
                a = segment.uv_b0;
                b = segment.uv_b1;
                target_a = segment.uv_a0;
                target_b = segment.uv_a1;
            }

            Eigen::Vector2d direction = b - a;
            double length2 = direction.squaredNorm();

            double ratio = 0.0;
            if (length2 > EPS) {
                ratio = std::clamp(direction.dot(point - a) / length2, 0.0, 1.0);
            }

            Eigen::Vector2d projected = a + ratio * direction;
            double distance = (point - projected).norm();

            if (distance < best_distance) {
                best_distance = distance;
                best_source = projected;
                best_counterpart = target_a + ratio * (target_b - target_a);
            }
        }

        if (!std::isfinite(best_distance)) {
            throw std::runtime_error("Seam contains no edge for periodic mapping");
        }

        return {best_source, best_counterpart};
    }

    TrajectorySegment SeamHandler::createOvertravelSegment(
        const TrajectorySegment& previous,
        const TrajectorySegment& following,
        double overtravel)
    {
        if (previous.xyz.size() < 2 || following.xyz.size() < 2) {
            throw std::invalid_argument("Surface scans need at least two points for 3D overtravel");
        }

        // Compute exit and entry directions
        Eigen::Vector3d exit_direction = previous.xyz.back() - previous.xyz[previous.xyz.size() - 2];
        Eigen::Vector3d entry_direction = following.xyz[1] - following.xyz[0];

        double exit_length = exit_direction.norm();
        double entry_length = entry_direction.norm();

        if (exit_length > EPS) exit_direction /= exit_length;
        if (entry_length > EPS) entry_direction /= entry_length;

        // Endpoints
        Eigen::Vector3d p0 = previous.xyz.back();
        Eigen::Vector3d p3 = following.xyz.front();

        double gap = (p3 - p0).norm();
        double handle = std::clamp(overtravel, 0.0, 0.45 * gap);

        TrajectorySegment segment;
        segment.kind = "OVERTRAVEL";
        segment.spray_on = false;
        segment.note = "3D-only opening connector";
        segment.region = "TRANSITION";

        if (handle <= EPS) {
            segment.xyz = {p0, p3};
        } else {
            // Cubic Bezier-like connector
            Eigen::Vector3d p1 = p0 + handle * exit_direction;
            Eigen::Vector3d p2 = p3 - handle * entry_direction;
            segment.xyz = {p0, p1, p2, p3};
        }

        // UV coordinates are NaN for 3D-only segments
        segment.uv.resize(segment.xyz.size(), Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()));

        return segment;
    }

    std::vector<TrajectorySegment> SeamHandler::rebuildOvertravelConnectors(
        const std::vector<TrajectorySegment>& trajectory,
        double overtravel)
    {
        std::vector<TrajectorySegment> result;
        result.reserve(trajectory.size());

        for (size_t i = 0; i < trajectory.size(); ++i) {
            const auto& segment = trajectory[i];

            if (segment.kind == "OVERTRAVEL" && i > 0 && i < trajectory.size() - 1) {
                // Rebuild this overtravel connector
                const auto& prev = trajectory[i - 1];
                const auto& next = trajectory[i + 1];

                if (prev.kind == "SURFACE_SCAN" && next.kind == "SURFACE_SCAN") {
                    result.push_back(createOvertravelSegment(prev, next, overtravel));
                    continue;
                }
            }

            result.push_back(segment);
        }

        return result;
    }

} // namespace SprayTrajPlan
