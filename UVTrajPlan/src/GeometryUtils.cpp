#include "GeometryUtils.h"
#include "GpuMeshProjection.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

namespace SprayTrajPlan
{
    namespace
    {
        Eigen::Vector3d closestPointOnTriangle(const Eigen::Vector3d& point,
            const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
        {
            const Eigen::Vector3d ab = b - a, ac = c - a, ap = point - a;
            const double d1 = ab.dot(ap), d2 = ac.dot(ap);
            if (d1 <= 0.0 && d2 <= 0.0) return a;
            const Eigen::Vector3d bp = point - b;
            const double d3 = ab.dot(bp), d4 = ac.dot(bp);
            if (d3 >= 0.0 && d4 <= d3) return b;
            const double vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) return a + (d1 / (d1 - d3)) * ab;
            const Eigen::Vector3d cp = point - c;
            const double d5 = ab.dot(cp), d6 = ac.dot(cp);
            if (d6 >= 0.0 && d5 <= d6) return c;
            const double vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) return a + (d2 / (d2 - d6)) * ac;
            const double va = d3 * d6 - d5 * d4;
            if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
                return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
            }
            const double inverse = 1.0 / (va + vb + vc);
            return a + ab * (vb * inverse) + ac * (vc * inverse);
        }

        // A cached exact triangle BVH replaces the former O(query_count *
        // face_count) projection loop.  It is also the CPU-side acceleration
        // structure required by the CUDA backend for large meshes.
        struct BvhNode {
            Eigen::Vector3d lower = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
            Eigen::Vector3d upper = Eigen::Vector3d::Constant(-std::numeric_limits<double>::max());
            int left = -1, right = -1, first = 0, count = 0;
            bool leaf() const { return left < 0; }
        };

        struct TriangleBvh {
            const std::vector<Eigen::Vector3d>* vertices = nullptr;
            const std::vector<Eigen::Vector3i>* faces = nullptr;
            std::vector<int> faceIds;
            std::vector<BvhNode> nodes;

            explicit TriangleBvh(const std::vector<Eigen::Vector3d>& v, const std::vector<Eigen::Vector3i>& f)
                : vertices(&v), faces(&f) {
                faceIds.reserve(f.size());
                for (size_t i = 0; i < f.size(); ++i)
                    if (f[i].minCoeff() >= 0 && f[i].maxCoeff() < static_cast<int>(v.size())) faceIds.push_back(static_cast<int>(i));
                if (!faceIds.empty()) build(0, static_cast<int>(faceIds.size()));
            }

            Eigen::Vector3d centroid(int faceId) const {
                const auto& face = (*faces)[faceId];
                return ((*vertices)[face[0]] + (*vertices)[face[1]] + (*vertices)[face[2]]) / 3.0;
            }
            void bounds(BvhNode& node, int first, int count) const {
                for (int i = first; i < first + count; ++i) {
                    const auto& face = (*faces)[faceIds[i]];
                    for (int k = 0; k < 3; ++k) {
                        node.lower = node.lower.cwiseMin((*vertices)[face[k]]);
                        node.upper = node.upper.cwiseMax((*vertices)[face[k]]);
                    }
                }
            }
            int build(int first, int count) {
                const int nodeId = static_cast<int>(nodes.size()); nodes.emplace_back();
                bounds(nodes[nodeId], first, count);
                if (count <= 8) { nodes[nodeId].first = first; nodes[nodeId].count = count; return nodeId; }
                Eigen::Vector3d extent = nodes[nodeId].upper - nodes[nodeId].lower;
                int axis = 0; if (extent.y() > extent.x()) axis = 1; if (extent.z() > extent[axis]) axis = 2;
                const int middle = first + count / 2;
                std::nth_element(faceIds.begin() + first, faceIds.begin() + middle, faceIds.begin() + first + count,
                    [&](int a, int b) { return centroid(a)[axis] < centroid(b)[axis]; });
                const int left = build(first, middle - first);
                const int right = build(middle, first + count - middle);
                nodes[nodeId].left = left; nodes[nodeId].right = right;
                return nodeId;
            }
            static double boxDistanceSquared(const Eigen::Vector3d& point, const BvhNode& node) {
                double distance = 0.0;
                for (int axis = 0; axis < 3; ++axis) {
                    const double delta = point[axis] < node.lower[axis] ? node.lower[axis] - point[axis] :
                        (point[axis] > node.upper[axis] ? point[axis] - node.upper[axis] : 0.0);
                    distance += delta * delta;
                }
                return distance;
            }
            void query(const Eigen::Vector3d& point, Eigen::Vector3d& closest, double& distance, int& faceId) const {
                double best = std::numeric_limits<double>::max(); faceId = -1; closest = Eigen::Vector3d::Zero();
                if (nodes.empty()) { distance = std::numeric_limits<double>::max(); return; }
                std::vector<int> stack{0};
                while (!stack.empty()) {
                    const int nodeId = stack.back(); stack.pop_back(); const auto& node = nodes[nodeId];
                    if (boxDistanceSquared(point, node) > best) continue;
                    if (node.leaf()) {
                        for (int i = node.first; i < node.first + node.count; ++i) {
                            const int currentFace = faceIds[i]; const auto& face = (*faces)[currentFace];
                            const Eigen::Vector3d candidate = closestPointOnTriangle(point, (*vertices)[face[0]], (*vertices)[face[1]], (*vertices)[face[2]]);
                            const double candidateDistance = (point - candidate).squaredNorm();
                            if (candidateDistance < best) { best = candidateDistance; closest = candidate; faceId = currentFace; }
                        }
                    } else {
                        const double leftDistance = boxDistanceSquared(point, nodes[node.left]);
                        const double rightDistance = boxDistanceSquared(point, nodes[node.right]);
                        if (leftDistance < rightDistance) { stack.push_back(node.right); stack.push_back(node.left); }
                        else { stack.push_back(node.left); stack.push_back(node.right); }
                    }
                }
                distance = std::sqrt(best);
            }
        };

        std::mutex bvhMutex;
        std::shared_ptr<TriangleBvh> cachedBvh;
        const std::vector<Eigen::Vector3d>* cachedVertices = nullptr;
        const std::vector<Eigen::Vector3i>* cachedFaces = nullptr;
        size_t cachedVertexCount = 0;
        size_t cachedFaceCount = 0;
        std::atomic<std::uint64_t> cpuBatches{0}, gpuBatches{0}, cpuQueries{0}, gpuQueries{0};
        std::atomic<std::uint64_t> cpuNanoseconds{0}, gpuNanoseconds{0};

        std::shared_ptr<TriangleBvh> projectionBvh(const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<Eigen::Vector3i>& faces) {
            std::lock_guard<std::mutex> guard(bvhMutex);
            if (!cachedBvh || cachedVertices != &vertices || cachedFaces != &faces ||
                cachedVertexCount != vertices.size() || cachedFaceCount != faces.size()) {
                cachedBvh = std::make_shared<TriangleBvh>(vertices, faces);
                cachedVertices = &vertices; cachedFaces = &faces;
                cachedVertexCount = vertices.size(); cachedFaceCount = faces.size();
            }
            return cachedBvh;
        }
    }

    void GeometryUtils::resetProjectionStatistics()
    {
        cpuBatches = gpuBatches = cpuQueries = gpuQueries = 0;
        cpuNanoseconds = gpuNanoseconds = 0;
    }

    GeometryUtils::ProjectionStatistics GeometryUtils::projectionStatistics()
    {
        ProjectionStatistics result;
        result.cpu_batches = cpuBatches.load(); result.gpu_batches = gpuBatches.load();
        result.cpu_queries = cpuQueries.load(); result.gpu_queries = gpuQueries.load();
        result.cpu_milliseconds = static_cast<double>(cpuNanoseconds.load()) / 1.0e6;
        result.gpu_milliseconds = static_cast<double>(gpuNanoseconds.load()) / 1.0e6;
        return result;
    }
    // Segment distance implementation
    double GeometryUtils::segmentDistance3D(
        const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
        const Eigen::Vector3d& q0, const Eigen::Vector3d& q1)
    {
        Eigen::Vector3d u = p1 - p0;
        Eigen::Vector3d v = q1 - q0;
        Eigen::Vector3d w = p0 - q0;

        double a = u.dot(u);
        double b = u.dot(v);
        double c = v.dot(v);
        double d = u.dot(w);
        double e = v.dot(w);

        double denominator = a * c - b * b;
        double s_num = denominator;
        double s_den = denominator;
        double t_num = denominator;
        double t_den = denominator;

        if (denominator <= EPS) {
            s_num = 0.0;
            s_den = 1.0;
            t_num = e;
            t_den = c;
        } else {
            s_num = b * e - c * d;
            t_num = a * e - b * d;

            if (s_num < 0.0) {
                s_num = 0.0;
                t_num = e;
                t_den = c;
            } else if (s_num > s_den) {
                s_num = s_den;
                t_num = e + b;
                t_den = c;
            }
        }

        if (t_num < 0.0) {
            t_num = 0.0;
            if (-d < 0.0) {
                s_num = 0.0;
            } else if (-d > a) {
                s_num = s_den;
            } else {
                s_num = -d;
                s_den = a;
            }
        } else if (t_num > t_den) {
            t_num = t_den;
            if (-d + b < 0.0) {
                s_num = 0.0;
            } else if (-d + b > a) {
                s_num = s_den;
            } else {
                s_num = -d + b;
                s_den = a;
            }
        }

        double s = (std::abs(s_num) <= EPS) ? 0.0 : s_num / std::max(s_den, EPS);
        double t = (std::abs(t_num) <= EPS) ? 0.0 : t_num / std::max(t_den, EPS);

        Eigen::Vector3d closest = w + s * u - t * v;
        return closest.norm();
    }

    double GeometryUtils::lineToPointsDistance(
        const std::pair<Eigen::Vector3d, Eigen::Vector3d>& line,
        const std::vector<Eigen::Vector3d>& points)
    {
        const int samples = 17;
        std::vector<Eigen::Vector3d> line_samples(samples);

        for (int i = 0; i < samples; ++i) {
            double t = static_cast<double>(i) / (samples - 1);
            line_samples[i] = (1.0 - t) * line.first + t * line.second;
        }

        double min_dist = std::numeric_limits<double>::max();
        for (const auto& sample : line_samples) {
            for (const auto& point : points) {
                double dist = (sample - point).norm();
                min_dist = std::min(min_dist, dist);
            }
        }

        return min_dist;
    }

    int GeometryUtils::findNearestPoint(
        const std::vector<Eigen::Vector3d>& points,
        const Eigen::Vector3d& query)
    {
        if (points.empty()) return -1;

        int nearest_idx = 0;
        double min_dist = (points[0] - query).squaredNorm();

        for (size_t i = 1; i < points.size(); ++i) {
            double dist = (points[i] - query).squaredNorm();
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = static_cast<int>(i);
            }
        }

        return nearest_idx;
    }

    std::vector<int> GeometryUtils::findNearestPoints(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<Eigen::Vector3d>& queries)
    {
        std::vector<int> indices(queries.size());
        for (size_t i = 0; i < queries.size(); ++i) {
            indices[i] = findNearestPoint(points, queries[i]);
        }
        return indices;
    }

    std::vector<Eigen::Vector3d> GeometryUtils::trackSlice(
        const std::vector<Eigen::Vector3d>& points,
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end)
    {
        if (points.empty()) {
            return {start, end};
        }

        int start_idx = findNearestPoint(points, start);
        int end_idx = findNearestPoint(points, end);

        std::vector<Eigen::Vector3d> result;
        result.push_back(start);

        if (start_idx <= end_idx) {
            for (int i = start_idx; i <= end_idx; ++i) {
                result.push_back(points[i]);
            }
        } else {
            if (end_idx == 0) {
                for (int i = start_idx; i >= 0; --i) {
                    result.push_back(points[i]);
                }
            } else {
                for (int i = start_idx; i >= end_idx; --i) {
                    result.push_back(points[i]);
                }
            }
        }

        result.push_back(end);
        return result;
    }

    std::vector<bool> GeometryUtils::pointsInPolygon(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<Eigen::Vector2d>& polygon)
    {
        std::vector<bool> inside(points.size(), false);

        if (polygon.size() < 3) return inside;

        for (size_t pt_idx = 0; pt_idx < points.size(); ++pt_idx) {
            double x = points[pt_idx].x();
            double y = points[pt_idx].y();

            int previous = static_cast<int>(polygon.size()) - 1;
            for (size_t current = 0; current < polygon.size(); ++current) {
                double px_cur = polygon[current].x();
                double py_cur = polygon[current].y();
                double px_prev = polygon[previous].x();
                double py_prev = polygon[previous].y();

                bool crosses = ((py_cur > y) != (py_prev > y)) &&
                    (x < (px_prev - px_cur) * (y - py_cur) /
                     (py_prev - py_cur + EPS) + px_cur);

                if (crosses) {
                    inside[pt_idx] = !inside[pt_idx];
                }
                previous = static_cast<int>(current);
            }
        }

        return inside;
    }

    Eigen::Vector3d GeometryUtils::closedPolylineCenter(
        const std::vector<Eigen::Vector3d>& points)
    {
        if (points.empty()) {
            return Eigen::Vector3d::Zero();
        }

        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (const auto& pt : points) {
            sum += pt;
        }
        return sum / static_cast<double>(points.size());
    }

    std::optional<Eigen::Vector3d> GeometryUtils::projectToMesh(
        const std::vector<Eigen::Vector3d>& vertices,
        const std::vector<Eigen::Vector3i>& faces,
        const Eigen::Vector3d& point)
    {
        if (vertices.empty() || faces.empty()) {
            return std::nullopt;
        }

        if (!point.allFinite()) {
            return std::nullopt;
        }

        std::vector<Eigen::Vector3d> closest;
        std::vector<double> distances;
        std::vector<int> faceIndices;
        closestPointsRobust(vertices, faces, {point}, closest, distances, faceIndices);
        return closest.empty() ? std::nullopt : std::optional<Eigen::Vector3d>(closest.front());
    }

    void GeometryUtils::closestPointsRobust(
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const std::vector<Eigen::Vector3d>& query_points,
        std::vector<Eigen::Vector3d>& closest_points,
        std::vector<double>& distances,
        std::vector<int>& face_indices)
    {
        closest_points.resize(query_points.size());
        distances.resize(query_points.size());
        face_indices.resize(query_points.size());
        if (query_points.empty()) return;
#ifdef SPRAYTRAJPLAN_HAS_CUDA
        // CUDA is the default projection backend.  The CPU BVH is used only
        // when the CUDA backend reports an allocation, launch, or copy failure.
        const auto gpuStart = std::chrono::steady_clock::now();
        if (projectClosestPointsCuda(mesh_vertices, mesh_faces, query_points,
                closest_points, distances, face_indices)) {
            gpuBatches.fetch_add(1); gpuQueries.fetch_add(query_points.size());
            gpuNanoseconds.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - gpuStart).count());
            return;
        }
#endif
        const auto cpuStart = std::chrono::steady_clock::now();
        const auto bvh = projectionBvh(mesh_vertices, mesh_faces);
        const auto process = [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                Eigen::Vector3d bestPoint; double bestDistance; int bestFace;
                bvh->query(query_points[i], bestPoint, bestDistance, bestFace);
                if (bestFace < 0) {
                    const int nearest = findNearestPoint(mesh_vertices, query_points[i]);
                    bestPoint = nearest >= 0 ? mesh_vertices[nearest] : query_points[i];
                    bestDistance = (query_points[i] - bestPoint).norm();
                }
                closest_points[i] = bestPoint;
                distances[i] = bestDistance;
                face_indices[i] = bestFace;
            }
        };
        // Queries are independent.  Keep small batches serial to avoid worker
        // startup overhead, while million-point meshes use all CPU cores.
        const unsigned int workers = std::min<unsigned int>(std::thread::hardware_concurrency(),
            static_cast<unsigned int>(query_points.size() / 128));
        if (workers < 2) {
            process(0, query_points.size());
            cpuBatches.fetch_add(1); cpuQueries.fetch_add(query_points.size());
            cpuNanoseconds.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - cpuStart).count());
            return;
        }
        std::vector<std::thread> threads; threads.reserve(workers);
        const size_t chunk = (query_points.size() + workers - 1) / workers;
        for (unsigned int worker = 0; worker < workers; ++worker) {
            const size_t begin = worker * chunk;
            const size_t end = std::min(query_points.size(), begin + chunk);
            if (begin < end) threads.emplace_back(process, begin, end);
        }
        for (auto& thread : threads) thread.join();
        cpuBatches.fetch_add(1); cpuQueries.fetch_add(query_points.size());
        cpuNanoseconds.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - cpuStart).count());
    }

    std::vector<bool> GeometryUtils::localConnectedFaceMask(
        const std::vector<Eigen::Vector3d>& triangle_centers,
        const std::vector<Eigen::Vector3i>& faces,
        const std::vector<std::pair<int, int>>& face_adjacency,
        const std::vector<bool>& candidate_mask,
        const std::vector<Eigen::Vector3d>& boundary_points)
    {
        std::vector<int> candidate_ids;
        for (size_t i = 0; i < candidate_mask.size(); ++i) {
            if (candidate_mask[i]) {
                candidate_ids.push_back(static_cast<int>(i));
            }
        }

        if (candidate_ids.empty()) {
            return candidate_mask;
        }

        // Find seed face closest to boundary
        int seed = candidate_ids[0];
        double min_dist = std::numeric_limits<double>::max();

        for (int id : candidate_ids) {
            for (const auto& boundary_pt : boundary_points) {
                double dist = (triangle_centers[id] - boundary_pt).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    seed = id;
                }
            }
        }

        // Build adjacency graph
        std::map<int, std::vector<int>> graph;
        for (const auto& edge : face_adjacency) {
            if (candidate_mask[edge.first] && candidate_mask[edge.second]) {
                graph[edge.first].push_back(edge.second);
                graph[edge.second].push_back(edge.first);
            }
        }

        // Flood fill from seed
        std::vector<bool> component(candidate_mask.size(), false);
        component[seed] = true;
        std::vector<int> stack = {seed};

        while (!stack.empty()) {
            int face = stack.back();
            stack.pop_back();

            if (graph.find(face) != graph.end()) {
                for (int neighbour : graph[face]) {
                    if (!component[neighbour]) {
                        component[neighbour] = true;
                        stack.push_back(neighbour);
                    }
                }
            }
        }

        return component;
    }

    GeometryUtils::BarrierState GeometryUtils::radialBarrierState(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<RadialBarrier>& barriers,
        double spacing,
        double stiffness)
    {
        BarrierState state;
        size_t count = points.size();

        state.force.resize(count, Eigen::Vector3d::Zero());
        state.direction.resize(count, Eigen::Vector3d::Zero());
        state.dangerous.resize(count, false);
        state.inside_any.resize(count, false);
        state.escape_step.resize(count, 0.0);

        std::vector<Eigen::Vector3d> magnitude_vec(count, Eigen::Vector3d::Zero());
        std::vector<double> magnitude(count, 0.0);

        double d = std::max(spacing, EPS);
        double inner_limit = 0.5 * d;
        double outer_limit = 0.9 * d;

        for (const auto& barrier : barriers) {
            const Eigen::Vector3d& center = barrier.center;
            const auto& boundary_points = barrier.boundary_points;

            for (size_t i = 0; i < count; ++i) {
                // Find nearest boundary point
                int nearest_idx = findNearestPoint(boundary_points, points[i]);
                if (nearest_idx < 0) continue;

                const Eigen::Vector3d& nearest = boundary_points[nearest_idx];
                double distance = (points[i] - nearest).norm();

                Eigen::Vector3d ray = points[i] - center;
                double radius = ray.norm();

                Eigen::Vector3d unit = Eigen::Vector3d::Zero();
                if (radius > 1.0e-8) {
                    unit = ray / radius;
                } else {
                    Eigen::Vector3d boundary_ray = nearest - center;
                    double boundary_radius = boundary_ray.norm();
                    if (boundary_radius > 1.0e-8) {
                        unit = boundary_ray / boundary_radius;
                    }
                }

                double boundary_radius = (nearest - center).norm();
                bool inside = radius < boundary_radius - 1.0e-6 * d;
                bool ultra_close = (!inside) && (distance < inner_limit);
                bool transition = (!inside) && (!ultra_close) && (distance < outer_limit);

                double local_magnitude = 0.0;
                if (inside) {
                    double depth = std::max(boundary_radius - radius, 0.0) / d;
                    local_magnitude = stiffness * (20.0 + 20.0 * depth + 10.0 * depth * depth);
                } else if (ultra_close) {
                    double close_ratio = std::clamp((inner_limit - distance) / inner_limit, 0.0, 1.0);
                    local_magnitude = stiffness * (3.0 + 17.0 * close_ratio * close_ratio);
                } else if (transition) {
                    double transition_t = std::clamp(
                        (outer_limit - distance) / std::max(outer_limit - inner_limit, EPS),
                        0.0, 1.0
                    );
                    double transition_smooth = transition_t * transition_t * (3.0 - 2.0 * transition_t);
                    local_magnitude = stiffness * 3.0 * transition_smooth;
                }

                if (local_magnitude > magnitude[i]) {
                    magnitude[i] = local_magnitude;
                    state.direction[i] = unit;
                }

                if (inside || ultra_close) {
                    state.dangerous[i] = true;
                }
                if (inside) {
                    state.inside_any[i] = true;
                }

                double needed = inside ?
                    std::max(boundary_radius - radius, 0.0) + 0.85 * d :
                    std::max(0.85 * d - distance, 0.0);

                double local_escape = (inside || ultra_close) ?
                    std::clamp(needed, 0.6 * d, 0.8 * d) : 0.0;

                state.escape_step[i] = std::max(state.escape_step[i], local_escape);
            }
        }

        for (size_t i = 0; i < count; ++i) {
            state.force[i] = state.direction[i] * magnitude[i];
        }

        return state;
    }

    // Special boundary geometry - simplified version
    GeometryUtils::BoundaryGeometry GeometryUtils::specialBoundaryGeometry(
        const std::vector<TrajectorySegment>& trajectory,
        double spacing,
        const std::vector<Eigen::Vector3d>* mesh_vertices,
        const std::vector<Eigen::Vector3i>* mesh_faces)
    {
        BoundaryGeometry result;
        result.periodic_pair_blocked = false;
        result.target_distance = 1.5 * spacing;

        // Extract CENTER tracks
        struct Track {
            std::string region;
            int segment_index;
            int phase;
            std::vector<Eigen::Vector3d> xyz;
            std::vector<Eigen::Vector2d> uv;
        };

        std::vector<Track> tracks;
        for (size_t seg_idx = 0; seg_idx < trajectory.size(); ++seg_idx) {
            const auto& seg = trajectory[seg_idx];
            if (seg.kind != "SURFACE_SCAN") continue;

            std::string region = seg.region;
            std::transform(region.begin(), region.end(), region.begin(), ::toupper);

            if (region != "CENTER" && region != "LOWER" && region != "UPPER") continue;
            if (seg.xyz.size() < 2) continue;

            Track track;
            track.region = region;
            track.segment_index = static_cast<int>(seg_idx);
            track.phase = 0;
            track.xyz = seg.xyz;
            track.uv = seg.uv;
            tracks.push_back(track);
        }

        // This is a simplified implementation
        // Full implementation would require all the logic from the Python version

        return result;
    }

} // namespace SprayTrajPlan
