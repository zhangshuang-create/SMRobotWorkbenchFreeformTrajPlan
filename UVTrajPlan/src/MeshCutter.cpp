#include "MeshCutter.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

namespace SprayTrajPlan
{
    namespace
    {
        constexpr size_t kReflectionSampleCount = 600;

        std::vector<Eigen::Vector3d> resampleClosedPolyline(
            const std::vector<Eigen::Vector3d>& points)
        {
            std::vector<Eigen::Vector3d> uniquePoints;
            uniquePoints.reserve(points.size());
            for (const auto& point : points) {
                if (uniquePoints.empty() ||
                    (point - uniquePoints.back()).squaredNorm() > 1e-18) {
                    uniquePoints.push_back(point);
                }
            }
            if (uniquePoints.size() > 1 &&
                (uniquePoints.front() - uniquePoints.back()).squaredNorm() <= 1e-18) {
                uniquePoints.pop_back();
            }
            if (uniquePoints.size() < 3) return {};

            std::vector<double> cumulative(uniquePoints.size() + 1, 0.0);
            for (size_t i = 0; i < uniquePoints.size(); ++i) {
                cumulative[i + 1] = cumulative[i] +
                    (uniquePoints[(i + 1) % uniquePoints.size()] - uniquePoints[i]).norm();
            }
            if (cumulative.back() <= 1e-12) return {};

            std::vector<Eigen::Vector3d> samples;
            samples.reserve(kReflectionSampleCount);
            size_t segment = 0;
            for (size_t i = 0; i < kReflectionSampleCount; ++i) {
                const double distance = cumulative.back() * static_cast<double>(i) /
                    static_cast<double>(kReflectionSampleCount);
                while (segment + 1 < uniquePoints.size() && distance > cumulative[segment + 1]) ++segment;
                const Eigen::Vector3d& start = uniquePoints[segment];
                const Eigen::Vector3d& end = uniquePoints[(segment + 1) % uniquePoints.size()];
                const double t = (distance - cumulative[segment]) /
                    (cumulative[segment + 1] - cumulative[segment]);
                samples.push_back(start + t * (end - start));
            }
            return samples;
        }

        double pointToSampleCloudSquaredDistance(const Eigen::Vector3d& point,
            const std::vector<Eigen::Vector3d>& samples)
        {
            double minimum = std::numeric_limits<double>::infinity();
            for (const auto& sample : samples) {
                minimum = std::min(minimum, (point - sample).squaredNorm());
            }
            return minimum;
        }

        bool fitReflectionSymmetryNormal(const std::vector<Eigen::Vector3d>& topPoints,
            Eigen::Vector3d& normalOut)
        {
            const auto samples = resampleClosedPolyline(topPoints);
            if (samples.size() != kReflectionSampleCount) return false;

            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (const auto& point : samples) center += point;
            center /= static_cast<double>(samples.size());

            std::vector<Eigen::Vector3d> centeredSamples;
            centeredSamples.reserve(samples.size());
            for (const auto& point : samples) centeredSamples.push_back(point - center);
            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (const auto& delta : centeredSamples) {
                covariance += delta * delta.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
            if (solver.info() != Eigen::Success) return false;
            const Eigen::Vector3d xApprox = solver.eigenvectors().col(2);
            const Eigen::Vector3d zApprox = solver.eigenvectors().col(0);
            const Eigen::Vector3d yApprox = zApprox.cross(xApprox).normalized();

            auto objective = [&](const Eigen::Vector3d& parameters) {
                const Eigen::Vector3d inPlane = std::cos(parameters.x()) * xApprox +
                    std::sin(parameters.x()) * yApprox;
                Eigen::Vector3d normal = std::cos(parameters.y()) * inPlane +
                    std::sin(parameters.y()) * zApprox;
                if (normal.norm() <= 1e-12) return std::numeric_limits<double>::infinity();
                normal.normalize();
                double total = 0.0;
                for (const auto& point : centeredSamples) {
                    const Eigen::Vector3d reflected = point - 2.0 * normal.dot(point) * normal;
                    total += pointToSampleCloudSquaredDistance(reflected, centeredSamples);
                }
                return total / static_cast<double>(samples.size());
            };

            std::array<std::pair<double, Eigen::Vector3d>, 120> seeds;
            for (size_t i = 0; i < seeds.size(); ++i) {
                const Eigen::Vector3d parameters(2.0 * std::acos(-1.0) * static_cast<double>(i) / seeds.size(), 0.0, 0.0);
                seeds[i] = { objective(parameters), parameters };
            }
            std::sort(seeds.begin(), seeds.end(),
                [](const auto& left, const auto& right) { return left.first < right.first; });

            double bestScore = std::numeric_limits<double>::infinity();
            Eigen::Vector3d best = Eigen::Vector3d::Zero();
            for (size_t start = 0; start < 1; ++start) {
                Eigen::Vector3d parameters = seeds[start].second;
                double score = seeds[start].first;
                Eigen::Vector2d step(0.10, 0.05);
                for (int iteration = 0; iteration < 100; ++iteration) {
                    bool improved = false;
                    for (int dimension = 0; dimension < 2; ++dimension) {
                        for (const double sign : { -1.0, 1.0 }) {
                            Eigen::Vector3d candidate = parameters;
                            candidate[dimension] += sign * step[dimension];
                            const double candidateScore = objective(candidate);
                            if (candidateScore < score) {
                                parameters = candidate;
                                score = candidateScore;
                                improved = true;
                            }
                        }
                    }
                    if (!improved) step *= 0.5;
                    if (step.maxCoeff() < 1e-6) break;
                }
                if (score < bestScore) {
                    bestScore = score;
                    best = parameters;
                }
            }
            const Eigen::Vector3d inPlane = std::cos(best.x()) * xApprox +
                std::sin(best.x()) * yApprox;
            normalOut = std::cos(best.y()) * inPlane + std::sin(best.y()) * zApprox;
            if (!std::isfinite(bestScore) || normalOut.norm() <= 1e-12) return false;
            normalOut.normalize();
            return true;
        }

        std::vector<int> planeBoundaryCandidates(const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<int>& loop, const Eigen::Vector3d& normal, double offset)
        {
            std::vector<int> candidates;
            for (size_t i = 0; i < loop.size(); ++i) {
                const int first = loop[i];
                const int second = loop[(i + 1) % loop.size()];
                const double firstDistance = normal.dot(vertices[first]) + offset;
                const double secondDistance = normal.dot(vertices[second]) + offset;
                if (firstDistance == 0.0 || secondDistance == 0.0 || firstDistance * secondDistance < 0.0) {
                    candidates.push_back(std::abs(firstDistance) <= std::abs(secondDistance) ? first : second);
                }
            }
            std::sort(candidates.begin(), candidates.end());
            candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
            if (candidates.size() >= 2) return candidates;

            std::vector<std::pair<double, int>> closest;
            closest.reserve(loop.size());
            for (const int vertex : loop) closest.emplace_back(std::abs(normal.dot(vertices[vertex]) + offset), vertex);
            std::sort(closest.begin(), closest.end());
            for (const auto& [distance, vertex] : closest) {
                if (std::find(candidates.begin(), candidates.end(), vertex) == candidates.end()) candidates.push_back(vertex);
                if (candidates.size() == 2) break;
            }
            return candidates;
        }
    }

    // ============================================================================
    // MeshCutter Implementation
    // ============================================================================

    std::pair<std::vector<int>, std::vector<std::pair<int, int>>>
    MeshCutter::boundaryVerticesFromFaces(const std::vector<Eigen::Vector3i>& faces)
    {
        // Collect all edges
        std::map<std::pair<int, int>, int> edge_count;

        for (const auto& face : faces) {
            for (int i = 0; i < 3; ++i) {
                int v0 = face[i];
                int v1 = face[(i + 1) % 3];

                std::pair<int, int> edge = {std::min(v0, v1), std::max(v0, v1)};
                edge_count[edge]++;
            }
        }

        // Boundary edges appear exactly once
        std::vector<std::pair<int, int>> boundary_edges;
        std::set<int> boundary_vertices_set;

        for (const auto& [edge, count] : edge_count) {
            if (count == 1) {
                boundary_edges.push_back(edge);
                boundary_vertices_set.insert(edge.first);
                boundary_vertices_set.insert(edge.second);
            }
        }

        std::vector<int> boundary_vertices(boundary_vertices_set.begin(), boundary_vertices_set.end());

        return {boundary_vertices, boundary_edges};
    }

    std::vector<std::vector<int>>
    MeshCutter::orderedBoundaryLoops(const std::vector<Eigen::Vector3i>& faces)
    {
        auto [boundary_verts, boundary_edges] = boundaryVerticesFromFaces(faces);

        if (boundary_edges.empty()) {
            return {};
        }

        // Build adjacency
        std::unordered_map<int, std::vector<int>> adjacency;
        std::set<std::pair<int, int>> unused_edges;

        for (const auto& [a, b] : boundary_edges) {
            adjacency[a].push_back(b);
            adjacency[b].push_back(a);
            unused_edges.insert({std::min(a, b), std::max(a, b)});
        }

        std::vector<std::vector<int>> loops;

        while (!unused_edges.empty()) {
            auto [start, current] = *unused_edges.begin();
            unused_edges.erase(unused_edges.begin());

            std::vector<int> loop = {start, current};
            int previous = start;

            while (current != start) {
                int next_vertex = -1;

                for (int candidate : adjacency[current]) {
                    if (candidate == previous) continue;

                    std::pair<int, int> edge = {std::min(current, candidate), std::max(current, candidate)};

                    if (unused_edges.count(edge)) {
                        unused_edges.erase(edge);
                        next_vertex = candidate;
                        break;
                    }
                }

                if (next_vertex == -1) {
                    throw std::runtime_error("Mesh boundary is open, branched, or non-manifold");
                }

                previous = current;
                current = next_vertex;

                if (current != start) {
                    loop.push_back(current);
                }
            }

            loops.push_back(loop);
        }

        return loops;
    }

    std::vector<std::pair<int, int>>
    MeshCutter::meshEdgesFromFaces(const std::vector<Eigen::Vector3i>& faces)
    {
        std::set<std::pair<int, int>> edges_set;

        for (const auto& face : faces) {
            for (int i = 0; i < 3; ++i) {
                int v0 = face[i];
                int v1 = face[(i + 1) % 3];
                edges_set.insert({std::min(v0, v1), std::max(v0, v1)});
            }
        }

        return std::vector<std::pair<int, int>>(edges_set.begin(), edges_set.end());
    }

    std::vector<std::set<int>> MeshCutter::findVertexComponents(
        int vertex,
        const std::vector<int>& incident_faces,
        const std::vector<Eigen::Vector3i>& faces,
        const std::set<std::pair<int, int>>& seam_edges)
    {
        // Build adjacency graph of incident faces (excluding seam edges)
        std::unordered_map<int, std::unordered_set<int>> adjacency;

        for (int face_id : incident_faces) {
            adjacency[face_id] = {};
        }

        // Connect faces that share non-seam edges
        for (int face_id : incident_faces) {
            const auto& face = faces[face_id];

            for (int i = 0; i < 3; ++i) {
                int v0 = face[i];
                int v1 = face[(i + 1) % 3];

                if (v0 != vertex && v1 != vertex) continue;

                std::pair<int, int> edge = {std::min(v0, v1), std::max(v0, v1)};

                if (seam_edges.count(edge)) continue;

                // Find other face sharing this edge
                for (int other_face_id : incident_faces) {
                    if (other_face_id == face_id) continue;

                    const auto& other_face = faces[other_face_id];
                    bool shares_edge = false;

                    for (int j = 0; j < 3; ++j) {
                        int w0 = other_face[j];
                        int w1 = other_face[(j + 1) % 3];
                        std::pair<int, int> other_edge = {std::min(w0, w1), std::max(w0, w1)};

                        if (edge == other_edge) {
                            shares_edge = true;
                            break;
                        }
                    }

                    if (shares_edge) {
                        adjacency[face_id].insert(other_face_id);
                        adjacency[other_face_id].insert(face_id);
                    }
                }
            }
        }

        // Find connected components
        std::unordered_set<int> remaining(incident_faces.begin(), incident_faces.end());
        std::vector<std::set<int>> components;

        while (!remaining.empty()) {
            int start = *remaining.begin();
            remaining.erase(start);

            std::set<int> component = {start};
            std::queue<int> queue;
            queue.push(start);

            while (!queue.empty()) {
                int current = queue.front();
                queue.pop();

                for (int neighbor : adjacency[current]) {
                    if (remaining.count(neighbor)) {
                        remaining.erase(neighbor);
                        component.insert(neighbor);
                        queue.push(neighbor);
                    }
                }
            }

            components.push_back(component);
        }

        return components;
    }

    std::map<int, std::set<int>>
    MeshCutter::facesToDuplicateAlongSeam(
        const std::vector<Eigen::Vector3i>& faces,
        const std::vector<int>& seam_path)
    {
        if (seam_path.size() < 2) {
            throw std::invalid_argument("Seam must have at least two vertices");
        }

        // Build seam edges
        std::set<std::pair<int, int>> seam_edges;
        for (size_t i = 0; i < seam_path.size() - 1; ++i) {
            int a = seam_path[i];
            int b = seam_path[i + 1];
            seam_edges.insert({std::min(a, b), std::max(a, b)});
        }

        // Find incident faces for each seam vertex
        std::unordered_map<int, std::vector<int>> incident;
        for (int v : seam_path) {
            incident[v] = {};
        }

        for (size_t face_id = 0; face_id < faces.size(); ++face_id) {
            const auto& face = faces[face_id];
            for (int v : face) {
                if (incident.count(v)) {
                    incident[v].push_back(static_cast<int>(face_id));
                }
            }
        }

        // Find components for each vertex
        std::map<int, std::set<int>> selected;

        for (size_t i = 0; i < seam_path.size(); ++i) {
            int vertex = seam_path[i];
            auto components = findVertexComponents(vertex, incident[vertex], faces, seam_edges);

            const bool endpoint = i == 0 || i + 1 == seam_path.size();
            // A path endpoint lies on an existing open boundary. Removing its
            // single incident seam edge leaves one fan of faces, which is valid;
            // only interior seam vertices must split into two fans.
            if (components.empty() || (!endpoint && components.size() != 2)) {
                throw std::runtime_error(
                    "Seam vertex " + std::to_string(vertex) +
                    " is not locally manifold (found " + std::to_string(components.size()) + " components)"
                );
            }

            if (endpoint && components.size() == 1) {
                selected[vertex] = components.front();
                continue;
            }

            // Select component based on adjacent seam edges
            if (i == 0) {
                // First vertex: pick component containing first edge's adjacent face
                int next = seam_path[1];
                std::pair<int, int> edge = {std::min(vertex, next), std::max(vertex, next)};

                for (int face_id : incident[vertex]) {
                    const auto& face = faces[face_id];
                    bool has_edge = false;

                    for (int j = 0; j < 3; ++j) {
                        int v0 = face[j];
                        int v1 = face[(j + 1) % 3];
                        if ((std::min(v0, v1) == edge.first) && (std::max(v0, v1) == edge.second)) {
                            has_edge = true;
                            break;
                        }
                    }

                    if (has_edge) {
                        for (const auto& comp : components) {
                            if (comp.count(face_id)) {
                                selected[vertex] = comp;
                                break;
                            }
                        }
                        break;
                    }
                }
            } else {
                // Continue with component containing previous selected face
                int prev = seam_path[i - 1];
                std::pair<int, int> edge = {std::min(prev, vertex), std::max(prev, vertex)};

                int selected_face = -1;
                for (int face_id : incident[vertex]) {
                    if (!selected[prev].count(face_id)) continue;

                    const auto& face = faces[face_id];
                    for (int j = 0; j < 3; ++j) {
                        int v0 = face[j];
                        int v1 = face[(j + 1) % 3];
                        if ((std::min(v0, v1) == edge.first) && (std::max(v0, v1) == edge.second)) {
                            selected_face = face_id;
                            break;
                        }
                    }
                    if (selected_face >= 0) break;
                }

                for (const auto& comp : components) {
                    if (comp.count(selected_face)) {
                        selected[vertex] = comp;
                        break;
                    }
                }
            }
        }

        return selected;
    }

    bool MeshCutter::cutAlongPath(
        const std::vector<Eigen::Vector3d>& vertices,
        const std::vector<Eigen::Vector3i>& faces,
        const std::vector<int>& seam_path,
        std::vector<Eigen::Vector3d>& output_vertices,
        std::vector<Eigen::Vector3i>& output_faces,
        std::vector<int>& vertex_map)
    {
        try {
            auto duplicate_faces = facesToDuplicateAlongSeam(faces, seam_path);

            // Create duplicate vertex map
            std::unordered_map<int, int> dup_map;
            output_vertices = vertices;

            for (int seam_vertex : seam_path) {
                int new_idx = static_cast<int>(output_vertices.size());
                dup_map[seam_vertex] = new_idx;
                output_vertices.push_back(vertices[seam_vertex]);
            }

            // Update faces
            output_faces.reserve(faces.size());
            for (size_t i = 0; i < faces.size(); ++i) {
                Eigen::Vector3i face = faces[i];

                // Check if this face should use duplicated vertices
                for (int seam_vertex : seam_path) {
                    if (duplicate_faces.count(seam_vertex) && duplicate_faces[seam_vertex].count(static_cast<int>(i))) {
                        // Replace vertices with duplicates
                        for (int j = 0; j < 3; ++j) {
                            if (face[j] == seam_vertex) {
                                face[j] = dup_map[seam_vertex];
                            }
                        }
                    }
                }

                output_faces.push_back(face);
            }

            // Build vertex map
            vertex_map.resize(output_vertices.size());
            for (size_t i = 0; i < vertices.size(); ++i) {
                vertex_map[i] = static_cast<int>(i);
            }
            for (const auto& [old_idx, new_idx] : dup_map) {
                vertex_map[new_idx] = old_idx;
            }

            return true;

        } catch (const std::exception& e) {
            std::cerr << "Cut along path failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool MeshCutter::precutTwoBoundaries(
        const std::vector<Eigen::Vector3d>& vertices,
        const std::vector<Eigen::Vector3i>& faces,
        std::vector<Eigen::Vector3d>& output_vertices,
        std::vector<Eigen::Vector3i>& output_faces,
        PrecutSeam& seam_data)
    {
        // Find two boundary loops
        std::vector<int> boundary1, boundary2;
        if (!BoundaryAnalyzer::hasTwoBoundaries(faces, boundary1, boundary2)) {
            std::cerr << "Mesh does not have exactly two boundaries" << std::endl;
            return false;
        }

        // The UV pre-cut shares the adaptive slicer's reference-plane
        // definition: fit the upper rim's continuous 3D mirror symmetry,
        // then anchor that direction at the lower boundary center.
        const auto info1 = BoundaryAnalyzer::analyzeBoundaryLoop(vertices, boundary1);
        const auto info2 = BoundaryAnalyzer::analyzeBoundaryLoop(vertices, boundary2);
        const bool firstIsTop = info1.center.z() >= info2.center.z();
        const auto& topLoop = firstIsTop ? boundary1 : boundary2;
        const auto& bottomLoop = firstIsTop ? boundary2 : boundary1;
        const auto& topInfo = firstIsTop ? info1 : info2;
        const auto& bottomInfo = firstIsTop ? info2 : info1;

        Eigen::Vector3d planeNormal;
        if (!fitReflectionSymmetryNormal(topInfo.points, planeNormal)) {
            std::cerr << "Failed to fit UV pre-cut reflection symmetry plane" << std::endl;
            return false;
        }
        const double planeOffset = -planeNormal.dot(bottomInfo.center);
        const auto topCandidates = planeBoundaryCandidates(
            vertices, topLoop, planeNormal, planeOffset);
        const auto bottomCandidates = planeBoundaryCandidates(
            vertices, bottomLoop, planeNormal, planeOffset);
        if (topCandidates.empty() || bottomCandidates.empty()) {
            std::cerr << "Reference plane did not identify UV seam endpoints" << std::endl;
            return false;
        }

        // Retain the existing mesh-graph seam extraction and cut operation;
        // only its boundary endpoints are now selected by the fitted plane.
        std::vector<int> seam_path;
        double shortestLength = std::numeric_limits<double>::infinity();
        for (const int bottomVertex : bottomCandidates) {
            for (const int topVertex : topCandidates) {
                const auto candidate = BoundaryAnalyzer::findShortestSeamPath(
                    vertices, faces, { bottomVertex }, { topVertex });
                if (candidate.empty()) continue;
                double length = 0.0;
                for (size_t i = 1; i < candidate.size(); ++i) {
                    length += (vertices[candidate[i]] - vertices[candidate[i - 1]]).norm();
                }
                if (length < shortestLength) {
                    shortestLength = length;
                    seam_path = candidate;
                }
            }
        }

        if (seam_path.empty()) {
            std::cerr << "Failed to find seam path on the reference plane" << std::endl;
            return false;
        }

        // Cut along seam
        std::vector<int> vertex_map;
        if (!cutAlongPath(vertices, faces, seam_path, output_vertices, output_faces, vertex_map)) {
            return false;
        }

        // Store seam data
        seam_data.original_vertex_ids = seam_path;
        seam_data.cut_vertex_pairs.clear();
        // cutAlongPath appends one duplicate for each seam vertex in path order.
        // Preserve this correspondence so the unfolded OBJ can recover both
        // UV copies of the physical seam, matching the cache contract.
        for (size_t index = 0; index < seam_path.size(); ++index) {
            seam_data.cut_vertex_pairs.emplace_back(seam_path[index],
                static_cast<int>(vertices.size() + index));
        }
        seam_data.xyz.clear();
        for (int vid : seam_path) {
            seam_data.xyz.push_back(vertices[vid]);
        }

        return true;
    }

    // ============================================================================
    // BoundaryAnalyzer Implementation
    // ============================================================================

    BoundaryAnalyzer::BoundaryInfo BoundaryAnalyzer::analyzeBoundaryLoop(
        const std::vector<Eigen::Vector3d>& vertices,
        const std::vector<int>& loop)
    {
        BoundaryInfo info;
        info.is_closed = true;

        if (loop.empty()) return info;

        // Extract points
        for (int vid : loop) {
            info.points.push_back(vertices[vid]);
        }

        // Compute center
        info.center = Eigen::Vector3d::Zero();
        for (const auto& pt : info.points) {
            info.center += pt;
        }
        info.center /= static_cast<double>(info.points.size());

        // Compute perimeter
        info.perimeter = 0.0;
        for (size_t i = 0; i < info.points.size(); ++i) {
            size_t next = (i + 1) % info.points.size();
            info.perimeter += (info.points[next] - info.points[i]).norm();
        }

        // Estimate normal using PCA
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const auto& pt : info.points) {
            Eigen::Vector3d centered = pt - info.center;
            covariance += centered * centered.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        info.normal = solver.eigenvectors().col(0);  // Smallest eigenvalue

        return info;
    }

    bool BoundaryAnalyzer::hasTwoBoundaries(
        const std::vector<Eigen::Vector3i>& faces,
        std::vector<int>& boundary1,
        std::vector<int>& boundary2)
    {
        auto loops = MeshCutter::orderedBoundaryLoops(faces);

        if (loops.size() != 2) {
            return false;
        }

        boundary1 = loops[0];
        boundary2 = loops[1];
        return true;
    }

    std::vector<int> BoundaryAnalyzer::findShortestSeamPath(
        const std::vector<Eigen::Vector3d>& vertices,
        const std::vector<Eigen::Vector3i>& faces,
        const std::vector<int>& boundary1,
        const std::vector<int>& boundary2)
    {
        if (vertices.empty()) return {};

        std::vector<std::vector<std::pair<int, double>>> adjacency(vertices.size());
        for (const auto& face : faces) {
            for (int i = 0; i < 3; ++i) {
                const int a = face[i];
                const int b = face[(i + 1) % 3];
                if (a < 0 || b < 0 || a >= static_cast<int>(vertices.size()) || b >= static_cast<int>(vertices.size())) return {};
                const double weight = (vertices[a] - vertices[b]).norm();
                adjacency[a].emplace_back(b, weight);
                adjacency[b].emplace_back(a, weight);
            }
        }

        std::vector<bool> target(vertices.size(), false);
        for (int vertex : boundary2) if (vertex >= 0 && vertex < static_cast<int>(vertices.size())) target[vertex] = true;
        using QueueItem = std::pair<double, int>;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
        std::vector<double> distances(vertices.size(), std::numeric_limits<double>::max());
        std::vector<int> previous(vertices.size(), -1);
        for (int vertex : boundary1) {
            if (vertex < 0 || vertex >= static_cast<int>(vertices.size())) continue;
            distances[vertex] = 0.0;
            queue.emplace(0.0, vertex);
        }
        int destination = -1;
        while (!queue.empty()) {
            const auto [distance, vertex] = queue.top();
            queue.pop();
            if (distance != distances[vertex]) continue;
            if (target[vertex]) { destination = vertex; break; }
            for (const auto& [neighbor, weight] : adjacency[vertex]) {
                const double candidate = distance + weight;
                if (candidate < distances[neighbor]) {
                    distances[neighbor] = candidate;
                    previous[neighbor] = vertex;
                    queue.emplace(candidate, neighbor);
                }
            }
        }
        if (destination < 0) return {};
        std::vector<int> path;
        for (int vertex = destination; vertex >= 0; vertex = previous[vertex]) path.push_back(vertex);
        std::reverse(path.begin(), path.end());
        return path;
    }

} // namespace SprayTrajPlan
