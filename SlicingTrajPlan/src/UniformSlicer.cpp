#include <SlicingTrajPlan/UniformSlicer.h>
#include <SlicingTrajPlan/GeneralData.h>
#include <SlicingTrajPlan/STLProcessor.h>
#include <SlicingTrajPlan/BoundaryDetector.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <map>
#include <limits>
#include <iostream>
#include <array>
#include <tuple>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ModelTrajPlan
{
UniformSlicer::UniformSlicer()
{
    // [DESIGN] Pure algorithm class - no internal data storage
}

UniformSlicer::~UniformSlicer()
{
}

bool UniformSlicer::performSlicing(const ModelData& modelData,
    const BoundaryLine& upperBoundary,
    const BoundaryLine& lowerBoundary,
    const SlicingParams& params,
    SlicingLocation& slicingLocations,
    Epiphyseal& slicingnode,
    SliceResult& sliceResult,
    std::vector<CuttingPlane>* cuttingPlanes,
    const std::vector<CuttingPlane>* oldCuttingPlanes)
{
    // [ALGORITHM] Perform slicing on model
    // Input: modelData - STL model
    //        upperBoundary, lowerBoundary - boundary data
    //        params - slicing parameters
    //        oldCuttingPlanes - previous cutting planes (optional, for preserving transformations)
    // Output: sliceResult - slicing result
    //         cuttingPlanes - basic cutting planes for display (optional output)
    
    try {
        // Clear and initialize result structure
        sliceResult = SliceResult();
        sliceResult.usedParams = params;

        // 鐠侊紕鐣籞閸婅偐娈戦懠鍐ㄦ纯
        double zMin = std::numeric_limits<double>::max();
        double zMax = std::numeric_limits<double>::lowest();

        if (!upperBoundary.points.empty()) {
            for (const auto& point : upperBoundary.points) {
                zMax = std::max(zMax, point.position.z());
            }
        }

        if (!lowerBoundary.points.empty()) {
            for (const auto& point : lowerBoundary.points) {
                zMin = std::min(zMin, point.position.z());
            }
        }
        slicingLocations.clear();
        const bool hasBothBoundaries = !upperBoundary.points.empty() &&
            !lowerBoundary.points.empty();
        if (!hasBothBoundaries) {
            // A closed mesh has no physical end rims or skeleton.  Its
            // editable slicing domain is therefore the global STL Z extent.
            Eigen::Vector3d boundsMin = Eigen::Vector3d::Zero();
            Eigen::Vector3d boundsMax = Eigen::Vector3d::Zero();
            STLProcessor::calculateBounds(modelData.mainModelData_, boundsMin, boundsMax);
            zMin = boundsMin.z();
            zMax = boundsMax.z();
            const double height = zMax - zMin;
            if (!std::isfinite(zMin) || !std::isfinite(zMax) || height <= 1e-9 ||
                !std::isfinite(params.minStep) || params.minStep <= 0.0) {
                return false;
            }

            const int layerCount = params.maxSliceCount > 0
                ? params.maxSliceCount
                : std::max(2, static_cast<int>(std::floor(height / params.minStep)) + 1);
            slicingLocations.reserve(static_cast<size_t>(layerCount));
            for (int index = 0; index < layerCount; ++index) {
                // Keep planes inside the mesh range to avoid vertex-only
                // intersections at the two extrema.
                const double ratio = static_cast<double>(index + 1) /
                    static_cast<double>(layerCount + 1);
                slicingLocations.push_back({
                    Eigen::Vector3d(0.0, 0.0, zMin + height * ratio),
                    Eigen::Vector3d::UnitZ() });
            }
            sliceResult.isBoundarylessFallback = true;
        }
        else if (params.mode == SlicingMode::Parallel) {
            if (upperBoundary.points.empty() || lowerBoundary.points.empty() ||
                !std::isfinite(params.minStep) || params.minStep <= 0.0 ||
                !std::isfinite(zMin) || !std::isfinite(zMax)) {
                return false;
            }

            // Parallel slicing uses the requested boundary overlap range, not
            // the global STL bounds: upperMinZ - lowerMaxZ.
            double upperMinZ = std::numeric_limits<double>::max();
            double lowerMaxZ = std::numeric_limits<double>::lowest();
            for (const auto& point : upperBoundary.points) {
                upperMinZ = std::min(upperMinZ, point.position.z());
            }
            for (const auto& point : lowerBoundary.points) {
                lowerMaxZ = std::max(lowerMaxZ, point.position.z());
            }

            const double height = upperMinZ - lowerMaxZ;
            const int intervalCount = static_cast<int>(std::floor(height / params.minStep));
            if (!std::isfinite(height) || height <= 0.0 || intervalCount <= 0) {
                return false;
            }

            // The two boundary contours are consumed directly by trajectory
            // generation. Only create the strictly internal planes here;
            // otherwise the top/bottom boundary would be represented twice.
            // A positive limit is an upper bound on the final internal planes,
            // for both adaptive and parallel (uniform) slicing.  When capped,
            // redistribute the retained planes across the original range.
            const int uncappedInternalCount = std::max(0, intervalCount - 1);
            const int internalCount = params.maxSliceCount > 0
                ? std::min(uncappedInternalCount, params.maxSliceCount)
                : uncappedInternalCount;
            slicingLocations.reserve(static_cast<size_t>(internalCount));
            for (int i = 1; i <= internalCount; ++i) {
                const double ratio = static_cast<double>(i) /
                    static_cast<double>(internalCount + 1);
                SlicePlane plane;
                plane.point = Eigen::Vector3d(0.0, 0.0, lowerMaxZ + height * ratio);
                plane.normal = Eigen::Vector3d::UnitZ();
                slicingLocations.push_back(plane);
            }
        }
        else if (!buildSlicePlanes(slicingnode, slicingLocations)) {
            return false;
        }
        // Generate internal boundary lines
        sliceResult.internalBoundaries = generateInternalBoundaries(modelData, params, zMin, zMax, slicingLocations);
        if (sliceResult.internalBoundaries.size() != slicingLocations.size()) {
            if (cuttingPlanes) cuttingPlanes->clear();
            return false;
        }

        // 閵嗘劖鏌婃晶鐐偓鎴濐洤閺嬫粏顕Ч鍌滄晸閹?cuttingPlanes閿涘奔绮?sliceResult 閻㈢喐鍨氶崺铏诡攨閻?cuttingPlanes閿涘牏鏁ゆ禍搴㈡▔缁€鐚寸礆
        if (cuttingPlanes && !sliceResult.internalBoundaries.empty()) {
            cuttingPlanes->clear();
            cuttingPlanes->reserve(sliceResult.internalBoundaries.size());
            
            // 娑撶儤鐦℃稉顏囩珶閻ｅ瞼鏁撻幋鎰娑擃亜鐔€绾偓閻?CuttingPlane
            for (size_t i = 0; i < sliceResult.internalBoundaries.size(); ++i) {
                const auto& boundary = sliceResult.internalBoundaries[i];
                
                CuttingPlane plane;
                plane.layer_index = static_cast<int>(i);
                
                // 娴兼ê鍘涙担璺ㄦ暏 buildSlicePlanes 閻㈢喐鍨氶惃鍕殰闁倸绨查獮鎶芥桨婵寧鈧?
                if (i < slicingLocations.size()) {
                    plane.center = slicingLocations[i].point;
                    plane.normal = slicingLocations[i].normal.normalized();
                } else if (!boundary.points.empty()) {
                    Eigen::Vector3d centerSum(0, 0, 0);
                    for (const auto& point : boundary.points) {
                        centerSum += point.position;
                    }
                    plane.center = centerSum / static_cast<double>(boundary.points.size());
                    plane.normal = Eigen::Vector3d(0, 0, 1);
                } else {
                    plane.center = Eigen::Vector3d(0, 0, boundary.z_level);
                    plane.normal = Eigen::Vector3d(0, 0, 1);
                }

                // 閹稿閽╅棃銏＄《閸氭垶鐎柅鐘茬湰闁劌娼楅弽鍥╅兇閿涘瞼鏁撻幋鎰瑢婵寧鈧椒绔撮懛瀵告畱閻晛鑸版い鍓佸仯
                Eigen::Vector3d refAxis(0, 0, 1);
                if (std::abs(plane.normal.dot(refAxis)) > 0.95) {
                    refAxis = Eigen::Vector3d(1, 0, 0);
                }

                Eigen::Vector3d localX = plane.normal.cross(refAxis).normalized();
                Eigen::Vector3d localY = plane.normal.cross(localX).normalized();

                // 閸︺劌缍嬮崜宥呭瀼楠炴娊娼扮仦鈧柈銊ユ綏閺嶅洨閮存稉瀣剁礉閹稿顕氱仦鍌濈珶閻ｅ瞼鍋ｇ拋锛勭暬閸栧懎娲块惄鎺炵礉閸愬秴娲撴稉顏呮煙閸氭垵鎮囬幍?0%
                const double planeMargin = 0.1;  // each side 10%, total 1.2x
                double minLocalX = std::numeric_limits<double>::max();
                double maxLocalX = std::numeric_limits<double>::lowest();
                double minLocalY = std::numeric_limits<double>::max();
                double maxLocalY = std::numeric_limits<double>::lowest();

                if (!boundary.points.empty()) {
                    for (const auto& bp : boundary.points) {
                        Eigen::Vector3d rel = bp.position - plane.center;
                        double x = rel.dot(localX);
                        double y = rel.dot(localY);
                        minLocalX = std::min(minLocalX, x);
                        maxLocalX = std::max(maxLocalX, x);
                        minLocalY = std::min(minLocalY, y);
                        maxLocalY = std::max(maxLocalY, y);
                    }
                } else {
                    // degenerate fallback size
                    minLocalX = -1.0;
                    maxLocalX = 1.0;
                    minLocalY = -1.0;
                    maxLocalY = 1.0;
                }

                double xRange = maxLocalX - minLocalX;
                double yRange = maxLocalY - minLocalY;
                if (xRange < 1e-6) xRange = 1.0;
                if (yRange < 1e-6) yRange = 1.0;
                double xPad = xRange * planeMargin;
                double yPad = yRange * planeMargin;

                minLocalX -= xPad;
                maxLocalX += xPad;
                minLocalY -= yPad;
                maxLocalY += yPad;

                plane.vertices.resize(4);
                plane.vertices[0] = plane.center + localX * minLocalX + localY * minLocalY;
                plane.vertices[1] = plane.center + localX * maxLocalX + localY * minLocalY;
                plane.vertices[2] = plane.center + localX * maxLocalX + localY * maxLocalY;
                plane.vertices[3] = plane.center + localX * minLocalX + localY * maxLocalY;
                
                // 娣囨繄鏆€閺冄呮畱閸欐ɑ宕查崣鍌涙殶閿涘牆顩ч弸婊冪摠閸︻煉绱?
                if (oldCuttingPlanes && oldCuttingPlanes->size() > i) {
                    plane.translation_x = (*oldCuttingPlanes)[i].translation_x;
                    plane.translation_y = (*oldCuttingPlanes)[i].translation_y;
                    plane.translation_z = (*oldCuttingPlanes)[i].translation_z;
                    plane.rotation_x = (*oldCuttingPlanes)[i].rotation_x;
                    plane.rotation_y = (*oldCuttingPlanes)[i].rotation_y;
                    plane.rotation_z = (*oldCuttingPlanes)[i].rotation_z;
                }
                
                cuttingPlanes->push_back(plane);
            }
        }

        // 閵嗘劙鍣搁弸鍕┾偓鎴滅瑝閸愬秷顓哥粻妤佺《閸氭垿鍣洪崪灞芥磽閸忓啯鏆?- 鏉╂瑤绨洪悽?TrajectoryGenerator 鐠愮喕鐭?
        // UniformSlicer 閸欘亣绀嬬拹锝呭殤娴ｆ洘鐪版禍銈忕礉鐠侊紕鐣绘担宥囩枂娣団剝浼?
        // A valid Parallel range may contain only the two boundary contours
        // when floor(height / minStep) == 1, so an empty internal contour set
        // is still a valid slicing result in that mode.
        if (sliceResult.isBoundarylessFallback) {
            return !sliceResult.internalBoundaries.empty();
        }
        return params.mode == SlicingMode::Parallel
            ? (std::isfinite(zMin) && std::isfinite(zMax) && !upperBoundary.points.empty() && !lowerBoundary.points.empty())
            : !sliceResult.internalBoundaries.empty();
    }
    catch (const std::exception&) {
        return false;
    }
}

std::vector<BoundaryLine> UniformSlicer::generateInternalBoundaries(
    const ModelData& modelData,
    const SlicingParams& params,
    double zMin, double zMax,
    const SlicingLocation& slicingLocations
)
{
    // [ALGORITHM] Generate internal boundaries for slicing
    // Input: modelData - STL model
    //        params - slicing parameters
    //        zMin, zMax - Z range
    // Output: vector<BoundaryLine> - internal boundaries (sliceLines_)
    
    std::vector<BoundaryLine> boundaries;
    (void)params;
    (void)zMin;
    (void)zMax;

    if (slicingLocations.empty()) {
        return boundaries;
    }
    
    // 鏉堟挸鍙嗛崚鍥ч挬闂堛垺娼甸懛?buildSlicePlanes閿涘牆鐔€娴?slicingnode閿涘绱濋柅鎰挬闂堛垺鐪版禍銈囨晸閹存劘绔熼悾?
    for (size_t idx = 0; idx < slicingLocations.size(); ++idx) {
        std::vector<BoundaryLine> contours = traceContoursFromIntersections(
            modelData, slicingLocations[idx]);

        if (contours.empty()) {
            // Do not shift subsequent contours onto the wrong plane indices.
            return {};
        }

        boundaries.push_back(contours[0]);
    }

    return boundaries;
}

bool UniformSlicer::convertToCuttingPlanes(
    SliceResult& sliceResult,
    const std::vector<CuttingPlane>* oldCuttingPlanes,
    const ModelData* modelData)
{
    if (!oldCuttingPlanes || !modelData) return true;
    std::vector<BoundaryLine> updated;
    updated.reserve(oldCuttingPlanes->size());
    for (const auto& plane : *oldCuttingPlanes) {
        const double radians = M_PI / 180.0;
        const Eigen::Matrix3d rotation =
            (Eigen::AngleAxisd(plane.rotation_z * radians, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(plane.rotation_y * radians, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(plane.rotation_x * radians, Eigen::Vector3d::UnitX())).toRotationMatrix();
        SlicePlane transformed;
        transformed.point = plane.center +
            Eigen::Vector3d(plane.translation_x, plane.translation_y, plane.translation_z);
        transformed.normal = rotation * plane.normal;
        // Recompute even at identity: moving back must restore the original
        // contour, not retain the last translated/rotated one.
        auto contours = traceContoursFromIntersections(*modelData, transformed);
        if (contours.empty()) {
            sliceResult.internalBoundaries.clear();
            return false;
        }
        updated.push_back(std::move(contours.front()));
    }
    sliceResult.internalBoundaries = std::move(updated);
    return true;
}


// [ALGORITHM] Trace contours from intersection points
// Input: modelData - STL model
//        planeCenter - plane center point
//        planeNormal - plane normal vector
//        z - Z height
// Output: vector<BoundaryLine> - traced contour lines
std::vector<BoundaryLine> UniformSlicer::traceContoursFromIntersections(
    const ModelData& modelData,
    const SlicePlane& plane)
{
    std::vector<BoundaryLine> contours;
    constexpr double tolerance = 1e-6;
    if (!plane.point.allFinite() || !plane.normal.allFinite() ||
        plane.normal.norm() < tolerance) return contours;
    const Eigen::Vector3d normal = plane.normal.normalized();

    // Weld geometric endpoints, not triangle-edge records. A plane through
    // a mesh ring encounters the same endpoint/segment in several triangles.
    using Cell = std::tuple<long long, long long, long long>;
    std::map<Cell, std::vector<size_t>> cells;
    std::vector<Eigen::Vector3d> points;
    const auto vertexId = [&](const Eigen::Vector3d& input) {
        const Eigen::Vector3d point = input - normal * normal.dot(input - plane.point);
        const long long x = static_cast<long long>(std::floor(point.x() / tolerance));
        const long long y = static_cast<long long>(std::floor(point.y() / tolerance));
        const long long z = static_cast<long long>(std::floor(point.z() / tolerance));
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz) {
                    const auto found = cells.find(Cell{x + dx, y + dy, z + dz});
                    if (found == cells.end()) continue;
                    for (size_t id : found->second)
                        if ((points[id] - point).norm() <= tolerance) return id;
                }
        const size_t id = points.size();
        points.push_back(point);
        cells[Cell{x, y, z}].push_back(id);
        return id;
    };
    using Edge = std::pair<size_t, size_t>;
    struct EdgeUse { bool crossing = false; int coplanarFaces = 0; };
    std::map<Edge, EdgeUse> edges;
    std::set<std::array<size_t, 3>> coplanarFaces;
    const auto addEdge = [&](size_t a, size_t b, bool coplanar) {
        if (a == b) return;
        auto& use = edges[std::minmax(a, b)];
        if (coplanar) ++use.coplanarFaces;
        else use.crossing = true;
    };
    for (size_t triangle = 0;
         triangle < STLProcessor::getTriangleCount(modelData.mainModelData_); ++triangle) {
        Eigen::Vector3d v[3];
        STLProcessor::getTriangleVertices(modelData.mainModelData_, triangle, v);
        double distance[3];
        int side[3];
        bool finite = true;
        for (int i = 0; i < 3; ++i) {
            finite = finite && v[i].allFinite();
            distance[i] = normal.dot(v[i] - plane.point);
            side[i] = distance[i] > tolerance ? 1 : (distance[i] < -tolerance ? -1 : 0);
        }
        if (!finite) continue;
        if (side[0] == 0 && side[1] == 0 && side[2] == 0) {
            std::array<size_t, 3> ids{vertexId(v[0]), vertexId(v[1]), vertexId(v[2])};
            auto key = ids;
            std::sort(key.begin(), key.end());
            if (key[0] == key[1] || key[1] == key[2] || !coplanarFaces.insert(key).second)
                continue;
            for (int i = 0; i < 3; ++i) addEdge(ids[i], ids[(i + 1) % 3], true);
            continue;
        }
        std::vector<size_t> hits;
        for (int i = 0; i < 3; ++i) {
            const int j = (i + 1) % 3;
            if (side[i] == 0) hits.push_back(vertexId(v[i]));
            if (side[i] * side[j] < 0) {
                const double t = distance[i] / (distance[i] - distance[j]);
                hits.push_back(vertexId(v[i] + t * (v[j] - v[i])));
            }
        }
        std::sort(hits.begin(), hits.end());
        hits.erase(std::unique(hits.begin(), hits.end()), hits.end());
        if (hits.size() == 2) addEdge(hits[0], hits[1], false);
    }

    std::vector<std::vector<size_t>> adjacency(points.size());
    for (const auto& entry : edges) {
        // Internal diagonals of a coplanar patch are not contour edges.
        if (!entry.second.crossing && entry.second.coplanarFaces != 1) continue;
        const auto [a, b] = entry.first;
        adjacency[a].push_back(b);
        adjacency[b].push_back(a);
    }
    std::vector<bool> visited(points.size(), false);
    for (size_t start = 0; start < points.size(); ++start) {
        if (visited[start] || adjacency[start].empty()) continue;
        std::vector<size_t> component{start};
        visited[start] = true;
        bool closed = true;
        for (size_t i = 0; i < component.size(); ++i) {
            const size_t id = component[i];
            closed = closed && adjacency[id].size() == 2;
            for (size_t next : adjacency[id]) {
                if (!visited[next]) {
                    visited[next] = true;
                    component.push_back(next);
                }
            }
        }
        // Never bridge a gap with a nearest-point jump or accept A-B-B as a loop.
        if (!closed || component.size() < 3) continue;
        std::vector<size_t> ring;
        size_t previous = points.size(), current = start;
        do {
            ring.push_back(current);
            const auto& neighbors = adjacency[current];
            const size_t next = neighbors[0] == previous ? neighbors[1] : neighbors[0];
            previous = current;
            current = next;
        } while (current != start && ring.size() <= component.size());
        if (current != start || ring.size() != component.size()) continue;

        Eigen::Vector3d areaVector = Eigen::Vector3d::Zero();
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < ring.size(); ++i) {
            center += points[ring[i]];
            areaVector += (points[ring[i]] - points[start]).cross(
                points[ring[(i + 1) % ring.size()]] - points[start]);
        }
        const double signedArea = areaVector.dot(normal);
        if (std::abs(signedArea) <= tolerance * tolerance) continue;
        if (signedArea < 0.0) std::reverse(ring.begin(), ring.end());
        BoundaryLine boundary;
        boundary.normal = normal;
        boundary.center = center / static_cast<double>(ring.size());
        boundary.z_level = normal.dot(boundary.center);
        for (size_t id : ring) {
            BasicPoint point;
            point.position = points[id];
            point.index = static_cast<int>(boundary.points.size());
            const Eigen::Vector3d relative = point.position - boundary.center;
            point.angle = std::atan2(relative.y(), relative.x());
            boundary.points.push_back(point);
        }
        boundary.points.push_back(boundary.points.front()); // explicit closing segment for rendering
        contours.push_back(std::move(boundary));
    }
    const auto perimeter = [](const BoundaryLine& line) {
        double length = 0.0;
        for (size_t i = 1; i < line.points.size(); ++i)
            length += (line.points[i].position - line.points[i - 1].position).norm();
        return length;
    };
    // This API stores one contour per plane. Choose the dominant loop
    // consistently in both initial slicing and subsequent adjustments.
    std::stable_sort(contours.begin(), contours.end(),
        [&](const BoundaryLine& a, const BoundaryLine& b) { return perimeter(a) > perimeter(b); });
    return contours;
}
bool UniformSlicer::buildSlicePlanes(
    const Epiphyseal& slicingnode,
    SlicingLocation& planes)
{
    planes.clear();

    Eigen::Vector3d prevNormal = Eigen::Vector3d::Zero();

    if (slicingnode.size() < 3)
        return false;

    const auto& line1 = slicingnode[0];
    const auto& line2 = slicingnode[1];
    const auto& line3 = slicingnode[2];

    size_t n = std::min({ line1.size(), line2.size(), line3.size() });

    if (n < 2)
        return false;

    bool projected = true;
    for (size_t i = 0; i < n; ++i)
        projected = projected && (line3[i].position - 0.5 *
            (line1[i].position + line2[i].position)).norm() < 1e-7;
    if (projected) {
        if (n == 2) {
            const Eigen::Vector3d tangent = line3.back().position - line3.front().position;
            if (tangent.norm() < 1e-8) return false;
            planes.emplace_back(SlicePlane{0.5 * (line3.front().position + line3.back().position), tangent.normalized()});
        } else {
            for (size_t i = 1; i + 1 < n; ++i) {
                Eigen::Vector3d tangent = line3[i].tangent;
                if (!tangent.allFinite() || tangent.norm() < 1e-8)
                    tangent = line3[i + 1].position - line3[i - 1].position;
                if (tangent.norm() > 1e-8)
                    planes.emplace_back(SlicePlane{line3[i].position, tangent.normalized()});
            }
        }
        return !planes.empty();
    }

    const double tol = 1e-4; // 娴溿倗鍋ｇ€圭懓妯婇敍鍧閿?

    // ============================
    // ? 缁?vs 閹舵鍤?濮瑰倷姘﹂崙鑺ユ殶
    // ============================
    auto intersectLineWithPolyline =
        [&](const Eigen::Vector3d& A,
            const Eigen::Vector3d& B,
            Eigen::Vector3d& outPoint,
            Eigen::Vector3d& outTangent) -> bool
    {
        Eigen::Vector3d u = B - A;
        if (u.norm() < 1e-8) return false;
        u.normalize();

        for (size_t j = 0; j < line3.size() - 1; ++j)
        {
            Eigen::Vector3d P0 = line3[j].position;
            Eigen::Vector3d P1 = line3[j + 1].position;
            Eigen::Vector3d v = P1 - P0;

            Eigen::Vector3d w0 = A - P0;

            double a = u.dot(u);
            double b = u.dot(v);
            double c = v.dot(v);
            double d = u.dot(w0);
            double e = v.dot(w0);

            double denom = a * c - b * b;

            if (std::abs(denom) < 1e-8)
                continue; // 楠炲疇顢戦幋鏍偓鈧崠?

            double sc = (b * e - c * d) / denom;
            double tc = (a * e - b * d) / denom;

            // tc 韫囧懘銆忛崷?segment 閸?
            if (tc < 0.0 || tc > 1.0)
                continue;

            Eigen::Vector3d Pc = A + sc * u;
            Eigen::Vector3d Qc = P0 + tc * v;

            double dist = (Pc - Qc).norm();

            if (dist < tol)
            {
                outPoint = Qc;
                outTangent = v.normalized();
                return true;
            }
        }

        return false;
    };

    // ============================
//  n == 2 
// ============================
    if (n == 2) {
        const auto& A0 = line1[0];
        const auto& A1 = line1[1];
        const auto& B0 = line2[0];
        const auto& B1 = line2[1];

 
        Eigen::Vector3d midA = 0.5 * (A0.position + A1.position);
        Eigen::Vector3d midB = 0.5 * (B0.position + B1.position);

 
        Eigen::Vector3d AB = midB - midA;
        if (AB.norm() < 1e-8)
            return false;
        AB.normalize();

        Eigen::Vector3d D, T;
        if (!intersectLineWithPolyline(midA, midB, D, T))
            return false;

        if (T.norm() < 1e-8)
            return false;
        T.normalize();

        Eigen::Vector3d normal_E = T - T.dot(AB) * AB;
        if (normal_E.norm() < 1e-8) {
            Eigen::Vector3d ref(0, 0, 1);
            if (std::abs(AB.dot(ref)) > 0.9)
                ref = Eigen::Vector3d(1, 0, 0);
            normal_E = ref - ref.dot(AB) * AB;
            if (normal_E.norm() < 1e-8)
                return false;
        }
        normal_E.normalize();

        Eigen::Vector3d midPoint = 0.5 * (midA + midB);

        planes.emplace_back(SlicePlane{ midPoint, normal_E });

        return !planes.empty();
    }

    // ============================
    // 娑撹鎯婇悳?
    // ============================
    for (size_t i = 1; i < n - 1; ++i) // 閸樼粯甯€妫ｆ牕鐔?
    {
        const auto& A = line1[i];
        const auto& B = line2[i];

        // ===== AB閺傜懓鎮?=====
        Eigen::Vector3d AB = B.position - A.position;
        if (AB.norm() < 1e-8)
            continue;
        AB.normalize();

        // The three node lines have already been generated with a shared
        // arc-length parameter.  Do not recover their correspondence by a
        // second line/polyline intersection: at a corner that intersection
        // lies on two segments and the old search chose one arbitrarily (or
        // rejected it because of a 1e-4 distance tolerance).  A centered
        // tangent provides the angle-bisector direction at the corner and is
        // stable for ordinary smooth samples as well.
        Eigen::Vector3d before = line3[i].position - line3[i - 1].position;
        Eigen::Vector3d after = line3[i + 1].position - line3[i].position;
        Eigen::Vector3d T = Eigen::Vector3d::Zero();
        if (before.norm() > 1e-8) before.normalize();
        if (after.norm() > 1e-8) after.normalize();
        if (before.norm() > 1e-8 && after.norm() > 1e-8) {
            T = before + after;
        } else {
            T = line3[i + 1].position - line3[i - 1].position;
        }
        if (T.norm() < 1e-8) continue;
        T.normalize();

        // ============================
        // 閺嬪嫰鈧姴閽╅棃銏＄《閸氭埊绱欓幎鏇炲濞夋洩绱?
        // ============================
        Eigen::Vector3d normal_E = T - T.dot(AB) * AB;

        if (normal_E.norm() < 1e-8)
        {
            // fallback
            Eigen::Vector3d ref(0, 0, 1);
            if (std::abs(AB.dot(ref)) > 0.9)
                ref = Eigen::Vector3d(1, 0, 0);

            normal_E = ref - ref.dot(AB) * AB;

            if (normal_E.norm() < 1e-8)
                continue;
        }

        normal_E.normalize();

        // ============================
        // 闂冭尙鐐曟潪顒婄礄鏉╃偟鐢婚幀褝绱?
        // ============================
        if (prevNormal.norm() > 1e-8 &&
            normal_E.dot(prevNormal) < 0.0)
        {
            normal_E = -normal_E;
        }

        prevNormal = normal_E;

        // ============================
        // 楠炴娊娼伴悙鐧哥礄AB娑擃厾鍋ｉ敍?
        // ============================
        Eigen::Vector3d mid = 0.5 * (A.position + B.position);

        planes.emplace_back(SlicePlane{ mid, normal_E });
    }

    return !planes.empty();
}

// [ALGORITHM] Calculate STL intersection with arbitrary plane
// Input: modelData - STL model
//        planeCenter - plane center point
//        planeNormal - plane normal vector
//        referenceZ - reference Z value (for Z-plane optimization)
// Output: vector<Vector3d> - intersection points
std::vector<Eigen::Vector3d> UniformSlicer::calculateSTLIntersectionWithPlane(
    const ModelData& modelData,
    const Eigen::Vector3d& planeCenter,
    const Eigen::Vector3d& planeNormal,
    double referenceZ)
{
    (void)referenceZ;
    const auto contours = traceContoursFromIntersections(modelData, SlicePlane{planeCenter, planeNormal});
    std::vector<Eigen::Vector3d> points;
    if (!contours.empty()) {
        for (const auto& point : contours.front().points) points.push_back(point.position);
    }
    return points;
}


//娴犮儰绗傛稉鍝勭杽闂勫懏鎼锋担婊堚偓鏄忕帆
// 娑撳娼伴弰顖涜閺屾捇鍎撮崚鍡涒偓鏄忕帆
// [ALGORITHM] Generate reference lines from boundaries
// Input: boundaries - internal boundaries
//        upperBoundary, lowerBoundary - boundary data
// Output: vector<vector<Vector3d>> - reference lines
// 閵嗘劕鍑￠崚鐘绘珟閵嗘吇enerateReferenceLines 閸?calculateQuaternions 閸戣姤鏆?
// 閼卞矁鐭楁潪顒傂╅敍姘崇箹娴滄稑濮涢懗鐣屽箛閸︺劎鏁?TrajectoryGenerator 鐠愮喕鐭?
// UniformSlicer 閸欘亣绀嬬拹锝呭殤娴ｆ洘鐪版禍銈忕礉鐠侊紕鐣绘担宥囩枂娣団剝浼呴敍鍧昻ternalBoundaries閿?

// [NEW] Apply transformation parameters to vertices around center point (static helper function)
std::vector<Eigen::Vector3d> UniformSlicer::applyTransformationToVertices(
    const std::vector<Eigen::Vector3d>& vertices,
    const Eigen::Vector3d& center,
    double tx, double ty, double tz,
    double rx, double ry, double rz)
{
    std::vector<Eigen::Vector3d> transformedVertices;
    transformedVertices.reserve(vertices.size());
    
    // 婵″倹鐏夊▽鈩冩箒閸欐ɑ宕查崣鍌涙殶閿涘瞼娲块幒銉ㄧ箲閸ョ偛甯慨瀣€婇悙?
    if (tx == 0.0 && ty == 0.0 && tz == 0.0 && rx == 0.0 && ry == 0.0 && rz == 0.0) {
        return vertices;
    }
    
    // 鐏忓棜顫楁惔锕佹祮閹诡澀璐熷褍瀹?
    double rx_rad = rx * M_PI / 180.0;
    double ry_rad = ry * M_PI / 180.0;
    double rz_rad = rz * M_PI / 180.0;
    
    // 閸掓稑缂撻弮瀣祮閻晠妯€閿涘牊瀵?Z-Y-X 妞ゅ搫绨敍灞藉祮 Yaw-Pitch-Roll閿?
    Eigen::Matrix3d Rx, Ry, Rz;
    
    // X鏉炲瓨妫嗘潪顒傜叐闂?(Roll)
    Rx << 1, 0, 0,
          0, std::cos(rx_rad), -std::sin(rx_rad),
          0, std::sin(rx_rad), std::cos(rx_rad);
    
    // Y鏉炲瓨妫嗘潪顒傜叐闂?(Pitch)
    Ry << std::cos(ry_rad), 0, std::sin(ry_rad),
          0, 1, 0,
          -std::sin(ry_rad), 0, std::cos(ry_rad);
    
    // Z鏉炲瓨妫嗘潪顒傜叐闂?(Yaw)
    Rz << std::cos(rz_rad), -std::sin(rz_rad), 0,
          std::sin(rz_rad), std::cos(rz_rad), 0,
          0, 0, 1;
    
    // 缂佸嫬鎮庨弮瀣祮閻晠妯€閿涙瓓 = Rz * Ry * Rx
    Eigen::Matrix3d R = Rz * Ry * Rx;
    
    // 閻劍鍩涢幐鍥х暰閻ㄥ嫬閽╃粔璇叉倻闁?
    Eigen::Vector3d userTranslation(tx, ty, tz);
    
    // 閵嗘劕鍙ч柨顔绘叏婢跺秲鈧垹绮稉顓炵妇閻愬湱娈戦崣妯诲床閿?
    // 1. 鐏忓棝銆婇悙鐟伴挬缁夎鍩屾禒銉よ厬韫囧啰鍋ｆ稉鍝勫斧閻愬湱娈戦崸鎰垼缁?
    // 2. 鎼存梻鏁ら弮瀣祮
    // 3. 楠炲磭些閸ョ偛甯弶銉ф畱娴ｅ秶鐤?
    // 4. 閸愬秴绨查悽銊ф暏閹撮娈戦獮宕囆╅崣鍌涙殶
    for (const auto& vertex : vertices) {
        // 濮濄儵顎?閿涙艾閽╃粔璇插煂娑擃厼绺鹃悙鐟版綏閺嶅洨閮?(v - center)
        Eigen::Vector3d centeredVertex = vertex - center;
        
        // 濮濄儵顎?閿涙艾绨查悽銊︽鏉?R * (v - center)
        Eigen::Vector3d rotatedVertex = R * centeredVertex;
        
        // 濮濄儵顎?閿涙艾閽╃粔璇叉礀閸樼喍缍呯純?R * (v - center) + center
        Eigen::Vector3d restoredVertex = rotatedVertex + center;
        
        // 濮濄儵顎?閿涙艾绨查悽銊ф暏閹村嘲閽╃粔?R * (v - center) + center + userTranslation
        Eigen::Vector3d finalVertex = restoredVertex + userTranslation;
        
        transformedVertices.push_back(finalVertex);
    }
    
    return transformedVertices;
}

// 閵嗘劖鏌婃晶鐐偓鎴炴纯閺傛澘宕熸稉顏勫瀼楠炴娊娼伴惃鍕槀鐎甸潻绱欓弽瑙勫祦閸欐ɑ宕查崥搴ｆ畱娴溿倗鍋ｇ拋锛勭暬閸栧懎娲块惄?1.2閿?
bool UniformSlicer::updateSingleCuttingPlaneSize(CuttingPlane& plane, 
                                                const ModelData& modelData,
                                                const Eigen::Vector3d& originalCenter,
                                                 bool& updata)
{
    // 鐠侊紕鐣婚崣妯诲床閸氬海娈戦崚鍥ч挬闂堛垺纭堕崥鎴﹀櫤閸滃奔鑵戣箛?
    double rx_rad = plane.rotation_x * M_PI / 180.0;
    double ry_rad = plane.rotation_y * M_PI / 180.0;
    double rz_rad = plane.rotation_z * M_PI / 180.0;
    
    // 閸掓稑缂撻弮瀣祮閻晠妯€
    Eigen::Matrix3d Rx, Ry, Rz;
    Rx << 1, 0, 0,
          0, std::cos(rx_rad), -std::sin(rx_rad),
          0, std::sin(rx_rad), std::cos(rx_rad);
    Ry << std::cos(ry_rad), 0, std::sin(ry_rad),
          0, 1, 0,
          -std::sin(ry_rad), 0, std::cos(ry_rad);
    Rz << std::cos(rz_rad), -std::sin(rz_rad), 0,
          std::sin(rz_rad), std::cos(rz_rad), 0,
          0, 0, 1;
    Eigen::Matrix3d R = Rz * Ry * Rx;
    
    // 閸欐ɑ宕查崥搴ｆ畱濞夋洖鎮滈柌蹇撴嫲娑擃厼绺鹃悙?
    // Preserve adaptive per-plane normal, avoid forcing +Z.
    Eigen::Vector3d originalNormal(0, 0, 1);
    if (plane.normal.norm() > 1e-12) {
        originalNormal = plane.normal.normalized();
    }
    Eigen::Vector3d transformedNormal = R * originalNormal;
    // 閵嗘劒鎱ㄦ径宥冣偓鎲坮ansformedCenter鎼存棁顕氶弰顖氬斧婵獚enter閸旂姳绗傞獮宕囆╅敍灞肩瑢convertToCuttingPlanes娣囨繃瀵旀稉鈧懛?
    Eigen::Vector3d transformedCenter = originalCenter + Eigen::Vector3d(
        plane.translation_x, plane.translation_y, plane.translation_z);
    
    // 閻劌褰夐幑銏犳倵閻ㄥ嫬鍨忛獮鎶芥桨娑撳洞TL濮瑰倷姘?
    std::vector<Eigen::Vector3d> intersectionPoints = calculateSTLIntersectionWithPlane(
        modelData, transformedCenter, transformedNormal, transformedCenter.z());
    
    if (intersectionPoints.empty()) {
        return false;  // 濞屸剝婀佹禍銈囧仯閿涘奔绻氶幐浣稿斧閺堝鏄傜€?
    }
    
    // 閸︺劌鍨忛獮鎶芥桨閻ㄥ嫬鐪柈銊ユ綏閺嶅洨閮存稉顓☆吀缁犳瀵橀崶瀵告磪
    Eigen::Vector3d localX, localY;
    if (std::abs(transformedNormal.z()) > 0.9) {
        localX = Eigen::Vector3d(1, 0, 0) - transformedNormal * transformedNormal.dot(Eigen::Vector3d(1, 0, 0));
    } else {
        localX = Eigen::Vector3d(0, 0, 1) - transformedNormal * transformedNormal.dot(Eigen::Vector3d(0, 0, 1));
    }
    localX.normalize();
    localY = transformedNormal.cross(localX).normalized();
    
    double minLocalX = std::numeric_limits<double>::max();
    double maxLocalX = std::numeric_limits<double>::lowest();
    double minLocalY = std::numeric_limits<double>::max();
    double maxLocalY = std::numeric_limits<double>::lowest();
    
    for (const auto& point : intersectionPoints) {
        Eigen::Vector3d relativePoint = point - transformedCenter;
        double localXCoord = relativePoint.dot(localX);
        double localYCoord = relativePoint.dot(localY);
        
        minLocalX = std::min(minLocalX, localXCoord);
        maxLocalX = std::max(maxLocalX, localXCoord);
        minLocalY = std::min(minLocalY, localYCoord);
        maxLocalY = std::max(maxLocalY, localYCoord);
    }
    
    // 閵嗘劕鍙ч柨顔衡偓鎴炲潑閸?0%鏉堢绐涢敍?.2閸婂稄绱?
    double xRange = maxLocalX - minLocalX;
    double yRange = maxLocalY - minLocalY;
    double xPad = xRange * 0.1;  // 10%鏉堢绐涢敍灞锯偓璇插彙1.2閸?
    double yPad = yRange * 0.1;
    
    minLocalX -= xPad;
    maxLocalX += xPad;
    minLocalY -= yPad;
    maxLocalY += yPad;
    
    // 閸︺劌鍨忛獮鎶芥桨閻ㄥ嫬鐪柈銊ユ綏閺嶅洨閮存稉顓炲灡瀵ゅ搫娲撴稉顏堛€婇悙?
    plane.vertices.resize(4);
    
    // 閸︺劌褰夐幑銏犳倵閻ㄥ嫬娼楅弽鍥╅兇娑擃厼鍨卞娲€婇悙?
    std::vector<Eigen::Vector3d> localVertices(4);
    localVertices[0] = transformedCenter + localX * minLocalX + localY * minLocalY;
    localVertices[1] = transformedCenter + localX * maxLocalX + localY * minLocalY;
    localVertices[2] = transformedCenter + localX * maxLocalX + localY * maxLocalY;
    localVertices[3] = transformedCenter + localX * minLocalX + localY * maxLocalY;
    
    // 閸欏秴鎮滈崣妯诲床閸掓澘甯慨瀣綏閺嶅洨閮撮敍鍫ｇ箹閺嶉攱瑕嗛弻鎾存鎼存梻鏁ゅ锝呮倻閸欐ɑ宕茬亸杈ㄦЦ濮濓絿鈥橀惃鍕秴缂冾噯绱?
    Eigen::Matrix3d R_inv = R.transpose();
    Eigen::Vector3d userTranslation(plane.translation_x, plane.translation_y, plane.translation_z);
    
    for (size_t j = 0; j < 4; ++j) {
        Eigen::Vector3d vertex = localVertices[j] - userTranslation;
        Eigen::Vector3d centered = vertex - originalCenter;
        Eigen::Vector3d rotatedBack = R_inv * centered;
        plane.vertices[j] = rotatedBack + originalCenter;
    }
    updata = true;
    return true;
}

}

