#include <UniformSlicer/UniformSlicer.h>
#include <GeneralData/GeneralData.h>
#include <STLProcessor/STLProcessor.h>
#include <BoundaryDetector/BoundaryDetector.h>
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/imodel.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <map>
#include <limits>
#include <iostream>
#include <QDebug>
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
        if (params.mode == SlicingMode::Parallel) {
            if (upperBoundary.points.empty() || lowerBoundary.points.empty() ||
                !std::isfinite(params.minStep) || params.minStep <= 0.0 ||
                !std::isfinite(zMin) || !std::isfinite(zMax)) {
                qWarning() << "UniformSlicer: invalid parallel inputs: upperPoints="
                           << upperBoundary.points.size() << "lowerPoints=" << lowerBoundary.points.size()
                           << "minStep=" << params.minStep << "zMin=" << zMin << "zMax=" << zMax;
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
            const int layerCount = static_cast<int>(std::floor(height / params.minStep));
            if (!std::isfinite(height) || height <= 0.0 || layerCount <= 0) {
                qWarning() << "UniformSlicer: invalid parallel range: upperMinZ=" << upperMinZ
                           << "lowerMaxZ=" << lowerMaxZ << "height=" << height
                           << "minStep=" << params.minStep << "layerCount=" << layerCount;
                return false;
            }

            // The two boundary contours are consumed directly by trajectory
            // generation. Only create the strictly internal planes here;
            // otherwise the top/bottom boundary would be represented twice.
            slicingLocations.reserve(static_cast<size_t>(std::max(0, layerCount - 1)));
            for (int i = 1; i < layerCount; ++i) {
                const double ratio = static_cast<double>(i) / static_cast<double>(layerCount);
                SlicePlane plane;
                plane.point = Eigen::Vector3d(0.0, 0.0, lowerMaxZ + height * ratio);
                plane.normal = Eigen::Vector3d::UnitZ();
                slicingLocations.push_back(plane);
            }
        }
        else if (!buildSlicePlanes(slicingnode, slicingLocations)) {
            qWarning() << "UniformSlicer: buildSlicePlanes failed; slicing nodes=" << slicingnode.size();
            return false;
        }
        // Generate internal boundary lines
        sliceResult.internalBoundaries = generateInternalBoundaries(modelData, params, zMin, zMax, slicingLocations);

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
        return params.mode == SlicingMode::Parallel
            ? (std::isfinite(zMin) && std::isfinite(zMax) && !upperBoundary.points.empty() && !lowerBoundary.points.empty())
            : !sliceResult.internalBoundaries.empty();
    }
    catch (const std::exception& e) {
        qWarning() << "UniformSlicer: exception:" << e.what();
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
            continue;
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
    if (sliceResult.internalBoundaries.empty()) {
        return true;  // 濞屸剝婀佹潏鍦櫕閺佺増宓侀敍宀冪箲閸ョ偞鍨氶崝?
    }

    if (!modelData) {
        return true;  // 濞屸剝婀佸Ο鈥崇€烽弫鐗堝祦閿涘本妫ゅ▔鏇㈠櫢閺傜増鐪版禍銈忕礉鏉╂柨娲栭幋鎰閿涘牅绻氶幐浣稿斧鏉堝湱鏅敍?
    }

    // 闁秴宸诲В蹇庨嚋鏉堝湱鏅仦鍌︾礉濡偓閺屻儲妲搁崥锔芥箒閸欐ɑ宕查棁鈧憰浣哥安閻?
    for (size_t i = 0; i < sliceResult.internalBoundaries.size(); ++i) {
        // 濡偓閺屻儲妲搁崥锔芥箒閺冄呮畱閸欐ɑ宕查崣鍌涙殶闂団偓鐟曚礁绨查悽?
        bool hasTransformation = false;
        double tx = 0.0, ty = 0.0, tz = 0.0;
        double rx = 0.0, ry = 0.0, rz = 0.0;
        Eigen::Vector3d originalCenter(0, 0, sliceResult.internalBoundaries[i].z_level);

        if (oldCuttingPlanes && i < oldCuttingPlanes->size()) {
            const auto& oldPlane = (*oldCuttingPlanes)[i];
            tx = oldPlane.translation_x;
            ty = oldPlane.translation_y;
            tz = oldPlane.translation_z;
            rx = oldPlane.rotation_x;
            ry = oldPlane.rotation_y;
            rz = oldPlane.rotation_z;

            hasTransformation = (tx != 0.0 || ty != 0.0 || tz != 0.0 ||
                rx != 0.0 || ry != 0.0 || rz != 0.0);

            // 閼惧嘲褰囬崢鐔奉潗娑擃厼绺鹃悙?
            originalCenter = oldPlane.center;
        }
        else if (!sliceResult.internalBoundaries[i].points.empty()) {
            // 婵″倹鐏夊▽鈩冩箒閺冄冨瀼楠炴娊娼伴敍灞肩矤瑜版挸澧犳潏鍦櫕鐠侊紕鐣绘稉顓炵妇閻?
            Eigen::Vector3d centerSum(0, 0, 0);
            for (const auto& point : sliceResult.internalBoundaries[i].points) {
                centerSum += point.position;
            }
            originalCenter = centerSum / static_cast<double>(sliceResult.internalBoundaries[i].points.size());
        }

        // 婵″倹鐏夐張澶婂綁閹诡澁绱濋悽銊ュ綁閹广垹鎮楅惃鍕瀼楠炴娊娼伴柌宥嗘煀濮瑰倷姘﹂獮鑸垫纯閺傛媽绔熼悾?
        if (hasTransformation) {
            // 鐠侊紕鐣婚崣妯诲床閸氬海娈戦崚鍥ч挬闂堛垺纭堕崥鎴﹀櫤閸滃奔鑵戣箛?
            double rx_rad = rx * M_PI / 180.0;
            double ry_rad = ry * M_PI / 180.0;
            double rz_rad = rz * M_PI / 180.0;

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
            // Keep per-layer adaptive normal instead of forcing global +Z.
            Eigen::Vector3d originalNormal(0, 0, 1);
            if (oldCuttingPlanes && i < oldCuttingPlanes->size()) {
                const Eigen::Vector3d candidate = (*oldCuttingPlanes)[i].normal;
                if (candidate.norm() > 1e-12) {
                    originalNormal = candidate.normalized();
                }
            }
            Eigen::Vector3d transformedNormal = R * originalNormal;
            Eigen::Vector3d transformedCenter = originalCenter + Eigen::Vector3d(tx, ty, tz);

            // 娴ｈ法鏁?traceContoursFromIntersections 閻㈢喐鍨氭潏鍦櫕缁惧尅绱欓崪?generateInternalBoundaries 娑撯偓閺嶅嚖绱?
            SlicePlane transformedPlane;
            transformedPlane.point = transformedCenter;
            transformedPlane.normal = transformedNormal;
            std::vector<BoundaryLine> contours = traceContoursFromIntersections(
                *modelData, transformedPlane);

            if (!contours.empty()) {
                // 閸欘亙绻氶悾娆愭付闂€璺ㄦ畱鏉烆喖绮ㄧ痪鍖＄礄娑撴槒鐤嗗鎿勭礆閿涘苯鎷?generateInternalBoundaries 娣囨繃瀵旀稉鈧懛?
                auto longest_contour_it = std::max_element(contours.begin(), contours.end(),
                    [](const BoundaryLine& a, const BoundaryLine& b) {
                        return a.points.size() < b.points.size();
                    });

                // 閻╁瓨甯寸憰鍡欐磰 sliceResult 娑擃厼顕惔鏂跨湴閻ㄥ嫯绔熼悾灞炬殶閹?
                sliceResult.internalBoundaries[i] = *longest_contour_it;
            }
        }
    }

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
    const double tolerance = 1e-6;

    // ============================
    // 1. Plane definition
    // ============================
    Eigen::Vector3d normal = plane.normal.normalized();
    double d = normal.dot(plane.point);

    // ============================
    // 2. Intersection structure
    // ============================
    struct IntersectionInfo {
        Eigen::Vector3d point;
        size_t triangleIndex;
        int edgeIndex;
        Eigen::Vector3d edgeStart;
        Eigen::Vector3d edgeEnd;
    };

    std::vector<IntersectionInfo> intersections;

    // ============================
    // 3. Iterate triangles and compute intersections
    // ============================
    for (size_t triangleIndex = 0;
        triangleIndex < STLProcessor::getTriangleCount(modelData.mainModelData_);
        ++triangleIndex)
    {
        Eigen::Vector3d vertices[3];
        STLProcessor::getTriangleVertices(modelData.mainModelData_, triangleIndex, vertices);

        for (int edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
        {
            const Eigen::Vector3d& v1 = vertices[edgeIdx];
            const Eigen::Vector3d& v2 = vertices[(edgeIdx + 1) % 3];

            double dist1 = normal.dot(v1) - d;
            double dist2 = normal.dot(v2) - d;

            // Regular intersection
            if (dist1 * dist2 < 0.0)
            {
                double t = dist1 / (dist1 - dist2);

                if (t >= 0.0 && t <= 1.0)
                {
                    IntersectionInfo info;
                    info.point = v1 + t * (v2 - v1);
                    info.triangleIndex = triangleIndex;
                    info.edgeIndex = edgeIdx;
                    info.edgeStart = v1;
                    info.edgeEnd = v2;
                    intersections.push_back(info);
                }
            }
            // Coplanar edge case
            else if (std::abs(dist1) < tolerance && std::abs(dist2) < tolerance)
            {
                IntersectionInfo info1, info2;

                info1.point = v1;
                info1.triangleIndex = triangleIndex;
                info1.edgeIndex = edgeIdx;
                info1.edgeStart = v1;
                info1.edgeEnd = v2;

                info2.point = v2;
                info2.triangleIndex = triangleIndex;
                info2.edgeIndex = edgeIdx;
                info2.edgeStart = v1;
                info2.edgeEnd = v2;

                intersections.push_back(info1);
                intersections.push_back(info2);
            }
        }
    }

    if (intersections.empty())
        return contours;

    // ============================
    // 4. Mark used intersections
    // ============================
    std::vector<bool> used(intersections.size(), false);

    // ============================
    // 5. Trace closed contours
    // ============================
    for (size_t startIdx = 0; startIdx < intersections.size(); ++startIdx)
    {
        if (used[startIdx]) continue;

        std::vector<Eigen::Vector3d> contourPoints;
        size_t currentIdx = startIdx;
        Eigen::Vector3d startPoint = intersections[startIdx].point;

        while (true)
        {
            used[currentIdx] = true;
            contourPoints.push_back(intersections[currentIdx].point);

            size_t nextIdx = intersections.size();
            const auto& currentInfo = intersections[currentIdx];

            // Method 1: Find adjacent intersection on the same triangle
            for (size_t i = 0; i < intersections.size(); ++i)
            {
                if (used[i] || i == currentIdx) continue;

                if (intersections[i].triangleIndex == currentInfo.triangleIndex &&
                    intersections[i].edgeIndex != currentInfo.edgeIndex)
                {
                    nextIdx = i;
                    break;
                }
            }

            // Method 2: Find shared edge with another triangle
            if (nextIdx == intersections.size())
            {
                for (size_t i = 0; i < intersections.size(); ++i)
                {
                    if (used[i] || i == currentIdx) continue;

                    const auto& candidate = intersections[i];

                    bool sharesEdge =
                        ((candidate.edgeStart - currentInfo.edgeStart).norm() < tolerance &&
                            (candidate.edgeEnd - currentInfo.edgeEnd).norm() < tolerance) ||
                        ((candidate.edgeStart - currentInfo.edgeEnd).norm() < tolerance &&
                            (candidate.edgeEnd - currentInfo.edgeStart).norm() < tolerance);

                    if (sharesEdge && candidate.triangleIndex != currentInfo.triangleIndex)
                    {
                        nextIdx = i;
                        break;
                    }
                }
            }

            // Method 3: Fallback to nearest point
            if (nextIdx == intersections.size())
            {
                double minDist = std::numeric_limits<double>::max();

                for (size_t i = 0; i < intersections.size(); ++i)
                {
                    if (used[i] || i == currentIdx) continue;

                    double dist = (intersections[i].point - currentInfo.point).norm();
                    if (dist < minDist)
                    {
                        minDist = dist;
                        nextIdx = i;
                    }
                }
            }

            // Termination condition: no next point found
            if (nextIdx >= intersections.size())
                break;

            // Close loop if returning to start point
            double distToStart = (intersections[nextIdx].point - startPoint).norm();
            if (distToStart < tolerance && contourPoints.size() >= 2)
            {
                break;
            }

            currentIdx = nextIdx;

            // Safety break to prevent infinite loop
            if (contourPoints.size() > intersections.size())
                break;
        }

        // ============================
        // 6. Build valid BoundaryLine
        // ============================
        if (contourPoints.size() >= 3)
        {
            BoundaryLine boundary;
            boundary.normal = normal;

            // Compute contour center
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (const auto& p : contourPoints)
                center += p;
            center /= contourPoints.size();
            boundary.center = center;

            // Compute slice height (Z-level)
            boundary.z_level = normal.dot(center);

            // Convert to BasicPoint with angle for ordering
            for (size_t i = 0; i < contourPoints.size(); ++i)
            {
                BasicPoint bp;
                bp.position = contourPoints[i];
                bp.index = i;

                Eigen::Vector3d vec = contourPoints[i] - center;
                bp.angle = std::atan2(vec.y(), vec.x());

                boundary.points.push_back(bp);
            }

            contours.push_back(boundary);
        }
    }

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
    std::vector<Eigen::Vector3d> intersectionPoints;
    const double tolerance = 1e-6;
    
    // 绾喕绻氬▔鏇炴倻闁插繐缍婃稉鈧崠?
    Eigen::Vector3d normal = planeNormal.normalized();
    
    // 楠炴娊娼伴弬鍦柤閿涙 璺?(p - c) = 0閿涘苯宓?n 璺?p = d
    double d = normal.dot(planeCenter);

    // 闁秴宸婚幍鈧張澶夌瑏鐟欐帒鑸伴敍灞绢梾閺屻儰绗岄獮鎶芥桨閻ㄥ嫮娴夋禍銈嗗剰閸?
    for (size_t triangleIndex = 0; triangleIndex < STLProcessor::getTriangleCount(modelData.mainModelData_); ++triangleIndex) {
        Eigen::Vector3d vertices[3];
        STLProcessor::getTriangleVertices(modelData.mainModelData_, triangleIndex, vertices);
        std::vector<Eigen::Vector3d> edgeIntersections;

        // 濡偓閺屻儰绗佺憴鎺戣埌閻ㄥ嫭鐦￠弶陇绔熼弰顖氭儊娑撳骸閽╅棃銏㈡祲娴?
        for (int i = 0; i < 3; ++i) {
            const Eigen::Vector3d& v1 = vertices[i];
            const Eigen::Vector3d& v2 = vertices[(i + 1) % 3];

            // 鐠侊紕鐣绘い鍓佸仯閸掓澘閽╅棃銏㈡畱閺堝鎮滅捄婵堫瀲
            double dist1 = normal.dot(v1) - d;
            double dist2 = normal.dot(v2) - d;

            // 婵″倹鐏夋潏鍦畱娑撱倓閲滄い鍓佸仯閸︺劌閽╅棃銏㈡畱娑撱倓鏅堕敍鍫㈩儊閸欒渹绗夐崥宀嬬礆
            if ((dist1 * dist2) < 0.0) {
                // 缁炬寧顔屾稉搴￠挬闂堛垻娴夋禍銈忕礉鐠侊紕鐣绘禍銈囧仯
                double t = dist1 / (dist1 - dist2);
                if (t >= 0.0 && t <= 1.0) {
                    Eigen::Vector3d intersection = v1 + t * (v2 - v1);
                    edgeIntersections.push_back(intersection);
                }
            }
            else if (std::abs(dist1) < tolerance && std::abs(dist2) < tolerance) {
                // 婵″倹鐏夋潏閫涚瑢楠炴娊娼伴崗閬嶆桨
                edgeIntersections.push_back(v1);
                edgeIntersections.push_back(v2);
            }
        }

        // 鐏忓棙婀侀弫鍫㈡畱娴溿倗鍋ｅǎ璇插閸掓壆绮ㄩ弸婊冨灙鐞涖劋鑵?
        for (const auto& point : edgeIntersections) {
            intersectionPoints.push_back(point);
        }
    }

    // 閸樺鍣?
    if (intersectionPoints.size() > 1) {
        std::sort(intersectionPoints.begin(), intersectionPoints.end(),
            [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                if (std::abs(a.x() - b.x()) > 1e-6) return a.x() < b.x();
                if (std::abs(a.y() - b.y()) > 1e-6) return a.y() < b.y();
                return a.z() < b.z();
            });

        auto end = std::unique(intersectionPoints.begin(), intersectionPoints.end(),
            [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                return (a - b).norm() < 1e-6;
            });
        intersectionPoints.erase(end, intersectionPoints.end());
    }

    return intersectionPoints;
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

bool UniformSlicer::initializeRenderer(SlicingRenderData& renderData)
{
    // 鏉╂瑩鍣锋稉宥呭晙娓氭繆绂?scene閿涘牆娲滄稉鍝勫嚒缂佸繐顦婚柈銊ь吀閻炲棴绱?
    renderData.needsUpdate = true;

    return true;
}



// [ALGORITHM] Hue to RGB conversion helper for generating vivid colors
// Input: h - hue value (0-1)
// Output: Vector3f - RGB color
Eigen::Vector3f UniformSlicer::hue2rgb(float h) {
    float r = 0, g = 0, b = 0;
    int k = static_cast<int>(h * 6.0f);
    float f = h * 6.0f - k;
    switch (k % 6) {
    case 0: r = 1; g = f; b = 0; break;
    case 1: r = 1 - f; g = 1; b = 0; break;
    case 2: r = 0; g = 1; b = f; break;
    case 3: r = 0; g = 1 - f; b = 1; break;
    case 4: r = f; g = 0; b = 1; break;
    default: r = 1; g = 0; b = 1 - f; break;
    }
    return Eigen::Vector3f(r, g, b);
}


// 閸掓稑缂撻崚鍥╁楠炴娊娼板〒鍙夌厠濡€崇€烽敍鍫滅矤BurTrajPlan缁夋槒绻冮弶銉ф畱缁犳纭堕敍?
// 濞夈劍鍓伴敍姘崇箹娑擃亝鏌熷▔鏇氱矝閻掑爼娓剁憰浣界箲閸?imodel閿涘苯娲滄稉鍝勭暊閸?BurTrajPlan 娑擃叀顫﹂惄瀛樺复娴ｈ法鏁?
imodel_spointer UniformSlicer::createCuttingPlanesModel(
    SlicingRenderData& renderData,
    const std::vector<CuttingPlane>& cuttingPlanes,
    const SliceResult& sliceResult)
{
    if (cuttingPlanes.empty()) {
        return nullptr;
    }

    // 閼惧嘲褰囧鑼病閸掓稑缂撴總鐣屾畱濡€崇€?
    imodel_spointer cuttingPlanesModel = renderData.sliceModel;

    if (!cuttingPlanesModel) {
        return nullptr;
    }

    // Use persistent data buffer.
    MeshData& meshData = renderData.sliceData;
    meshData.m_vertices.clear();
    meshData.m_indices.clear();

    for (size_t i = 0; i < cuttingPlanes.size(); ++i) {
        const auto& plane = cuttingPlanes[i];

        if (plane.vertices.size() != 4) {
            continue;
        }

        float r, g, b;

        if (plane.isSelected) {
            r = 1.0f;
            g = 1.0f;
            b = 0.2f;
        }
        else {
            static const float colorTable[][3] = {
                {1.0f, 0.2f, 0.2f},
                {0.2f, 1.0f, 0.2f},
                {1.0f, 0.2f, 1.0f},
                {0.2f, 1.0f, 1.0f},
                {1.0f, 0.6f, 0.2f},
                {0.6f, 0.2f, 1.0f},
                {0.2f, 0.6f, 1.0f},
                {0.2f, 0.8f, 1.0f},
                {1.0f, 0.2f, 0.8f},
                {1.0f, 0.2f, 0.6f},
                {0.6f, 1.0f, 0.8f},
                {0.8f, 0.6f, 1.0f}
            };

            int colorIndex = i % 12;

            r = colorTable[colorIndex][0];
            g = colorTable[colorIndex][1];
            b = colorTable[colorIndex][2];
        }

        float alpha = 0.7f;

        // 妞ゅ墎鍋ｉ崣妯诲床
        std::vector<Eigen::Vector3d> transformedVertices =
            applyTransformationToVertices(
                plane.vertices,
                plane.center,
                plane.translation_x, plane.translation_y, plane.translation_z,
                plane.rotation_x, plane.rotation_y, plane.rotation_z
            );

        unsigned int baseIndex =
            static_cast<unsigned int>(meshData.m_vertices.size() / 7);

        for (int j = 0; j < 4; ++j) {
            const auto& v = transformedVertices[j];

            meshData.m_vertices.push_back((float)v.x());
            meshData.m_vertices.push_back((float)v.y());
            meshData.m_vertices.push_back((float)v.z());
            meshData.m_vertices.push_back(r);
            meshData.m_vertices.push_back(g);
            meshData.m_vertices.push_back(b);
            meshData.m_vertices.push_back(alpha);
        }

        // 娑撱倓閲滄稉澶庮潡瑜?
        meshData.m_indices.push_back(baseIndex + 0);
        meshData.m_indices.push_back(baseIndex + 1);
        meshData.m_indices.push_back(baseIndex + 2);

        meshData.m_indices.push_back(baseIndex + 0);
        meshData.m_indices.push_back(baseIndex + 2);
        meshData.m_indices.push_back(baseIndex + 3);
    }

    if (!meshData.m_vertices.empty() && !meshData.m_indices.empty()) {
        cuttingPlanesModel->import(meshData);
    }

    return cuttingPlanesModel;
}

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

bool UniformSlicer::draw(SlicingRenderData& renderData,
    const SlicingData& slicing) const
{
    // 1. 濡偓閺屻儱鍨忛悧鍥ㄦ殶閹?
    if (slicing.cuttingPlanes_.empty()) {
        return false;
    }

    // 2. 閸欘垵顫嗛幀褎甯堕崚?
    if (!renderData.isVisible) {
        return false;
    }

    // 3. 濡偓閺屻儲膩閸ㄥ妲搁崥锕€鐡ㄩ崷?
    if (!renderData.sliceModel) {
        return false;
    }

    // 4. 婵″倹鐏夐棁鈧憰浣规纯閺?閳?閸忓牆鍣径鍥ㄦ殶閹诡噯绱欓摶宥呮値 prepare 闁槒绶敍?
    if (renderData.needsUpdate) {
        createCuttingPlanesModel(
            renderData,
            slicing.cuttingPlanes_,
            slicing.currentSliceResult_
        );

        renderData.needsUpdate = false;
    }

    // 5. 濞撳弶鐓?
    if (!renderData.sliceData.m_vertices.empty()) {
        renderData.sliceModel->import(renderData.sliceData);
        renderData.sliceModel->draw();
        return true;
    }

    return false;
}
}

