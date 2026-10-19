#include "TrajectoryExporter.h"
#include "GeometryUtils.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <algorithm>
#include <cmath>

namespace SprayTrajPlan
{
    namespace
    {
        Eigen::Vector3d closestPointOnTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& a,
            const Eigen::Vector3d& b, const Eigen::Vector3d& c)
        {
            const Eigen::Vector3d ab = b - a;
            const Eigen::Vector3d ac = c - a;
            const Eigen::Vector3d ap = point - a;
            const double d1 = ab.dot(ap);
            const double d2 = ac.dot(ap);
            if (d1 <= 0.0 && d2 <= 0.0) return a;

            const Eigen::Vector3d bp = point - b;
            const double d3 = ab.dot(bp);
            const double d4 = ac.dot(bp);
            if (d3 >= 0.0 && d4 <= d3) return b;

            const double vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) return a + ab * (d1 / (d1 - d3));

            const Eigen::Vector3d cp = point - c;
            const double d5 = ab.dot(cp);
            const double d6 = ac.dot(cp);
            if (d6 >= 0.0 && d5 <= d6) return c;

            const double vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) return a + ac * (d2 / (d2 - d6));

            const double va = d3 * d6 - d5 * d4;
            if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
                const Eigen::Vector3d bc = c - b;
                return b + bc * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
            }

            const double denominator = va + vb + vc;
            if (std::abs(denominator) < EPS) return a;
            const double inverse = 1.0 / denominator;
            return a + ab * (vb * inverse) + ac * (vc * inverse);
        }

        std::vector<Eigen::Vector3d> vertexNormals(const std::vector<Eigen::Vector3d>& vertices,
            const std::vector<Eigen::Vector3i>& faces)
        {
            std::vector<Eigen::Vector3d> normals(vertices.size(), Eigen::Vector3d::Zero());
            for (const auto& face : faces) {
                if (face.minCoeff() < 0 || face.maxCoeff() >= static_cast<int>(vertices.size())) continue;
                const Eigen::Vector3d areaNormal = (vertices[face[1]] - vertices[face[0]])
                    .cross(vertices[face[2]] - vertices[face[0]]);
                for (int corner = 0; corner < 3; ++corner) normals[face[corner]] += areaNormal;
            }
            for (auto& normal : normals) {
                if (normal.norm() > EPS) normal.normalize();
                else normal = Eigen::Vector3d::UnitZ();
            }
            return normals;
        }

        Eigen::Vector3d barycentricNormal(const Eigen::Vector3d& point, const Eigen::Vector3i& face,
            const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3d>& normals)
        {
            const Eigen::Vector3d& a = vertices[face[0]];
            const Eigen::Vector3d& b = vertices[face[1]];
            const Eigen::Vector3d& c = vertices[face[2]];
            const Eigen::Vector3d v0 = b - a, v1 = c - a, v2 = point - a;
            const double d00 = v0.dot(v0), d01 = v0.dot(v1), d11 = v1.dot(v1);
            const double d20 = v2.dot(v0), d21 = v2.dot(v1);
            const double denominator = d00 * d11 - d01 * d01;
            if (std::abs(denominator) <= EPS) return (b - a).cross(c - a).normalized();
            const double weightB = (d11 * d20 - d01 * d21) / denominator;
            const double weightC = (d00 * d21 - d01 * d20) / denominator;
            const Eigen::Vector3d interpolated = (1.0 - weightB - weightC) * normals[face[0]] +
                weightB * normals[face[1]] + weightC * normals[face[2]];
            return interpolated.norm() > EPS ? interpolated.normalized() : (b - a).cross(c - a).normalized();
        }

        std::vector<Eigen::Vector3d> smoothMeshNormalsAtPoints(const std::vector<Eigen::Vector3d>& points,
            const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& faces)
        {
            std::vector<Eigen::Vector3d> closest;
            std::vector<double> distances;
            std::vector<int> faceIds;
            GeometryUtils::closestPointsRobust(vertices, faces, points, closest, distances, faceIds);
            const auto normals = vertexNormals(vertices, faces);
            std::vector<Eigen::Vector3d> result(points.size(), Eigen::Vector3d::UnitZ());
            for (size_t i = 0; i < points.size() && i < closest.size() && i < faceIds.size(); ++i) {
                const int faceId = faceIds[i];
                if (faceId < 0 || faceId >= static_cast<int>(faces.size())) continue;
                result[i] = barycentricNormal(closest[i], faces[faceId], vertices, normals);
            }
            return result;
        }

        std::vector<Eigen::Vector3d> fitContinuousNormals(const std::vector<Eigen::Vector3d>& points,
            std::vector<Eigen::Vector3d> normals)
        {
            if (points.size() != normals.size() || normals.empty()) return normals;
            std::vector<double> lengths(points.size(), 0.0), positiveLengths;
            for (size_t i = 1; i < points.size(); ++i) {
                lengths[i] = (points[i] - points[i - 1]).norm();
                if (lengths[i] > EPS) positiveLengths.push_back(lengths[i]);
            }
            for (size_t i = 0; i < normals.size(); ++i) {
                if (normals[i].norm() <= EPS) normals[i] = i ? normals[i - 1] : Eigen::Vector3d::UnitZ();
                else normals[i].normalize();
                if (i && normals[i].dot(normals[i - 1]) < 0.0) normals[i] = -normals[i];
            }
            if (normals.size() < 3 || positiveLengths.empty()) return normals;
            std::nth_element(positiveLengths.begin(), positiveLengths.begin() + positiveLengths.size() / 2, positiveLengths.end());
            const double radius = 3.0 * positiveLengths[positiveLengths.size() / 2];
            if (radius <= EPS) return normals;
            std::vector<double> arclength(points.size(), 0.0);
            for (size_t i = 1; i < points.size(); ++i) arclength[i] = arclength[i - 1] + lengths[i];
            const Eigen::Vector3d first = normals.front(), last = normals.back();
            std::vector<Eigen::Vector3d> fitted(normals.size());
            for (size_t i = 0; i < normals.size(); ++i) {
                Eigen::Vector3d sum = Eigen::Vector3d::Zero();
                for (size_t j = 0; j < normals.size(); ++j) {
                    const double distance = arclength[j] - arclength[i];
                    if (std::abs(distance) > radius) continue;
                    sum += std::exp(-0.5 * std::pow(distance / std::max(radius * 0.5, EPS), 2.0)) * normals[j];
                }
                fitted[i] = sum.norm() > EPS ? sum.normalized() : normals[i];
                if (i && fitted[i].dot(fitted[i - 1]) < 0.0) fitted[i] = -fitted[i];
            }
            fitted.front() = first; fitted.back() = last;
            return fitted;
        }

        Eigen::Quaterniond orientationQuaternion(const Eigen::Vector3d& tangent, const Eigen::Vector3d& inward)
        {
            Eigen::Vector3d yAxis = tangent.norm() > EPS ? tangent.normalized() : Eigen::Vector3d::UnitY();
            Eigen::Vector3d zAxis = inward.norm() > EPS ? inward.normalized() : Eigen::Vector3d::UnitZ();
            Eigen::Vector3d xAxis = yAxis.cross(zAxis);
            if (xAxis.norm() <= EPS) xAxis = (std::abs(zAxis.x()) > 0.9 ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitX()).cross(zAxis);
            xAxis.normalize(); yAxis = zAxis.cross(xAxis).normalized();
            Eigen::Matrix3d rotation; rotation.col(0) = xAxis; rotation.col(1) = yAxis; rotation.col(2) = zAxis;
            return Eigen::Quaterniond(rotation).normalized();
        }

        std::vector<Eigen::Vector3d> slerpUnitVectors(Eigen::Vector3d first, Eigen::Vector3d last, size_t count)
        {
            std::vector<Eigen::Vector3d> result(count, Eigen::Vector3d::UnitZ());
            if (!count) return result;
            first = first.norm() > EPS ? first.normalized() : Eigen::Vector3d::UnitZ();
            last = last.norm() > EPS ? last.normalized() : first;
            double cosine = std::clamp(first.dot(last), -1.0, 1.0);
            if (cosine < 0.0) { last = -last; cosine = -cosine; }
            const double angle = std::acos(cosine), sine = std::sin(angle);
            for (size_t i = 0; i < count; ++i) {
                const double t = count > 1 ? static_cast<double>(i) / static_cast<double>(count - 1) : 0.0;
                Eigen::Vector3d value = angle <= 1.0e-5 ? (1.0 - t) * first + t * last :
                    (std::sin((1.0 - t) * angle) / sine) * first + (std::sin(t * angle) / sine) * last;
                result[i] = value.normalized();
            }
            return result;
        }

        double seamValue(const TrajectorySegment& segment, const char* name, double fallback)
        {
            if (!segment.geometry_meta) return fallback;
            const auto it = segment.geometry_meta->find(name);
            return it == segment.geometry_meta->end() ? fallback : it->second;
        }

        bool isArcPoint(const TrajectorySegment& segment)
        {
            return segment.note.find("Fillet") != std::string::npos ||
                segment.note.find("Spatial Arc") != std::string::npos ||
                segment.note.find("SEAM_BLEND_3D") != std::string::npos;
        }
    }

    bool TrajectoryExporter::exportTrajectoryCSV(
        const std::string& csv_path,
        const std::vector<TrajectorySegment>& trajectory,
        const TrajectoryMetadata& metadata)
    {
        std::ofstream file(csv_path);
        if (!file.is_open()) {
            return false;
        }

        // Write header
        file << "segment_id,point_id,x,y,z,spray_on,kind,region,note\n";

        // Write trajectory data
        for (size_t seg_id = 0; seg_id < trajectory.size(); ++seg_id) {
            const auto& seg = trajectory[seg_id];

            for (size_t pt_id = 0; pt_id < seg.xyz.size(); ++pt_id) {
                const auto& pt = seg.xyz[pt_id];

                file << seg_id << ","
                     << pt_id << ","
                     << std::fixed << std::setprecision(6)
                     << pt.x() << ","
                     << pt.y() << ","
                     << pt.z() << ","
                     << (seg.spray_on ? "1" : "0") << ","
                     << seg.kind << ","
                     << seg.region << ","
                     << seg.note << "\n";
            }
        }

        file.close();
        return true;
    }

    bool TrajectoryExporter::exportMetadataJSON(
        const std::string& json_path,
        const TrajectoryMetadata& metadata)
    {
        std::ofstream file(json_path);
        if (!file.is_open()) {
            return false;
        }

        // Simple JSON output (without using a JSON library)
        file << "{\n";
        file << "  \"mesh_path\": \"" << metadata.mesh_path << "\",\n";
        file << "  \"spacing\": " << metadata.spacing << ",\n";
        file << "  \"speed\": " << metadata.speed << ",\n";
        file << "  \"spray_distance\": " << metadata.spray_distance << ",\n";
        file << "  \"total_segments\": " << metadata.total_segments << ",\n";
        file << "  \"total_length\": " << metadata.total_length << "\n";
        file << "}\n";

        file.close();
        return true;
    }

    std::vector<ExportTrajectoryPoint> TrajectoryExporter::prepareExportTrajectory(
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const std::vector<TrajectorySegment>& trajectory,
        double speed, double spray_distance, double arc_speed_loss_pct)
    {
        std::vector<ExportTrajectoryPoint> result;
        std::vector<const TrajectorySegment*> segments;
        std::vector<size_t> localIndices;
        std::vector<Eigen::Vector3d> points;
        for (const auto& segment : trajectory) {
            if (segment.xyz.empty()) continue;
            for (size_t i = 0; i < segment.xyz.size(); ++i) {
                segments.push_back(&segment);
                localIndices.push_back(i);
                points.push_back(segment.xyz[i]);
            }
        }
        if (points.empty()) return {};

        // Mirror OptAeigenTest: face normals are observations only.  Build a
        // continuous vertex-normal field and fit it independently per path.
        const std::vector<Eigen::Vector3d> measuredNormals = smoothMeshNormalsAtPoints(points, mesh_vertices, mesh_faces);
        std::vector<std::pair<size_t, size_t>> ranges;
        for (size_t first = 0; first < points.size();) {
            size_t last = first + 1;
            while (last < points.size() && segments[last] == segments[first]) ++last;
            ranges.emplace_back(first, last); first = last;
        }
        std::vector<Eigen::Vector3d> outward(points.size(), Eigen::Vector3d::UnitZ());
        for (const auto& [first, last] : ranges) {
            std::vector<Eigen::Vector3d> localPoints(points.begin() + first, points.begin() + last);
            std::vector<Eigen::Vector3d> localNormals(measuredNormals.begin() + first, measuredNormals.begin() + last);
            const auto fitted = fitContinuousNormals(localPoints, std::move(localNormals));
            std::copy(fitted.begin(), fitted.end(), outward.begin() + first);
        }
        std::vector<Eigen::Vector3d> exportPoints(points.size());
        std::vector<double> standoffs(points.size());
        Eigen::Quaterniond previousOrientation = Eigen::Quaterniond::Identity();
        for (size_t i = 0; i < points.size(); ++i) {
            const bool seam = segments[i]->kind == "SEAM_TRANSITION";
            standoffs[i] = seam ? seamValue(*segments[i], "seam_process.height", spray_distance) : spray_distance;
            exportPoints[i] = points[i] + outward[i] * standoffs[i];
        }

        std::vector<Eigen::Vector3d> tangents(points.size(), Eigen::Vector3d::UnitX());
        std::vector<Eigen::Vector3d> inward(points.size());
        for (size_t i = 0; i < points.size(); ++i) inward[i] = -outward[i];
        for (const auto& [begin, end] : ranges) {
            if (end - begin > 1) {
                tangents[begin] = exportPoints[begin + 1] - exportPoints[begin];
                tangents[end - 1] = exportPoints[end - 1] - exportPoints[end - 2];
                for (size_t i = begin + 1; i + 1 < end; ++i) tangents[i] = exportPoints[i + 1] - exportPoints[i - 1];
                if (exportPoints[end - 1].z() - exportPoints[begin].z() < 0.0) {
                    for (size_t i = begin; i < end; ++i) tangents[i] = -tangents[i];
                }
            }
        }
        // A 3-D-only connector has no reliable surface normal.  Interpolate
        // its tool axis between the surrounding surface scan endpoints.
        for (size_t rangeIndex = 0; rangeIndex < ranges.size(); ++rangeIndex) {
            const auto [begin, end] = ranges[rangeIndex];
            if (segments[begin]->kind != "OVERTRAVEL") continue;
            size_t previous = rangeIndex, following = rangeIndex;
            while (previous > 0 && segments[ranges[--previous].first]->kind != "SURFACE_SCAN") {}
            while (++following < ranges.size() && segments[ranges[following].first]->kind != "SURFACE_SCAN") {}
            if (segments[ranges[previous].first]->kind != "SURFACE_SCAN" || following >= ranges.size()) continue;
            const size_t previousEnd = ranges[previous].second;
            const size_t followingBegin = ranges[following].first;
            const size_t split = begin + (end - begin + 1) / 2;
            const Eigen::Vector3d exit = end - begin > 1 ? exportPoints[begin + 1] - exportPoints[begin] : tangents[previousEnd - 1];
            const Eigen::Vector3d entry = end - begin > 1 ? exportPoints[end - 1] - exportPoints[end - 2] : tangents[followingBegin];
            for (size_t i = begin; i < split; ++i) tangents[i] = exit.norm() > EPS ? exit : tangents[previousEnd - 1];
            for (size_t i = split; i < end; ++i) tangents[i] = entry.norm() > EPS ? entry : tangents[followingBegin];
            const auto interpolated = slerpUnitVectors(inward[previousEnd - 1], inward[followingBegin], end - begin);
            std::copy(interpolated.begin(), interpolated.end(), inward.begin() + begin);
        }

        const double baseSpeed = std::max(speed, EPS);
        const double arcSpeed = baseSpeed * std::max(0.0, 1.0 - arc_speed_loss_pct / 100.0);
        std::vector<double> velocities(points.size());
        std::vector<double> timestamps(points.size(), 0.0);
        std::vector<double> accelerations(points.size(), 0.0);
        std::vector<int> pointTypes(points.size(), 0);
        for (size_t i = 0; i < points.size(); ++i) {
            velocities[i] = isArcPoint(*segments[i]) ? arcSpeed :
                std::max(seamValue(*segments[i], "seam_process.speed", baseSpeed), EPS);
            const bool first = i == 0 || segments[i] != segments[i - 1];
            const bool last = i + 1 == points.size() || segments[i] != segments[i + 1];
            pointTypes[i] = segments[i]->kind == "SURFACE_SCAN"
                ? ((first || last) ? 1 : 0) : 2;
            if (i > 0) {
                const double deltaTime = (exportPoints[i] - exportPoints[i - 1]).norm() / velocities[i];
                timestamps[i] = timestamps[i - 1] + deltaTime;
                if (deltaTime > EPS) accelerations[i] = (velocities[i] - velocities[i - 1]) / deltaTime;
            }
        }

        for (size_t i = 0; i < points.size(); ++i) {
            Eigen::Quaterniond orientation = orientationQuaternion(tangents[i], inward[i]);
            if (i > 0 && orientation.w() * previousOrientation.w() + orientation.vec().dot(previousOrientation.vec()) < 0.0)
                orientation.coeffs() = -orientation.coeffs();
            previousOrientation = orientation;
            ExportTrajectoryPoint point;
            point.position=exportPoints[i]; point.orientation=orientation;
            point.velocity=velocities[i]; point.acceleration=accelerations[i];
            point.time=timestamps[i]; point.sprayDistance=standoffs[i]; point.pointType=pointTypes[i];
            point.segmentIndex=static_cast<std::size_t>(segments[i]-trajectory.data()); point.sprayEnabled=segments[i]->spray_on;
            result.push_back(point);
        }
        return result;
    }

    bool TrajectoryExporter::exportPAQTrajectory(
        const std::string& paq_path,
        const std::vector<Eigen::Vector3d>& mesh_vertices,
        const std::vector<Eigen::Vector3i>& mesh_faces,
        const std::vector<TrajectorySegment>& trajectory,
        double speed,
        double spray_distance,
        double arc_speed_loss_pct)
    {
        std::ofstream file(paq_path);
        if(!file.is_open()) return false;
        for(const auto& point : prepareExportTrajectory(mesh_vertices, mesh_faces, trajectory, speed, spray_distance, arc_speed_loss_pct)) {
            file << std::fixed << std::setprecision(3)
                << point.position.x() << " " << point.position.y() << " " << point.position.z() << " "
                << std::setprecision(6) << point.orientation.w() << " " << point.orientation.x() << " " << point.orientation.y() << " " << point.orientation.z() << " "
                << point.velocity << " " << point.acceleration << " " << point.time << " " << point.pointType << " " << point.sprayDistance << "\n";
        }
        return bool(file);
    }


    bool TrajectoryExporter::exportMatrixFormat(
        const std::string& matrix_path,
        const std::vector<TrajectorySegment>& trajectory,
        double speed)
    {
        std::ofstream file(matrix_path);
        if (!file.is_open()) {
            return false;
        }

        // Matrix format: [X, Y, Z, Roll, Pitch, Yaw, Speed, SprayOn]
        for (const auto& seg : trajectory) {
            for (const auto& pt : seg.xyz) {
                file << std::fixed << std::setprecision(6)
                     << pt.x() << " "
                     << pt.y() << " "
                     << pt.z() << " "
                     << "0.0 0.0 0.0 "  // Roll, Pitch, Yaw (placeholder)
                     << speed << " "
                     << (seg.spray_on ? "1" : "0") << "\n";
            }
        }

        file.close();
        return true;
    }

    std::string TrajectoryExporter::formatPAQPoint(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& normal,
        bool spray_on,
        double speed)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << position.x() << " "
            << position.y() << " "
            << position.z() << " "
            << normal.x() << " "
            << normal.y() << " "
            << normal.z() << " "
            << speed << " "
            << (spray_on ? "1" : "0");
        return oss.str();
    }

    void TrajectoryExporter::computeStatistics(
        const std::vector<TrajectorySegment>& trajectory,
        double& total_length,
        int& total_points)
    {
        total_length = 0.0;
        total_points = 0;

        for (const auto& seg : trajectory) {
            total_points += static_cast<int>(seg.xyz.size());

            for (size_t i = 1; i < seg.xyz.size(); ++i) {
                total_length += (seg.xyz[i] - seg.xyz[i-1]).norm();
            }
        }
    }

} // namespace SprayTrajPlan
