#pragma once
#include <SlicingTrajPlan/BoundaryData.h>
#include <Eigen/Geometry>
#include <limits>
#include <cmath>
#include <algorithm>

namespace ModelTrajPlan {
// Circle fitting uses the polygon plane and radius residuals, not PCA axis
// equality (which also holds for squares). Coordinates are scale normalized.
inline bool fitBoundaryCircle(const BoundaryLine& boundary, Eigen::Vector3d& center) {
    if(boundary.points.size() < 8) return false;
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    for(const auto& p : boundary.points) origin += p.position;
    origin /= double(boundary.points.size());
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double scale = 0.0;
    for(size_t i = 0; i < boundary.points.size(); ++i) {
        const Eigen::Vector3d a = boundary.points[i].position - origin;
        const Eigen::Vector3d b = boundary.points[(i + 1) % boundary.points.size()].position - origin;
        normal += a.cross(b);
        scale = std::max(scale, a.norm());
    }
    if(scale <= 1e-10 || normal.norm() <= scale * scale * 1e-10) return false;
    normal.normalize();
    Eigen::Vector3d u = boundary.points.front().position - origin;
    u -= normal * normal.dot(u);
    if(u.norm() <= scale * 1e-10) return false;
    u.normalize();
    const Eigen::Vector3d v = normal.cross(u);
    Eigen::Matrix3d lhs = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for(const auto& p : boundary.points) {
        const Eigen::Vector3d d = (p.position - origin) / scale;
        if(std::abs(d.dot(normal)) > 1e-3) return false;
        const double x = d.dot(u), y = d.dot(v);
        const Eigen::Vector3d row(2*x, 2*y, 1);
        lhs += row * row.transpose();
        rhs += row * (x*x + y*y);
    }
    const auto solver = lhs.fullPivLu();
    if(!solver.isInvertible()) return false;
    const Eigen::Vector3d solution = solver.solve(rhs);
    const double r2 = solution.z() + solution.head<2>().squaredNorm();
    if(!solution.allFinite() || r2 <= 1e-12) return false;
    const double radius = scale * std::sqrt(r2);
    const Eigen::Vector3d fitted = origin + scale * (solution.x()*u + solution.y()*v);
    double sum = 0.0;
    for(const auto& p : boundary.points) {
        const double residual = std::abs((p.position - fitted).norm() - radius) / radius;
        if(residual > 0.01) return false;
        sum += residual * residual;
    }
    if(std::sqrt(sum / boundary.points.size()) > 0.005) return false;
    // Reject coarse polygons whose vertices happen to be on a circle.
    for(size_t i = 0; i < boundary.points.size(); ++i) {
        const Eigen::Vector3d midpoint = 0.5 * (boundary.points[i].position +
            boundary.points[(i + 1) % boundary.points.size()].position);
        if(std::abs((midpoint - fitted).norm() - radius) > 0.02 * radius) return false;
    }
    center = fitted;
    return true;
}

inline bool circularBoundaryPlane(const BoundaryLine& a, const BoundaryLine& b,
    const Eigen::Vector3d& ca, const Eigen::Vector3d& cb, Eigen::Vector3d& normal) {
    const Eigen::Vector3d axis = cb - ca;
    if(axis.norm() <= 1e-10) return false;
    double best = std::numeric_limits<double>::infinity();
    Eigen::Vector3d selected = Eigen::Vector3d::Zero();
    // Search each vertex against opposite segments, including segment interiors.
    const auto search = [&](const BoundaryLine& from, const BoundaryLine& to) {
        for(size_t i = 0; i < from.points.size(); ++i) for(size_t j = 0; j < to.points.size(); ++j) {
            const auto& p = from.points[i];
            const Eigen::Vector3d q = to.points[j].position;
            const Eigen::Vector3d edge = to.points[(j + 1) % to.points.size()].position - q;
            const double t = edge.squaredNorm() > 1e-20 ?
                std::max(0.0, std::min(1.0, (p.position - q).dot(edge) / edge.squaredNorm())) : 0.0;
            const double distance = (p.position - q - t*edge).squaredNorm();
            if(distance < best) { best = distance; selected = p.position; }
            const Eigen::Vector3d u = from.points[(i + 1) % from.points.size()].position - p.position;
            const Eigen::Vector3d r = p.position - q;
            const double aa = u.squaredNorm(), bb = u.dot(edge), cc = edge.squaredNorm();
            const double dd = u.dot(r), ee = edge.dot(r), determinant = aa*cc - bb*bb;
            if(determinant > 1e-12 * aa * cc) {
                const double s = (bb*ee - cc*dd) / determinant;
                const double tInterior = (aa*ee - bb*dd) / determinant;
                if(s >= 0 && s <= 1 && tInterior >= 0 && tInterior <= 1) {
                    const double interiorDistance = (r + s*u - tInterior*edge).squaredNorm();
                    if(interiorDistance < best) { best = interiorDistance; selected = p.position + s*u; }
                }
            }
        }
    };
    search(a, b); search(b, a);
    normal = axis.cross(selected - ca);
    if(normal.norm() <= 1e-10 * axis.norm()) {
        // A closest point on the center line cannot define a plane. Use the
        // farthest rim point from that line; no world-axis or PCA dependency.
        for(const auto* rim : { &a, &b }) for(const auto& p : rim->points) {
            const Eigen::Vector3d candidate = axis.cross(p.position - ca);
            if(candidate.squaredNorm() > normal.squaredNorm()) normal = candidate;
        }
    }
    if(normal.norm() <= 1e-10 * axis.norm()) return false;
    normal.normalize();
    return true;
}
}
