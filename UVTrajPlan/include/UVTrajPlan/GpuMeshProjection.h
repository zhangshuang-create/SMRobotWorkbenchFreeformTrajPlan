#pragma once

#include <Eigen/Dense>
#include <vector>

namespace SprayTrajPlan
{
    // Exact closest-triangle projection through the optional CUDA BVH backend.
    // Returns false when CUDA is unavailable so callers can use the CPU BVH.
    bool projectClosestPointsCuda(
        const std::vector<Eigen::Vector3d>& vertices,
        const std::vector<Eigen::Vector3i>& faces,
        const std::vector<Eigen::Vector3d>& queries,
        std::vector<Eigen::Vector3d>& closest,
        std::vector<double>& distances,
        std::vector<int>& faceIds);

    // CUDA implementation of one CENTER/LOWER/UPPER force-field update.
    // Returns false when CUDA cannot execute so the caller can use its CPU path.
    bool relaxEnergyRegionCuda(
        const std::vector<std::vector<Eigen::Vector3d>>& lines,
        const std::vector<bool>& movable,
        const std::vector<Eigen::Vector3d>& fixed,
        const std::vector<Eigen::Vector3d>& seam,
        const std::vector<Eigen::Vector3d>& surfaceSamples,
        int regionCode,
        double spacing,
        std::vector<std::vector<Eigen::Vector3d>>& updated);
}
