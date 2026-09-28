#include "GpuMeshProjection.h"

namespace SprayTrajPlan
{
#ifndef SPRAYTRAJPLAN_HAS_CUDA
    bool projectClosestPointsCuda(
        const std::vector<Eigen::Vector3d>&,
        const std::vector<Eigen::Vector3i>&,
        const std::vector<Eigen::Vector3d>&,
        std::vector<Eigen::Vector3d>&,
        std::vector<double>&,
        std::vector<int>&)
    {
        return false;
    }
#endif
#ifndef SPRAYTRAJPLAN_HAS_CUDA
    bool relaxEnergyRegionCuda(
        const std::vector<std::vector<Eigen::Vector3d>>&, const std::vector<bool>&,
        const std::vector<Eigen::Vector3d>&, const std::vector<Eigen::Vector3d>&,
        const std::vector<Eigen::Vector3d>&, int, double,
        std::vector<std::vector<Eigen::Vector3d>>&)
    {
        return false;
    }
#endif
}
