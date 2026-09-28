#include "CommonDataTypes.h"

namespace SprayTrajPlan
{
    // PrecutSeam implementation
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> PrecutSeam::getEdges() const
    {
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;

        if (xyz.size() < 2) {
            return edges;
        }

        edges.reserve(xyz.size() - 1);
        for (size_t i = 0; i < xyz.size() - 1; ++i) {
            edges.emplace_back(xyz[i], xyz[i + 1]);
        }

        return edges;
    }

} // namespace SprayTrajPlan
