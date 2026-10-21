#pragma once

#include <SlicingTrajPlan/GeneralData.h>

#include <map>
#include <utility>

struct Vector3dLexicographicalCompare
{
    bool operator()(const Eigen::Vector3d& left, const Eigen::Vector3d& right) const;
};

struct PairOfVector3dCompare
{
    bool operator()(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& left,
        const std::pair<Eigen::Vector3d, Eigen::Vector3d>& right) const;
};

namespace ModelTrajPlan
{
    // Original BurTrajPlan boundary algorithm. It owns only algorithm data;
    // rendering conversion belongs to the Workbench.
    struct BoundaryTopologyDiagnostics {
        size_t loopCount=0, branchVertices=0, openVertices=0, nonManifoldEdges=0;
        bool usable() const { return loopCount==2 && !branchVertices && !openVertices && !nonManifoldEdges; }
    };
    class BoundaryDetector final
    {
    public:
        BoundaryDetector();
        ~BoundaryDetector();

        bool detectBoundaries(const ModelData& modelData, BoundaryData& boundaryData,
            Epiphyseal& intersectionPoints, BoundaryTopologyDiagnostics* topology = nullptr);
        void computeLineAttributes(Epiphyseal& lines);
        bool generateAdaptiveNodes(const Epiphyseal& lines, Epiphyseal& pointNodes,
            double epsilon = 5.0, double minimumStep = 50.0);
        bool buildSlicingNodes(const Epiphyseal& lines, const Epiphyseal& pointNodes,
            Epiphyseal& slicingNodes, double minimumStep = 50.0, int maximumSliceCount = 0);
        void computeOffsetBoundaries(BoundaryData& boundaryData, double upperOffset, double lowerOffset);
        std::map<std::pair<Eigen::Vector3d, Eigen::Vector3d>, int, PairOfVector3dCompare>
            getEdgeCountMap(const ModelData& modelData);
        bool calculatePlaneIntersection(const ModelData& modelData, const BoundaryLine& upperBoundary,
            const BoundaryLine& lowerBoundary, Epiphyseal& boundary, Eigen::Vector3d* phasePlaneNormal = nullptr);
        void clear(BoundaryData& boundaryData);
        void clearOffsetBoundaries(BoundaryData& boundaryData);

    private:
        static std::vector<Eigen::Vector3d> extractBoundaryEdges(
            const ModelData& modelData, bool isUpper, double minimumZ, double maximumZ);
        static BoundaryLine computeOffsetForBoundary(const BoundaryLine& boundary, double offset);
    };
}
