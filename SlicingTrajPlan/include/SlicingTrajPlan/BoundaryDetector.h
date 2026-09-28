#pragma once
#include "BoundaryDetector_global.h"
#include <GeneralData/GeneralData.h>  // 
#include <RenderData/BoundaryRenderData.h>
#include <vector>
#include <Eigen/Dense>
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/imodel.h>


struct Vector3dLexicographicalCompare {
    bool operator()(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) const {
        constexpr double epsilon = 1e-9;
        if (std::abs(lhs.x() - rhs.x()) > epsilon) return lhs.x() < rhs.x();
        if (std::abs(lhs.y() - rhs.y()) > epsilon) return lhs.y() < rhs.y();
        if (std::abs(lhs.z() - rhs.z()) > epsilon) return lhs.z() < rhs.z();
        return false;
    }
};

struct PairOfVector3dCompare {
    bool operator()(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& lhs,
        const std::pair<Eigen::Vector3d, Eigen::Vector3d>& rhs) const {
        Vector3dLexicographicalCompare vec_comp;
        if (vec_comp(lhs.first, rhs.first)) return true;
        if (vec_comp(rhs.first, lhs.first)) return false;
        return vec_comp(lhs.second, rhs.second);
    }
};

namespace ModelTrajPlan
{
    class BoundaryDetector_SHAREDLIB BoundaryDetector
    {
    public:
        BoundaryDetector();
        virtual ~BoundaryDetector();

        // [ALGORITHM] Detect boundaries from model
        // Input: modelData - STL model data
        //        numPoints - number of boundary points to generate
        // Output: boundaryData - detected upper/lower boundaries
        bool detectBoundaries(const ModelData& modelData, BoundaryData& boundaryData, Epiphyseal& intersectionPoints, MeshData& intersectionPointsData);
        void BoundaryDetector::computeLineAttributes(Epiphyseal& lines);
        bool BoundaryDetector::generateAdaptiveNodes(
            const Epiphyseal& lines,
            Epiphyseal& Pointnode,
            const double epsilon = 5.0,
            const double min_step = 50.0);
        //confire the relationship beteen lines and Pointnode
        bool BoundaryDetector::buildSlicingNodes(
            const Epiphyseal& lines,
            const Epiphyseal& Pointnode,
            Epiphyseal& slicingnode,
            const double min_step = 50.0);
        // [ALGORITHM] Compute offset boundaries
        // Input: boundaryData - original boundaries
        //        upperOffset, lowerOffset - offset distances
        // Output: boundaryData - updated with offset boundaries
        void computeOffsetBoundaries(BoundaryData& renderData, double upperOffset, double lowerOffset);

        bool initializeRenderer(BoundaryRenderData& renderData);

        // [RENDER] Draw boundaries from boundaryData
        // Input: boundaryData - boundary data to draw
        // Output: renderData - rendering calls
        bool draw(BoundaryRenderData& renderData, const BoundaryData& boundaryData);

        // [RENDER] Draw offset boundaries
        // Input: boundaryData - boundary data with offsets
        // Output: renderData - rendering calls
        void drawOffsetBoundaries(BoundaryRenderData& renderData, const BoundaryData& boundaryData);
        // [QUERY] Check if boundary data has renderable content
        static bool hasRenderableData(const BoundaryData& renderData);

        // Visibility and appearance control
        void setBoundaryVisibility(BoundaryRenderData& renderData, bool visible);
        void setOffsetBoundaryVisibility(BoundaryRenderData& renderData, bool visible);
        void setUpperBoundaryColor(BoundaryRenderData& renderData, const Eigen::Vector3f& color);
        void setLowerBoundaryColor(BoundaryRenderData& renderData, const Eigen::Vector3f& color);
        void setBoundaryLineWidth(BoundaryRenderData& renderData, float width);
        std::map<std::pair<Eigen::Vector3d, Eigen::Vector3d>, int, PairOfVector3dCompare>
            BoundaryDetector::getEdgeCountMap(const ModelData& modelData);
        bool BoundaryDetector::calculatePlaneIntersection(
            const ModelData& modelData,
            const BoundaryLine& upperBoundary,
            const BoundaryLine& lowerBoundary,
            Epiphyseal& boundary2,
            Eigen::Vector3d* phasePlaneNormal = nullptr);
        bool convertLinePointsToMeshData(
            const Epiphyseal& boundary,
            MeshData& colorline_data_element,
            float r = 1.0f, float g = 1.0f, float b = 0.2f, float a = 1.0f);
        void clear(BoundaryData& boundaryData);
        void clearOffsetBoundaries(BoundaryData& boundaryData);

        // [ALGORITHM] Convert boundary data to edge extraction result
        // Input: boundaryData - boundary data
        // Output: EdgeExtractionResult - converted result

    private:
        // === No internal data storage ===

        // [ALGORITHM] Internal processing methods (pure algorithms)
        void updateRenderData(BoundaryRenderData& renderData, const BoundaryData& boundaryData);
        // [ALGORITHM] Data conversion methods (pure data processing, all static)
        static void convertBoundaryToModelData(const BoundaryLine& boundary, MeshData& line_data);
        static std::vector<Eigen::Vector3d> extractBoundaryEdges(const ModelData& modelData, bool isUpper, double z_min, double z_max);
        static BoundaryLine computeOffsetForBoundary(const BoundaryLine& boundary, double offset);
    };

}
