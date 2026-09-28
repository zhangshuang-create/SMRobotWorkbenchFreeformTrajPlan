#pragma once
#include "UniformSlicer_global.h"
#include <GeneralData/GeneralData.h>  // Use unified type definitions from GeneralData
#include <RenderData/SlicingRenderData.h>
#include <STLProcessor/STLProcessor.h>
#include <BoundaryDetector/BoundaryDetector.h>
#include <vector>
#include <Eigen/Dense>
#include <ModelTangQing/imodel.h>
#include <ModelTangQing/MeshData.h>
#include <ModelTangQing/iscene.h>
namespace ModelTrajPlan
{
    class UniformSlicer_SHAREDLIB UniformSlicer
    {
    public:
        UniformSlicer();
        virtual ~UniformSlicer();

        // === Core Slicing Functions ===
        // [ALGORITHM] Perform slicing on model
        // Input: modelData - STL model data
        //        upperBoundary, lowerBoundary - boundary data
        //        params - slicing parameters
        //        oldCuttingPlanes - previous cutting planes with transformation parameters (optional, for preserving transformations)
        // Output: sliceResult - slicing result data
        //         cuttingPlanes - basic cutting planes for display (optional output)
        bool performSlicing(const ModelData& modelData,
            const BoundaryLine& upperBoundary,
            const BoundaryLine& lowerBoundary,
            const SlicingParams& params,
            SlicingLocation& slicingLocations,
            Epiphyseal& slicingnode,
            SliceResult& sliceResult,
            std::vector<CuttingPlane>* cuttingPlanes = nullptr,
            const std::vector<CuttingPlane>* oldCuttingPlanes = nullptr);

        // === Rendering Interface (receives data as parameters) ===
        bool initializeRenderer(SlicingRenderData& renderData);
        static bool convertToCuttingPlanes(
            SliceResult& sliceResult,
            const std::vector<CuttingPlane>* oldCuttingPlanes,
            const ModelData* modelData);

        // [RENDER] Create cutting planes rendering model
        // Input: cuttingPlanes - cutting plane data
        //        modelData - STL model (optional, for bounds)
        // Output: renderData - updated with rendering model
        static imodel_spointer createCuttingPlanesModel(SlicingRenderData& renderData,
            const std::vector<CuttingPlane>& cuttingPlanes,
            const SliceResult& sliceResult);
        // Update single cutting plane size (calculate bounding box * 1.2 from transformed intersection points)
        // Input: plane - cutting plane data (contains transformation parameters)
        //        modelData - STL model
        //        originalCenter - original center point (untransformed center point)
        // Output: plane.width, plane.height, plane.vertices - updated size and vertices
        static bool updateSingleCuttingPlaneSize(CuttingPlane& plane,
            const ModelData& modelData,
            const Eigen::Vector3d& originalCenter,
            bool& updata);
        bool draw(SlicingRenderData& renderData,
            const SlicingData& slicing) const;

        bool buildSlicePlanes(
            const Epiphyseal& slicingnode,
            SlicingLocation& planes);
    private:
        // === No internal data storage ===

        // [ALGORITHM] Private helper functions (pure algorithms, all static)

        // [NEW] Apply transformation parameters to vertices around center point
        // Input: vertices - original vertices
        //        center - transformation center point
        //        tx, ty, tz - translation parameters
        //        rx, ry, rz - rotation parameters (in degrees)
        // Output: vector<Vector3d> - transformed vertices
        static std::vector<Eigen::Vector3d> applyTransformationToVertices(
            const std::vector<Eigen::Vector3d>& vertices,
            const Eigen::Vector3d& center,
            double tx, double ty, double tz,
            double rx, double ry, double rz);
        static std::vector<BoundaryLine> generateInternalBoundaries(
            const ModelData& modelData,
            const SlicingParams& params,
            double zMin, double zMax,
            const SlicingLocation& slicingLocations);

        // [REMOVED] generateReferenceLines and calculateQuaternions functions
        // Responsibility transferred: TrajectoryGenerator is responsible for calculating reference lines and poses

        // Color conversion helper
        static Eigen::Vector3f hue2rgb(float h);

        static std::vector<Eigen::Vector3d> calculateSTLIntersectionWithPlane(
            const ModelData& modelData,
            const Eigen::Vector3d& planeCenter,
            const Eigen::Vector3d& planeNormal,
            double referenceZ = 0.0);

        // [ALGORITHM] Trace contours from intersection points
        // Input: modelData - STL model
        //        planeCenter - plane center point
        //        planeNormal - plane normal vector
        //        z - Z height
        // Output: vector<BoundaryLine> - traced contour lines
        static std::vector<BoundaryLine> UniformSlicer::traceContoursFromIntersections(
            const ModelData& modelData,
            const SlicePlane& plane);
    };

}