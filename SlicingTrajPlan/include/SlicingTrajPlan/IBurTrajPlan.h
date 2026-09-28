// Header file containing class IBurTrajPlan
#pragma once

#include "BurTrajPlan_global.h"
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <GeneralData/GeneralData.h>// Addition: centralized definitions for TrajectoryPoint/CuttingPlane/DataState
#include <RenderData/RenderData.h>  // Addition: rendering data structures
#include <ModelData/ModelData.h>
#include <TrajData/TrajData.h>

namespace ModelTrajPlan


{
    // Forward declarations
    struct TrajectoryPoint;
    struct CuttingPlane;
    struct ConnectionSegment;
    struct EdgeExtractionResult;
    struct PrivateModel;
    class iscene;
    class imodel;

    // Smart pointer type is provided by ModelTangQing (boost::shared_ptr)
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/imodel.h>


// Trajectory planning system interface - core data management
    class BurTrajPlan_SHAREDLIB IBurTrajPlan
    {
    protected:
        IBurTrajPlan() = default;  // Protected constructor, only callable by derived classes
    public:
        virtual ~IBurTrajPlan() = 0;

        // === Data management interfaces ===
        // File management
        virtual void getFilePaths(const GeneralData& generalData, std::vector<std::string>& filePaths) const = 0;
        virtual void clear(GeneralData& generalData) = 0;

        // [Addition] Load data from PublicModel into GeneralData
        virtual bool loadModelFromPublicData(const PublicData::PublicModel& publicModel, GeneralData& generalData) = 0;

        // Data state
        virtual void getCurrentState(const GeneralData& generalData, DataState& state) const = 0;

        // === Processing pipeline interfaces ===

        // Trajectory generation
        virtual void getTrajectoryPoints(const GeneralData& generalData, std::vector<TrajectoryPoint>& points) const = 0;

        // Cutting-plane generation
        // Optional renderData: when non-null, boundary detection can fill intersection mesh into RenderData.
        virtual bool generateCuttingPlanes(GeneralData& generalData, RenderData* renderData = nullptr) = 0;
        virtual void getCuttingPlanes(const GeneralData& generalData, std::vector<CuttingPlane>& planes) const = 0;

        // === Rendering interfaces (RenderData passed as argument) ===
        // Initialize rendering system
        virtual bool initializeRenderer(RenderData& renderData) = 0;

        // Main draw method
        virtual bool draw(const GeneralData& generalData, RenderData& renderData) = 0;

        // Visibility controls
        virtual void setOriginalDataVisibility(RenderData& renderData, bool visible) = 0;
        virtual void setTrajectoryVisibility(RenderData& renderData, bool visible) = 0;
        virtual void setCuttingPlanesVisibility(RenderData& renderData, bool visible) = 0;
        virtual void setBoundaryVisibility(RenderData& renderData, bool visible) = 0;

        // Addition: "keep original data visible after preprocessing" switch
        virtual void setKeepOriginalVisibleAfterPreprocess(bool keep) = 0;

        // Check state
        virtual bool needsRenderUpdate(const RenderData& renderData) const = 0;

        // Check whether renderable data exists
        virtual bool hasRenderableData(const GeneralData& generalData, const RenderData& renderData) const = 0;

        // === Statistics interfaces ===
        virtual void getTotalPointCount(const GeneralData& generalData, size_t& count) const = 0;
        virtual void getTotalTriangleCount(const GeneralData& generalData, size_t& count) const = 0;
        virtual void getTrajectoryPointCount(const GeneralData& generalData, size_t& count) const = 0;

        // === Parameter management interfaces ===
        // Slicing parameters - set and queried through GeneralData
        virtual bool setSlicingSize(GeneralData& generalData, int size) = 0;
        virtual void getSlicingSize(const GeneralData& generalData, int& size) const = 0;
        virtual bool setZ1Offset(GeneralData& generalData, double offset) = 0;
        virtual void getZ1Offset(const GeneralData& generalData, double& offset) const = 0;
        virtual bool setZ2Offset(GeneralData& generalData, double offset) = 0;
        virtual void getZ2Offset(const GeneralData& generalData, double& offset) const = 0;
        virtual bool setOffsetDistance(GeneralData& generalData, double distance) = 0;
        virtual void getOffsetDistance(const GeneralData& generalData, double& distance) const = 0;
        virtual bool setnormalVelocity_(GeneralData& generalData, double Velocity) = 0;
        virtual void getnormalVelocity_(const GeneralData& generalData, double& Velocity) const = 0;
        virtual bool settransitionVelocity_(GeneralData& generalData, double Velocity) = 0;
        virtual void gettransitionVelocity_(const GeneralData& generalData, double& Velocity) const = 0;
        // Parameter presets
        virtual bool applyCoarsePreset(GeneralData& generalData) = 0;

        // === Static factory method - following iscene style ===
        static boost::shared_ptr<IBurTrajPlan> create_trajplan();

        // Addition: custom slicing Z positions
        virtual bool setCustomZPositions(GeneralData& generalData, const std::vector<double>& zPositions) = 0;
        virtual void getCustomZPositions(const GeneralData& generalData, std::vector<double>& zPositions) const = 0;

        // === Slicing data management interfaces ===
        virtual void clearCuttingPlanes(GeneralData& generalData) = 0;  // Clear all slicing data

        // [Addition] Update size of a single cutting plane (compute transformed intersection bounding box * 1.2)
        virtual bool updateSingleCuttingPlaneSize(GeneralData& generalData, int layerIndex) = 0;

        // === STL processing interfaces ===
        virtual bool isSTLProcessed(const GeneralData& generalData) const = 0;  // Check whether STL is processed
        virtual bool isBoundaryDetected(const GeneralData& generalData) const = 0;  // Check whether boundary is detected

        // === Trajectory generation and export interfaces ===
        virtual bool generateMATLABTrajectory(GeneralData& generalData, int size_n,
            double z1Offset, double z2Offset,
            double offsetDistance,
            PublicData::TrajData& publicTrajData) = 0;

        virtual bool exportTrajectoryToFile(const GeneralData& generalData, const std::string& filename) const = 0;
        virtual bool exportFANUCProgram(const GeneralData& generalData, const std::string& filename) const = 0;
        virtual bool exportABBProgram(const GeneralData& generalData, const std::string& txtFilename, const std::string& modFilename) const = 0;
        virtual bool exportToExcel(const GeneralData& generalData, const std::string& filename) const = 0;
        virtual bool exportToExcelWithVelocity(const GeneralData& generalData, const std::string& filename) const = 0;

        // === Trajectory connection interfaces ===
        virtual bool connectTrajectoryPoints(GeneralData& generalData, ConnectionStrategy strategy = ConnectionStrategy::Sequential) = 0;
        virtual bool setConnectionVisibility(RenderData& renderData, bool visible) = 0;
    };

    // Typedef defined in iscene style
    typedef boost::shared_ptr<IBurTrajPlan> IBurTrajPlan_spointer;

}