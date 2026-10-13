#pragma once
#include <SlicingTrajPlan/BurTrajPlan_global.h>
#include <SlicingTrajPlan/IBurTrajPlan.h>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <SlicingTrajPlan/STLProcessor.h>
#include <SlicingTrajPlan/BoundaryDetector.h>
#include <SlicingTrajPlan/UniformSlicer.h>
#include <SlicingTrajPlan/TrajectoryConnector.h>
#include <SlicingTrajPlan/TrajectoryExporter.h>
#include <SlicingTrajPlan/TrajectoryGenerator.h>
#include <SlicingTrajPlan/GeneralData.h>
namespace ModelTrajPlan
{
    // Forward declarations to avoid cyclic dependencies and type conflicts
    class TrajectoryGenerator;
    class TrajectoryConnector;
    class STLProcessor;
    class BoundaryDetector;
    class UniformSlicer;

    // BurTrajPlan - core container class for the trajectory planning system (internal implementation, not exported directly)
    class BurTrajPlan_SHAREDLIB BurTrajPlan : public IBurTrajPlan
    {
    public:
        BurTrajPlan();
        virtual ~BurTrajPlan();

        // === IBurTrajPlan interface implementation ===

        // Data management interfaces
        void getFilePaths(const GeneralData& generalData, std::vector<std::string>& filePaths) const override;
        void clear(GeneralData& generalData) override;

        // Data state
        void getCurrentState(const GeneralData& generalData, DataState& state) const override;



        void getTrajectoryPoints(const GeneralData& generalData, std::vector<TrajectoryPoint>& points) const override;

        bool generateCuttingPlanes(GeneralData& generalData) override;
        void getCuttingPlanes(const GeneralData& generalData, std::vector<CuttingPlane>& planes) const override;

        // Statistics interfaces
        void getTotalPointCount(const GeneralData& generalData, size_t& count) const override;
        void getTotalTriangleCount(const GeneralData& generalData, size_t& count) const override;
        void getTrajectoryPointCount(const GeneralData& generalData, size_t& count) const override;

        // === Parameter management implementation ===
        // Slicing parameters
        bool setSlicingSize(GeneralData& generalData, int size) override;
        void getSlicingSize(const GeneralData& generalData, int& size) const override;
        bool setZ1Offset(GeneralData& generalData, double offset) override;
        void getZ1Offset(const GeneralData& generalData, double& offset) const override;
        bool setZ2Offset(GeneralData& generalData, double offset) override;
        void getZ2Offset(const GeneralData& generalData, double& offset) const override;
        bool setOffsetDistance(GeneralData& generalData, double distance) override;
        void getOffsetDistance(const GeneralData& generalData, double& distance) const override;
        bool setnormalVelocity_(GeneralData& generalData, double Velocity) override;
        void getnormalVelocity_(const GeneralData& generalData, double& Velocity) const override;
        bool settransitionVelocity_(GeneralData& generalData, double Velocity) override;
        void gettransitionVelocity_(const GeneralData& generalData, double& Velocity) const override;
        // Parameter presets
        bool applyCoarsePreset(GeneralData& generalData) override;


        // === STL processing interfaces ===
        bool isSTLProcessed(const GeneralData& generalData) const override;
        bool isBoundaryDetected(const GeneralData& generalData) const override;

        // === Slicing data management ===
        void clearCuttingPlanes(GeneralData& generalData) override;  // Clear all slicing data

        // [Addition] Update size of a single cutting plane (compute transformed intersection bounding box * 1.2)
        bool updateSingleCuttingPlaneSize(GeneralData& generalData, int layerIndex) override;
        bool deleteCuttingPlane(GeneralData& generalData, int layerIndex);

        // === Trajectory generation interfaces (matching MATLAB behavior) ===
        bool generateMATLABTrajectory(GeneralData& generalData, int size_n,
            double z1Offset, double z2Offset,
            double offsetDistance) override;

        // Export trajectory data
        bool exportTrajectoryToFile(const GeneralData& generalData, const std::string& filename) const override;
        bool exportFANUCProgram(const GeneralData& generalData, const std::string& filename) const override;
        bool exportABBProgram(const GeneralData& generalData, const std::string& txtFilename, const std::string& modFilename) const override;
        bool exportToExcel(const GeneralData& generalData, const std::string& filename) const override;
        bool exportToExcelWithVelocity(const GeneralData& generalData, const std::string& filename) const override;

        // === Trajectory connection feature interfaces ===

        // Execute trajectory connection
        bool connectTrajectoryPoints(GeneralData& generalData, ConnectionStrategy strategy = ConnectionStrategy::BowPath) override;

        // === Custom Z-position management ===
        bool setCustomZPositions(GeneralData& generalData, const std::vector<double>& zPositions) override;
        void getCustomZPositions(const GeneralData& generalData, std::vector<double>& zPositions) const override;

        // === Extended trajectory connection interfaces ===
        TrajectoryGenerator* getTrajectoryGenerator() const;
        bool exportConnectedTrajectory(const GeneralData& generalData, const std::string& filename) const;

    private:
        // === Algorithm modules (stateless, no data storage) ===
        std::unique_ptr<STLProcessor> stlProcessor_;
        std::unique_ptr<BoundaryDetector> boundaryDetector_;
        std::unique_ptr<UniformSlicer> uniformSlicer_;
        TrajectoryGenerator* trajectoryGenerator_;
        std::unique_ptr<TrajectoryConnector> trajectoryConnector_;

        // === Thread safety ===
        mutable std::mutex mutex_;

        // === Private algorithm methods (pure algorithms, no internal state) ===

        // [ALGORITHM] Boundary detection
        // Input: generalData - contains model data
        //        numTrajectories - number of trajectory lines to generate
        // Output: generalData.boundaryData_ - boundary data
        //         generalData.processingState_.isBoundaryDetected - state flag
        bool runBoundaryDetection(GeneralData& generalData);

        // [ALGORITHM] Clear cutting plane data
        // Input/Output: generalData - clears cutting planes and trajectory data
        void clearCuttingPlanesData(GeneralData& generalData);

        // [ALGORITHM] Prepare slicing parameters
        // Input: upperBoundary, lowerBoundary - boundary data
        //        generalData - contains parameter settings
        // Output: SlicingParams - validated parameters
        SlicingParams prepareSlicingParams(
            const GeneralData& generalData,
            const BoundaryLine& upperBoundary,
            const BoundaryLine& lowerBoundary
        );

        // [ALGORITHM] Validate Z positions
        // Input: zPositions - Z positions to validate
        //        upperBoundary, lowerBoundary - boundaries for range validation
        // Output: vector<double> - validated Z positions
        std::vector<double> validateZPositions(
            const std::vector<double>& zPositions,
            const BoundaryLine& upperBoundary,
            const BoundaryLine& lowerBoundary
        );

        // [ALGORITHM] Update slicing state after slicing
        // Input: uniformSlicer_ - slice result
        // Output: generalData - updated state and custom Z positions
        void updateSlicingState(GeneralData& generalData, const SliceResult& sliceResult);

        // [ALGORITHM] Convert slice data to cutting planes
        // Input: uniformSlicer_ - slice result
        // Output: generalData.slicingData_.cuttingPlanes_ - cutting plane data
        void convertSliceDataToCuttingPlanes(GeneralData& generalData);

    };
}
