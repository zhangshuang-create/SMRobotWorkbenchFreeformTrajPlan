#include <SlicingTrajPlan/BurTrajPlan.h>
#include <SlicingTrajPlan/TrajectoryGenerator.h>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <memory>
#include <limits>
#include <SlicingTrajPlan/STLProcessor.h>
#include <SlicingTrajPlan/BoundaryDetector.h>
#include <SlicingTrajPlan/UniformSlicer.h>
#include <SlicingTrajPlan/TrajectoryConnector.h>

namespace ModelTrajPlan
{
    // Constructor: Initialize algorithm module instances and set trajectory generator pointer to null
    BurTrajPlan::BurTrajPlan()
        : stlProcessor_(std::make_unique<STLProcessor>())
        , boundaryDetector_(std::make_unique<BoundaryDetector>())
        , uniformSlicer_(std::make_unique<UniformSlicer>())
        , trajectoryGenerator_(nullptr)
        , trajectoryConnector_(std::make_unique<TrajectoryConnector>())
    {
        // [DESIGN] Pure algorithm class - no internal data storage
        // All data is passed via GeneralData parameter
    }

    BurTrajPlan::~BurTrajPlan()
    {
        if (trajectoryGenerator_) {
            delete trajectoryGenerator_;
            trajectoryGenerator_ = nullptr;
        }
    }

    void BurTrajPlan::clear(GeneralData& generalData)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        generalData.filePaths_.clear();
        generalData.modelDataManager_.allModelsData_.clear();

        // Clear trajectory data
        generalData.trajData_.trajectoryPoints_.clear();
        generalData.trajData_.sampledBoundaryPoints_.clear();  // Clear sampled boundary point data

        // Clear slicing data
        generalData.slicingData_.cuttingPlanes_.clear();
        generalData.slicingData_.slicingLocations_.clear();
        generalData.slicingData_.sliceLines_.clear();           // Clear slicing boundary line data
        generalData.slicingData_.currentSliceResult_ = SliceResult();  // Reset current slice result
        generalData.slicingData_.managedParams_.customZPositions.clear();
        generalData.slicingData_.slicingisupdata = false;
        
        if (stlProcessor_) {
            stlProcessor_->clear(generalData.modelDataManager_);
        }

        if (boundaryDetector_) {
            boundaryDetector_->clear(generalData.boundaryData_);
        }
        if (uniformSlicer_) {
            clearCuttingPlanesData(generalData);
        }

        generalData.trajData_.connectionData_.connectedTrajectory_.clear();
        generalData.trajData_.connectionData_.connectionSegments_.clear();
        generalData.trajData_.connectionData_.isConnected_ = false;

        if (trajectoryGenerator_) {
            delete trajectoryGenerator_;
            trajectoryGenerator_ = nullptr;
        }

        generalData.processingState_.isSTLProcessed = false;
        generalData.processingState_.isBoundaryDetected = false;
        generalData.processingState_.isSlicingCompleted = false;
        generalData.processingState_.isTrajectoryGenerated = false;
        generalData.processingState_.isTrajectoryConnected = false;
        generalData.currentState_ = DataState::Empty;
    }

    bool BurTrajPlan::generateCuttingPlanes(GeneralData& generalData)
    {
        // [ALGORITHM] Generate cutting planes from model
        // Input: generalData.modelDataManager_.mainModelData_ - STL model
        //        generalData.slicingData_.managedParams_ - slicing parameters
        // Output: generalData.slicingData_.cuttingPlanes_ - generated cutting planes

        // 1. Check preconditions
        if (!generalData.processingState_.isSTLProcessed) {
            return false;
        }

        // 2. Try boundary detection. A mesh without two open boundaries uses
        // UniformSlicer's independent whole-model fallback instead of failing.
        if (!generalData.processingState_.isBoundaryDetected) {
            runBoundaryDetection(generalData);
        }
        const bool hasBothBoundaries = !generalData.boundaryData_.upperBoundary_.points.empty() &&
            !generalData.boundaryData_.lowerBoundary_.points.empty();
        if (!hasBothBoundaries) {
            // Never mix a partial boundary or stale skeleton with the
            // boundaryless path: it must be derived solely from the STL.
            generalData.boundaryData_.upperBoundary_ = BoundaryLine();
            generalData.boundaryData_.lowerBoundary_ = BoundaryLine();
            generalData.boundaryData_.upperBoundaryOffset_ = BoundaryLine();
            generalData.boundaryData_.lowerBoundaryOffset_ = BoundaryLine();
            generalData.boundaryData_.intersectionPoints_.clear();
            generalData.boundaryData_.projectedContourLines_.clear();
            generalData.boundaryData_.phaseReferenceLine_.clear();
            generalData.boundaryData_.phaseReferencePlaneNormal_.setZero();
            generalData.boundaryData_.LinePoints_.clear();
            generalData.boundaryData_.Mainslicingnode.clear();
        }

        // Rebuild adaptive slicing nodes every run so UI-tuned epsilon and minStep parameters take effect immediately
        if (generalData.slicingData_.managedParams_.mode == SlicingMode::AA && hasBothBoundaries &&
            boundaryDetector_ && !generalData.boundaryData_.projectedContourLines_.empty()) {
            Epiphyseal adaptiveNodes;
            const bool adaptiveOk = boundaryDetector_->generateAdaptiveNodes(
                generalData.boundaryData_.projectedContourLines_,
                adaptiveNodes,
                generalData.boundaryData_.epsilon_,
                generalData.boundaryData_.minStep_);

            Epiphyseal slicingNodes;
            const bool slicingNodeOk = adaptiveOk && boundaryDetector_->buildSlicingNodes(
                generalData.boundaryData_.projectedContourLines_,
                adaptiveNodes,
                slicingNodes,
                generalData.boundaryData_.minStep_,
                generalData.slicingData_.managedParams_.maxSliceCount);

            if (adaptiveOk && slicingNodeOk) {
                generalData.boundaryData_.LinePoints_ = std::move(adaptiveNodes);
                generalData.boundaryData_.Mainslicingnode = std::move(slicingNodes);
            } else {
                generalData.boundaryData_.LinePoints_.clear();
                generalData.boundaryData_.Mainslicingnode.clear();
            }
        }

        // 3. [Modified] Preserve transform parameters of existing cutting planes (if present)
        std::vector<CuttingPlane> oldCuttingPlanes = generalData.slicingData_.cuttingPlanes_;

        // Clear old data
        clearCuttingPlanesData(generalData);

        // 4. [Refactor Removed] No longer precompute offset boundaries
        // Offset boundaries are handled automatically by TrajectoryGenerator during trajectory generation

        // 5. Execute slicing - pure algorithm module call
        if (!uniformSlicer_) {
            return false;
        }

        const PrivateModel& modelData = generalData.modelDataManager_.mainModelData_;
        const BoundaryLine& upperBoundary = generalData.boundaryData_.upperBoundary_;
        const BoundaryLine& lowerBoundary = generalData.boundaryData_.lowerBoundary_;
        
        // Prepare slicing parameters from generalData
        SlicingParams params = prepareSlicingParams(generalData, upperBoundary, lowerBoundary);
        
        // Execute slicing
        ModelData tempModelData2;
        tempModelData2.mainModelData_ = modelData;

        SliceResult sliceResult;
        // Prefer adaptive slicing nodes; fallback to the projected contour
        // envelope if adaptive nodes are unavailable.
        Epiphyseal* slicingNodeInput = &generalData.boundaryData_.Mainslicingnode;
        if (slicingNodeInput->size() < 3) {
            std::cerr << "[BurTrajPlan] Fallback to projectedContourLines_ because Mainslicingnode is invalid. "
                << "This may reduce epsilon/min_step sensitivity. "
                << "Mainslicingnode.size=" << generalData.boundaryData_.Mainslicingnode.size()
                << ", epsilon=" << generalData.boundaryData_.epsilon_
                << ", minStep=" << generalData.boundaryData_.minStep_
                << std::endl;
            slicingNodeInput = &generalData.boundaryData_.projectedContourLines_;
        }
        
        // [Modified] performSlicing can now generate base cuttingPlanes at the same time
        // Pass oldCuttingPlanes to preserve pose and transform parameters
        if (!uniformSlicer_->performSlicing(tempModelData2, upperBoundary, lowerBoundary, params,
            generalData.slicingData_.slicingLocations_,
            *slicingNodeInput,
            sliceResult,
            &generalData.slicingData_.cuttingPlanes_,
            oldCuttingPlanes.empty() ? nullptr : &oldCuttingPlanes)) {
            std::cerr << "[BurTrajPlan] generateCuttingPlanes: UniformSlicer::performSlicing returned false"
                      << ", mode=" << static_cast<int>(params.mode)
                      << ", upperPoints=" << upperBoundary.points.size()
                      << ", lowerPoints=" << lowerBoundary.points.size()
                      << ", minStep=" << params.minStep
                      << std::endl;
            return false;
        }

        // 6. Store slice result
        generalData.slicingData_.currentSliceResult_ = sliceResult;

        // Update slicing completed state
        updateSlicingState(generalData, sliceResult);

        return true;
    }

    // [NEW] Dedicated function to clear cutting plane data
    void BurTrajPlan::clearCuttingPlanesData(GeneralData& generalData)
    {
        generalData.slicingData_.cuttingPlanes_.clear();
        generalData.slicingData_.slicingLocations_.clear();
        generalData.slicingData_.sliceLines_.clear();            // [Addition] Clear slicing boundary contour lines
        generalData.trajData_.trajectoryPoints_.clear();
        generalData.trajData_.sampledBoundaryPoints_.clear();    // [Addition] Clear sampled point data
    }

    // [ALGORITHM] Prepare and validate slicing parameters from generalData
    // Input: generalData.slicingData_.managedParams_ - parameter settings
    //        upperBoundary, lowerBoundary - for Z validation
    // Output: SlicingParams - validated parameters ready for slicing
    SlicingParams BurTrajPlan::prepareSlicingParams(
        const GeneralData& generalData,
        const BoundaryLine& upperBoundary,
        const BoundaryLine& lowerBoundary)
    {
        SlicingParams params;
        // Refactored to only extract parameters closely related to slicing
        params.numTrajectories = generalData.slicingData_.managedParams_.numTrajectories;
        params.trajectorySpacing = generalData.slicingData_.managedParams_.trajectorySpacing;
        params.minStep = generalData.boundaryData_.minStep_;
        params.maxSliceCount = generalData.slicingData_.managedParams_.maxSliceCount;
        params.mode = generalData.slicingData_.managedParams_.mode;
        // Offset parameters have been decoupled and are no longer fetched from SlicingParams

        // Validate and filter custom Z positions
        if (!generalData.slicingData_.managedParams_.customZPositions.empty()) {
            params.customZPositions = validateZPositions(
                generalData.slicingData_.managedParams_.customZPositions,
                upperBoundary,
                lowerBoundary
            );
        }

        return params;
    }

    // [NEW] Dedicated function to validate and clean custom Z layer positions
    std::vector<double> BurTrajPlan::validateZPositions(const std::vector<double>& zPositions,
        const BoundaryLine& upperBoundary,
        const BoundaryLine& lowerBoundary)
    {
        // Calculate Z range bounded by upper and lower boundaries
        double zMin = std::numeric_limits<double>::max();
        double zMax = std::numeric_limits<double>::lowest();

        for (const auto& p : lowerBoundary.points) zMin = std::min(zMin, p.position.z());
        for (const auto& p : upperBoundary.points) zMax = std::max(zMax, p.position.z());

        // Sort, deduplicate, filter out-of-range layers
        std::vector<double> validZPositions = zPositions;
        std::sort(validZPositions.begin(), validZPositions.end());
        validZPositions.erase(std::unique(validZPositions.begin(), validZPositions.end()), validZPositions.end());

        validZPositions.erase(
            std::remove_if(validZPositions.begin(), validZPositions.end(),
                [zMin, zMax](double z) { return (z < zMin || z > zMax); }),
            validZPositions.end());

        return validZPositions;
    }

    // [ALGORITHM] Update internal state and layer positions after slicing completes
    // Input: sliceResult - slice result from algorithm module
    // Output: generalData.processingState_.isSlicingCompleted - state flag
    //         generalData.slicingData_.managedParams_.customZPositions - write back layer heights
    void BurTrajPlan::updateSlicingState(GeneralData& generalData, const SliceResult& sliceResult)
    {
        generalData.processingState_.isSlicingCompleted = true;
        generalData.currentState_ = DataState::EdgeExtracted;

        // Write back actual sliced Z layer coordinates to managedParams
        generalData.slicingData_.managedParams_.customZPositions.clear();
        for (const auto& b : sliceResult.internalBoundaries) {
            generalData.slicingData_.managedParams_.customZPositions.push_back(b.z_level);
        }
    }

    // === Statistics implementation ===
    void BurTrajPlan::getTotalPointCount(const GeneralData& generalData, size_t& count) const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        count = 0;
        for (const auto& data : generalData.originalData_) {
            count += data.m_vertices.size();
        }
        for (const auto& data : generalData.preprocessedData_) {
            count += data.m_vertices.size();
        }
    }

    void BurTrajPlan::getTotalTriangleCount(const GeneralData& generalData, size_t& count) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        count = 0;
        // Count mesh triangle facets in ProcessingInput
        for (const auto& data : generalData.originalData_) {
            count += data.m_indices.size();
        }
        for (const auto& data : generalData.preprocessedData_) {
            count += data.m_indices.size();
        }

        // Count triangles in STLProcessor's ModelData
        if (stlProcessor_ && STLProcessor::hasValidData(generalData.modelDataManager_)) {
            const PrivateModel& modelData = STLProcessor::getModelData(generalData.modelDataManager_);
            count += STLProcessor::getTriangleCount(modelData);
        }
    }

    void BurTrajPlan::getTrajectoryPointCount(const GeneralData& generalData, size_t& count) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        count = generalData.trajData_.trajectoryPoints_.size();
    }

    // === Internal helper methods implementation ===

    void BurTrajPlan::clearCuttingPlanes(GeneralData& generalData)
    {
        // [ALGORITHM] Clear cutting planes while preserving STL and boundary feature state
        // Input: generalData.processingState_ - current processing stage state
        // Output: generalData.slicingData_.cuttingPlanes_ - cleared
        //         generalData.processingState_.isSlicingCompleted - reset to false

        // Temporarily store stage flags that need to be preserved
        bool savedSTLProcessed = generalData.processingState_.isSTLProcessed;
        bool savedBoundaryDetected = generalData.processingState_.isBoundaryDetected;

        // Clear slicing and generated trajectory data
        generalData.slicingData_.cuttingPlanes_.clear();
        generalData.slicingData_.slicingLocations_.clear();
        generalData.slicingData_.sliceLines_.clear();            // Clear slicing boundary lines
        generalData.trajData_.trajectoryPoints_.clear();
        generalData.trajData_.sampledBoundaryPoints_.clear();    // Clear sampled boundary points
        
        // A new slice set invalidates the connected cross-layer path as well. Leaving
        // this cache populated makes a second confirmation display/export the
        // previous trajectory even though its cutting planes were replaced.
        generalData.trajData_.connectionData_.connectedTrajectory_.clear();
        generalData.trajData_.connectionData_.connectionSegments_.clear();
        generalData.trajData_.connectionData_.isConnected_ = false;

        if (boundaryDetector_) {
            boundaryDetector_->clearOffsetBoundaries(generalData.boundaryData_);
        }

        // Restore model and boundary states, only reset slicing and subsequent states
        generalData.processingState_.isSTLProcessed = savedSTLProcessed;
        generalData.processingState_.isBoundaryDetected = savedBoundaryDetected;
        generalData.processingState_.isSlicingCompleted = false;
        generalData.processingState_.isTrajectoryGenerated = false;
        generalData.processingState_.isTrajectoryConnected = false;
    }

    // [Addition] Update geometric bounding size of a single cutting plane
    bool BurTrajPlan::updateSingleCuttingPlaneSize(GeneralData& generalData, int layerIndex)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (layerIndex < 0 || layerIndex >= static_cast<int>(generalData.slicingData_.cuttingPlanes_.size())) {
            return false;
        }

        if (!generalData.processingState_.isSTLProcessed) {
            return false;
        }

        // If a previously selected layer exists and is different from current layer, deselect it first
        int& lastSelected = generalData.slicingData_.lastSelectedLayerIndex;
        if (lastSelected != -1 &&
            lastSelected != layerIndex &&
            lastSelected < static_cast<int>(generalData.slicingData_.cuttingPlanes_.size())) {
            generalData.slicingData_.cuttingPlanes_[lastSelected].isSelected = 0;
        }

        // Select current layer
        CuttingPlane& plane = generalData.slicingData_.cuttingPlanes_[layerIndex];
        plane.isSelected = 1;

        // Update record index
        lastSelected = layerIndex;

        // [Fix] plane.center is already the original center without translation, use directly
        Eigen::Vector3d originalCenter = plane.center;

        // Prepare ModelData
        ModelData tempModelData;
        tempModelData.mainModelData_ = generalData.modelDataManager_.mainModelData_;

        // Rebuild the base quad, then re-intersect the transformed plane so
        // the persistent contour and the view overlay describe the same cut.
        if (!UniformSlicer::updateSingleCuttingPlaneSize(
                plane, tempModelData, originalCenter, generalData.slicingData_.slicingisupdata)) {
            return false;
        }
        return UniformSlicer::convertToCuttingPlanes(
            generalData.slicingData_.currentSliceResult_,
            &generalData.slicingData_.cuttingPlanes_, &tempModelData);
    }

    // === Custom Z position set/get ===
    bool BurTrajPlan::setCustomZPositions(GeneralData& generalData, const std::vector<double>& zPositions)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [ALGORITHM] Store custom Z coordinates in generalData only
        generalData.slicingData_.managedParams_.customZPositions = zPositions;
        return true;
    }

    void BurTrajPlan::getCustomZPositions(const GeneralData& generalData, std::vector<double>& zPositions) const
    {
        // [QUERY] Read configured custom Z coordinate list from generalData
        zPositions = generalData.slicingData_.managedParams_.customZPositions;
    }

    // === Trajectory generation core interface implementation ===
    bool BurTrajPlan::generateMATLABTrajectory(GeneralData& generalData, int size_n,
        double z1Offset, double z2Offset,
        double offsetDistance
    )
    {
        // A regenerated sampling matrix invalidates every old cross-layer
        // segment. Remember whether the user had a connected bow path so it
        // can be rebuilt from the new points below instead of rendering stale
        // connectors over the new trajectory.
        const bool rebuildBowPath = generalData.trajData_.connectionData_.isConnected_;
        generalData.trajData_.connectionData_.connectedTrajectory_.clear();
        generalData.trajData_.connectionData_.connectionSegments_.clear();
        generalData.trajData_.connectionData_.isConnected_ = false;

        if (generalData.slicingData_.slicingisupdata)
        {
            // [Modified] convertToCuttingPlanes now only updates sliceResult.internalBoundaries
            // If transforms exist, it re-intersects transformed cutting planes with STL and rebuilds boundary lines
            if (!UniformSlicer::convertToCuttingPlanes(
                generalData.slicingData_.currentSliceResult_,
                &generalData.slicingData_.cuttingPlanes_,
                &generalData.modelDataManager_)) {
                return false;
            }

            // [Critical Fix] Store updated sliceResult again to ensure trajectory generation uses latest intersection data
            updateSlicingState(generalData, generalData.slicingData_.currentSliceResult_);
        }

        try {
            // 1. Check prerequisite dependency states
            if (!stlProcessor_ || !boundaryDetector_) {
                return false;
            }

            if (!isBoundaryDetected(generalData) &&
                !generalData.slicingData_.currentSliceResult_.isBoundarylessFallback) {
                return false;
            }

            if (generalData.slicingData_.cuttingPlanes_.empty()) {
                return false;
            }

            // 2. [Refactor] Prepare trajectory generation parameters - use new TrajectoryGenerationParams
            // [Fix] Read velocity parameters from generalData instead of hardcoded values
            TrajectoryGenerationParams trajParams = generalData.trajData_.generationParams_;

            // Update offset parameters (these may come from UI and should override existing values)
            trajParams.z1Offset = z1Offset;
            trajParams.z2Offset = z2Offset;
            trajParams.offsetDistance = offsetDistance;

            // Use safe defaults if velocities are not set
            if (trajParams.normalVelocity == 0.0) {
                trajParams.normalVelocity = 50.0;       // Default normal working trajectory speed
            }
            if (trajParams.transitionVelocity == 0.0) {
                trajParams.transitionVelocity = 50.0;   // Default transition speed
            }

            // Save parameter structure to generalData
            generalData.trajData_.generationParams_ = trajParams;

            // 3. Call TrajectoryGenerator to generate trajectory
            if (!trajectoryGenerator_) {
                trajectoryGenerator_ = new TrajectoryGenerator();
            }

            // Prepare upper/lower boundaries and slicing intersection segments
            const BoundaryLine& upperBoundary = generalData.boundaryData_.upperBoundary_;
            const BoundaryLine& lowerBoundary = generalData.boundaryData_.lowerBoundary_;
            const SliceResult& sliceResult = generalData.slicingData_.currentSliceResult_;

            if (sliceResult.internalBoundaries.empty()) {
                return false;
            }

            // Prepare mesh data
            ModelData tempModelData;
            const ModelData* modelData = nullptr;
            if (stlProcessor_ && STLProcessor::hasValidData(generalData.modelDataManager_)) {
                tempModelData.mainModelData_ = generalData.modelDataManager_.mainModelData_;
                modelData = &tempModelData;
            }

            // [Core] Call trajectory generation interface
            if (!trajectoryGenerator_->generateMATLABTrajectory(
                upperBoundary, lowerBoundary,
                sliceResult, trajParams, modelData,
                generalData.trajData_.trajectoryPoints_,
                generalData.trajData_.groupInfo_,
                generalData.boundaryData_,
                generalData.slicingData_,
                generalData.trajData_)) {
                return false;
            }

            if (generalData.trajData_.trajectoryPoints_.empty()) {
                return false;
            }

            // If previously connected, automatically rebuild bow path using the new point matrix
            if (rebuildBowPath) {
                const auto groupedTrajectories =
                    TrajectoryConnector::convertTrajectoryPointsToGroupedFormat(
                        generalData.trajData_.trajectoryPoints_,
                        generalData.trajData_.sampledBoundaryPoints_);
                if (groupedTrajectories.empty() ||
                    !TrajectoryConnector::connectTrajectoryPoints(
                        groupedTrajectories, ConnectionStrategy::BowPath,
                        generalData.trajData_)) {
                    return false;
                }
            }

            // 6. Update state flags
            generalData.processingState_.isTrajectoryGenerated = true;
            generalData.currentState_ = DataState::TrajectoryGenerated;

            return true;
        }
        catch (const std::exception& e) {
            return false;
        }
    }

    bool BurTrajPlan::exportTrajectoryToFile(const GeneralData& generalData, const std::string& filename) const
    {
        // [REFACTOR] Directly call TrajectoryExporter to export generic file
        return TrajectoryExporter::exportTrajectoryToFile(generalData.trajData_.trajectoryPoints_, filename);
    }

    bool BurTrajPlan::exportFANUCProgram(const GeneralData& generalData, const std::string& filename) const
    {
        // [REFACTOR] Export FANUC robot machining program
        return TrajectoryExporter::exportFANUCProgram(generalData.trajData_.trajectoryPoints_, filename);
    }

    bool BurTrajPlan::exportABBProgram(const GeneralData& generalData, const std::string& txtFilename, const std::string& modFilename) const
    {
        // [REFACTOR] Export ABB robot machining program (.mod / .txt)
        return TrajectoryExporter::exportABBProgram(generalData.trajData_.trajectoryPoints_, txtFilename, modFilename);
    }

    bool BurTrajPlan::exportToExcel(const GeneralData& generalData, const std::string& filename) const
    {
        // [REFACTOR] Export standard Excel format
        return TrajectoryExporter::exportToExcel(generalData.trajData_.trajectoryPoints_, filename);
    }

    bool BurTrajPlan::exportToExcelWithVelocity(const GeneralData& generalData, const std::string& filename) const
    {
        // [NEW] Export complete trajectory data report including position, orientation, velocity, acceleration, and timestamp
        return TrajectoryExporter::exportToExcelWithVelocity(generalData.trajData_.trajectoryPoints_, filename);
    }

    TrajectoryGenerator* BurTrajPlan::getTrajectoryGenerator() const
    {
        return trajectoryGenerator_;
    }

    // === Trajectory path connection functionality implementation ===

    // Connect discrete trajectory points to generate continuous tool path
    // strategy: Sequential, Optimized, ZigZag, LayerByLayer, BowPath, etc.
    bool BurTrajPlan::connectTrajectoryPoints(GeneralData& generalData, ConnectionStrategy strategy)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!trajectoryConnector_) {
            return false;
        }

        if (generalData.trajData_.trajectoryPoints_.empty()) {
            return false;
        }

        // Clear old connection data first to avoid accumulation and pollution
        generalData.trajData_.connectionData_.connectedTrajectory_.clear();
        generalData.trajData_.connectionData_.connectionSegments_.clear();
        generalData.trajData_.connectionData_.isConnected_ = false;

        // Clear offset boundary data (no longer needed after trajectory connection)
        if (boundaryDetector_) {
            boundaryDetector_->clearOffsetBoundaries(generalData.boundaryData_);
        }

        // [REFACTOR] Call TrajectoryConnector static method to execute path concatenation
        // 1. Convert discrete trajectory points into layered/grouped format
        std::vector<std::vector<TrajectoryPoint>> groupedTrajectories;

        // Prefer using sampled boundary points for grouping; if sampledBoundaryPoints_ exists, group by it
        if (!generalData.trajData_.sampledBoundaryPoints_.empty()) {
            groupedTrajectories = TrajectoryConnector::convertTrajectoryPointsToGroupedFormat(
                generalData.trajData_.trajectoryPoints_,
                generalData.trajData_.sampledBoundaryPoints_);
        }
        else {
            // Unable to safely connect if neither grouped data nor valid boundary reference exists
            return false;
        }

        if (groupedTrajectories.empty()) {
            return false;
        }

        // 2. Execute path connection algorithm
        bool success = TrajectoryConnector::connectTrajectoryPoints(
            groupedTrajectories,           // Input: grouped trajectory point matrices
            strategy,                      // Input: connection strategy
            generalData.trajData_          // Output: receive connected complete trajectory data
        );

        return success;
    }

    bool BurTrajPlan::exportConnectedTrajectory(const GeneralData& generalData, const std::string& filename) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Directly use complete connected trajectory points from GeneralData for export
        return TrajectoryExporter::exportTrajectoryToFile(generalData.trajData_.trajectoryPoints_, filename);
    }

    // [ALGORITHM] Execute boundary feature detection
    // Input: generalData.modelDataManager_.mainModelData_ - STL mesh
    // Output: generalData.boundaryData_ - detected upper and lower open boundaries
    //         generalData.processingState_.isBoundaryDetected - set to true
    bool BurTrajPlan::runBoundaryDetection(GeneralData& generalData)
    {
        // Ensure STL is processed and detector instance is available
        if (!generalData.processingState_.isSTLProcessed || !boundaryDetector_) {
            return false;
        }

        const PrivateModel& modelData = generalData.modelDataManager_.mainModelData_;

        ModelData tempModelData;
        tempModelData.mainModelData_ = modelData;

        if (!boundaryDetector_->detectBoundaries(
            tempModelData,
            generalData.boundaryData_,
            generalData.boundaryData_.intersectionPoints_)) {
            return false;
        }

        // [KEY] Mark boundary detection as successfully completed
        generalData.processingState_.isBoundaryDetected = true;

        return true;
    }

    // [NEW] Convert slice data to cutting planes - delegate to UniformSlicer
    void BurTrajPlan::convertSliceDataToCuttingPlanes(GeneralData& generalData)
    {
        // Note: This method is deprecated as sliceResult is now passed directly
    }

    // === Getter interface implementation ===
    void BurTrajPlan::getFilePaths(const GeneralData& generalData, std::vector<std::string>& filePaths) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        filePaths = generalData.filePaths_;
    }

    void BurTrajPlan::getCurrentState(const GeneralData& generalData, DataState& state) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state = generalData.currentState_;
    }

    void BurTrajPlan::getTrajectoryPoints(const GeneralData& generalData, std::vector<TrajectoryPoint>& points) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        points = generalData.trajData_.trajectoryPoints_;
    }

    void BurTrajPlan::getCuttingPlanes(const GeneralData& generalData, std::vector<CuttingPlane>& planes) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        planes = generalData.slicingData_.cuttingPlanes_;
    }

    // === Parameter configuration and query interface implementation ===
    bool BurTrajPlan::setSlicingSize(GeneralData& generalData, int size)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [ALGORITHM] Store layer count in generalData only
        generalData.slicingData_.managedParams_.numTrajectories = size;
        return true;
    }

    void BurTrajPlan::getSlicingSize(const GeneralData& generalData, int& size) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        size = generalData.slicingData_.managedParams_.numTrajectories;
    }

    bool BurTrajPlan::setZ1Offset(GeneralData& generalData, double offset)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        generalData.trajData_.generationParams_.z1Offset = offset;
        return true;
    }

    void BurTrajPlan::getZ1Offset(const GeneralData& generalData, double& offset) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        offset = generalData.trajData_.generationParams_.z1Offset;
    }

    bool BurTrajPlan::setZ2Offset(GeneralData& generalData, double offset)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        generalData.trajData_.generationParams_.z2Offset = offset;
        return true;
    }

    void BurTrajPlan::getZ2Offset(const GeneralData& generalData, double& offset) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        offset = generalData.trajData_.generationParams_.z2Offset;
    }

    bool BurTrajPlan::setOffsetDistance(GeneralData& generalData, double distance)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        generalData.trajData_.generationParams_.offsetDistance = distance;
        return true;
    }

    void BurTrajPlan::getOffsetDistance(const GeneralData& generalData, double& distance) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        distance = generalData.trajData_.generationParams_.offsetDistance;
    }

    bool BurTrajPlan::setnormalVelocity_(GeneralData& generalData, double Velocity)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [Fix] Correctly set normalVelocity (previously offsetDistance was set by mistake)
        generalData.trajData_.generationParams_.normalVelocity = Velocity;
        return true;
    }

    void BurTrajPlan::getnormalVelocity_(const GeneralData& generalData, double& Velocity) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [Fix] Correctly read normalVelocity
        Velocity = generalData.trajData_.generationParams_.normalVelocity;
    }

    bool BurTrajPlan::settransitionVelocity_(GeneralData& generalData, double Velocity)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [Fix] Correctly set transitionVelocity
        generalData.trajData_.generationParams_.transitionVelocity = Velocity;
        return true;
    }

    void BurTrajPlan::gettransitionVelocity_(const GeneralData& generalData, double& Velocity) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [Fix] Correctly read transitionVelocity (previously offsetDistance was read by mistake)
        Velocity = generalData.trajData_.generationParams_.transitionVelocity;
    }

    // === Preset parameter implementation ===
    bool BurTrajPlan::applyCoarsePreset(GeneralData& generalData)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Apply coarse machining preset parameters
        generalData.slicingData_.managedParams_.applyCoarse();
        return true;
    }

    // === Model and processing state query interface ===
    bool BurTrajPlan::isSTLProcessed(const GeneralData& generalData) const
    {
        return generalData.processingState_.isSTLProcessed;
    }

    bool BurTrajPlan::isBoundaryDetected(const GeneralData& generalData) const
    {
        return generalData.processingState_.isBoundaryDetected;
    }

} // namespace ModelTrajPlan
