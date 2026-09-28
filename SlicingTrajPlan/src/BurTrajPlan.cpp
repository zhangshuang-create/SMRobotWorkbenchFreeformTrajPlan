#include "BurTrajPlan.h"
#include <TrajectoryGenerator/TrajectoryGenerator.h>
#include <ModelData/ModelData.h> // 
#include <algorithm>
#include <iostream>
#include <cmath>
#include <memory>
#include <limits>
#include <STLProcessor/STLProcessor.h>
#include <BoundaryDetector/BoundaryDetector.h>
#include <UniformSlicer/UniformSlicer.h>
#include <TrajectoryConnector/TrajectoryConnector.h>
#include <TrajectoryGenerator/TrajectoryGenerator.h>
#include <TrajectoryFrame/TrajectoryFrame.h>
#include <QDebug>
namespace ModelTrajPlan
{
    // �����˶�����ʵ��������ֵ����Ӧ������ָ���Ա����
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


    // ����������PublicModel�������ݵ�GeneralData
    bool BurTrajPlan::loadModelFromPublicData(const PublicData::PublicModel& publicModel, GeneralData& generalData)
    {
        // �������Ǹոո�д�ľ�̬���ߺ���

        generalData.hasOriginalData_ = true;

        // Mark STL as processed
        generalData.processingState_.isSTLProcessed = true;
        return STLProcessor::copyFromPublicModel(publicModel, generalData.modelDataManager_);


    }

    void BurTrajPlan::clear(GeneralData& generalData)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        generalData.filePaths_.clear();
        generalData.modelDataManager_.allModelsData_.clear();

        // ����켣����
        generalData.trajData_.trajectoryPoints_.clear();
        generalData.trajData_.sampledBoundaryPoints_.clear();  // ���������������������

        // �����Ƭ����
        generalData.slicingData_.cuttingPlanes_.clear();
        generalData.slicingData_.slicingLocations_.clear();
        generalData.slicingData_.sliceLines_.clear();  // �������������Ƭ�߽���
        generalData.slicingData_.currentSliceResult_ = SliceResult();  // ��������������Ƭ���
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



    bool BurTrajPlan::generateCuttingPlanes(GeneralData& generalData, RenderData* renderData)
    {
        // [ALGORITHM] Generate cutting planes from model
        // Input: generalData.modelDataManager_.mainModelData_ - STL model
        //        generalData.slicingData_.managedParams_ - slicing parameters
        // Output: generalData.slicingData_.cuttingPlanes_ - generated cutting planes

        // 1. Check preconditions
        if (!generalData.processingState_.isSTLProcessed) {
            return false;
        }

        // 2. Ensure boundary is detected
        if (!generalData.processingState_.isBoundaryDetected) {
            if (!runBoundaryDetection(generalData, renderData)) {
                return false;
            }
        }

        // Rebuild adaptive slicing nodes every run so UI-tuned epsilon/minStep take effect immediately.
        if (generalData.slicingData_.managedParams_.mode == SlicingMode::AA &&
            boundaryDetector_ && !generalData.boundaryData_.intersectionPoints_.empty()) {
            Epiphyseal adaptiveNodes;
            const bool adaptiveOk = boundaryDetector_->generateAdaptiveNodes(
                generalData.boundaryData_.intersectionPoints_,
                adaptiveNodes,
                generalData.boundaryData_.epsilon_,
                generalData.boundaryData_.minStep_);

            Epiphyseal slicingNodes;
            const bool slicingNodeOk = adaptiveOk && boundaryDetector_->buildSlicingNodes(
                generalData.boundaryData_.intersectionPoints_,
                adaptiveNodes,
                slicingNodes,
                generalData.boundaryData_.minStep_);

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

        // 5. Execute slicing - pure module call
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
        // Prefer adaptive slicing nodes; fallback to raw intersection lines if adaptive nodes are unavailable.
        Epiphyseal* slicingNodeInput = &generalData.boundaryData_.Mainslicingnode;
        if (slicingNodeInput->size() < 3) {
            std::cerr << "[BurTrajPlan] Fallback to intersectionPoints_ because Mainslicingnode is invalid. "
                << "This may reduce epsilon/min_step sensitivity. "
                << "Mainslicingnode.size=" << generalData.boundaryData_.Mainslicingnode.size()
                << ", epsilon=" << generalData.boundaryData_.epsilon_
                << ", minStep=" << generalData.boundaryData_.minStep_
                << std::endl;
            slicingNodeInput = &generalData.boundaryData_.intersectionPoints_;
        }
        // [Modified] performSlicing can now generate base cuttingPlanes at the same time
        // Pass oldCuttingPlanes to preserve transform parameters
        if (!uniformSlicer_->performSlicing(tempModelData2, upperBoundary, lowerBoundary, params,
            generalData.slicingData_.slicingLocations_,
            *slicingNodeInput,
            sliceResult,
            &generalData.slicingData_.cuttingPlanes_,
            oldCuttingPlanes.empty() ? nullptr : &oldCuttingPlanes)) {
            qWarning() << "[BurTrajPlan] generateCuttingPlanes: UniformSlicer::performSlicing returned false"
                       << "mode=" << static_cast<int>(params.mode)
                       << "upperPoints=" << upperBoundary.points.size()
                       << "lowerPoints=" << lowerBoundary.points.size()
                       << "minStep=" << params.minStep;
            return false;
        }

        // 6. Store slice result
        generalData.slicingData_.currentSliceResult_ = sliceResult;

        // Update slicing state
        updateSlicingState(generalData, sliceResult);

        return true;
    }

    // [NEW] Dedicated function to clear cutting plane data
    void BurTrajPlan::clearCuttingPlanesData(GeneralData& generalData)
    {
        generalData.slicingData_.cuttingPlanes_.clear();
        generalData.slicingData_.slicingLocations_.clear();
        generalData.slicingData_.sliceLines_.clear();  // [Addition] Clear slicing boundary lines
        generalData.trajData_.trajectoryPoints_.clear();
        generalData.trajData_.sampledBoundaryPoints_.clear();  // [Addition] Clear sampled point data
    }

    // [ALGORITHM] Prepare slicing parameters from generalData
    // Input: generalData.slicingData_.managedParams_ - parameter settings
    //        upperBoundary, lowerBoundary - for Z validation
    // Output: SlicingParams - validated parameters ready for slicing
    SlicingParams BurTrajPlan::prepareSlicingParams(
        const GeneralData& generalData,
        const BoundaryLine& upperBoundary,
        const BoundaryLine& lowerBoundary)
    {
        SlicingParams params;
        // ���ع���ֻ��ȡ��Ƭ��ز���
        params.numTrajectories = generalData.slicingData_.managedParams_.numTrajectories;
        params.trajectorySpacing = generalData.slicingData_.managedParams_.trajectorySpacing;
        params.minStep = generalData.boundaryData_.minStep_;
        params.mode = generalData.slicingData_.managedParams_.mode;
        // ƫ�Ʋ������Ƴ������ٴ� SlicingParams ��ȡ

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

    // [NEW] Dedicated function to validate Z positions
    std::vector<double> BurTrajPlan::validateZPositions(const std::vector<double>& zPositions,
        const BoundaryLine& upperBoundary,
        const BoundaryLine& lowerBoundary)
    {
        // Calculate Z range
        double zMin = std::numeric_limits<double>::max();
        double zMax = std::numeric_limits<double>::lowest();

        for (const auto& p : lowerBoundary.points) zMin = std::min(zMin, p.position.z());
        for (const auto& p : upperBoundary.points) zMax = std::max(zMax, p.position.z());

        // Sort, deduplicate, filter
        std::vector<double> validZPositions = zPositions;
        std::sort(validZPositions.begin(), validZPositions.end());
        validZPositions.erase(std::unique(validZPositions.begin(), validZPositions.end()), validZPositions.end());

        validZPositions.erase(
            std::remove_if(validZPositions.begin(), validZPositions.end(),
                [zMin, zMax](double z) { return (z < zMin || z > zMax); }),
            validZPositions.end());

        return validZPositions;
    }

    // [ALGORITHM] Update slicing state after slicing completes
    // Input: sliceResult - slice result from algorithm
    // Output: generalData.processingState_.isSlicingCompleted - state flag
    //         generalData.slicingData_.managedParams_.customZPositions - Z positions
    void BurTrajPlan::updateSlicingState(GeneralData& generalData, const SliceResult& sliceResult)
    {
        generalData.processingState_.isSlicingCompleted = true;
        generalData.currentState_ = DataState::EdgeExtracted;

        // Write back Z layer position information to generalData
        generalData.slicingData_.managedParams_.customZPositions.clear();
        for (const auto& b : sliceResult.internalBoundaries) {
            generalData.slicingData_.managedParams_.customZPositions.push_back(b.z_level);
        }
    }

    // === Rendering implementation (receives RenderData as parameter) ===
    bool BurTrajPlan::initializeRenderer(RenderData& renderData)
    {
        // Initialize renderers for all processing modules

        // Initialize STL processing module renderer
        if (stlProcessor_) {
            stlProcessor_->initializeRenderer(renderData.stl());
            // Note: Visibility will be set later when data is loaded
        }

        if (boundaryDetector_) {
            boundaryDetector_->initializeRenderer(renderData.boundary());
        }

        if (uniformSlicer_) {
            uniformSlicer_->initializeRenderer(renderData.slicing());
        }

        if (trajectoryGenerator_) {
            trajectoryGenerator_->initializeRenderer(renderData.trajectory());
        }

        if (trajectoryConnector_) {
            trajectoryConnector_->initializeRenderer(renderData.connection());
        }


        return true;
    }

    bool BurTrajPlan::draw(const GeneralData& generalData, RenderData& renderData)
    {
        // Render data from STL processing module
        bool result = true;

        // Draw original data (STL model)
        if (stlProcessor_ && STLProcessor::hasValidData(generalData.modelDataManager_) && renderData.stl().isVisible) {
            result &= stlProcessor_->draw(renderData.stl(), generalData.modelDataManager_);
        }

        // Draw boundary lines (different colors)
        if (boundaryDetector_ && BoundaryDetector::hasRenderableData(generalData.boundaryData_) && renderData.boundary().isVisible) {
            result &= boundaryDetector_->draw(renderData.boundary(), generalData.boundaryData_);
        }

        // Offset boundary visibility syncs with original boundary: only show when both are enabled
        if (boundaryDetector_ && BoundaryDetector::hasRenderableData(generalData.boundaryData_) &&
            renderData.boundary().isVisible && renderData.boundary().isOffsetVisible) {
            boundaryDetector_->drawOffsetBoundaries(renderData.boundary(), generalData.boundaryData_);
        }

        // Draw trajectory point models (if exists and visible) - use passed data
        if (!generalData.trajData_.trajectoryPoints_.empty() && renderData.trajectory().isVisible) {
            // [REFACTOR] Use TrajectoryGenerator::draw instead of prepareTrajectoryRenderFromData
            if (trajectoryGenerator_ && renderData.trajectory().trajectoryModel) {
                // Call TrajectoryGenerator to handle rendering update
                trajectoryGenerator_->draw(renderData.trajectory(), generalData.trajData_.trajectoryPoints_, 30.0f);
                result = true;
            }
        }

        // Draw cutting planes
        if (!generalData.slicingData_.cuttingPlanes_.empty() && renderData.slicing().isVisible) {
            uniformSlicer_->draw(renderData.slicing(), generalData.slicingData_);
            result = true;
        }

        // Draw trajectory connection lines (green=intra-trajectory, orange=inter-trajectory)
        if (trajectoryConnector_ && TrajectoryConnector::hasRenderableData(generalData.trajData_) && renderData.connection().isVisible) {
            bool connectionDrawn = TrajectoryConnector::draw(renderData.connection(), generalData.trajData_);
            if (connectionDrawn) {
                result = true;
            }
        }

        return true;
    }

    // === Convenience interface implementation ===


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
        // ͳ��ProcessingInput�еĴ�ͳmesh_triangles�����ڵ������ݣ�
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


    // === Visibility control implementation (receives RenderData as parameter) ===
    void BurTrajPlan::setOriginalDataVisibility(RenderData& renderData, bool visible)
    {
        // Set STL data visibility (via RenderData)
        if (stlProcessor_) {
            stlProcessor_->setVisibility(renderData.stl(), visible);
        }
    }



    void BurTrajPlan::setTrajectoryVisibility(RenderData& renderData, bool visible)
    {
        renderData.trajectory().isVisible = visible;
    }

    void BurTrajPlan::setCuttingPlanesVisibility(RenderData& renderData, bool visible)
    {
        renderData.slicing().isVisible = visible;
    }

    void BurTrajPlan::setBoundaryVisibility(RenderData& renderData, bool visible)
    {
        if (boundaryDetector_) {
            boundaryDetector_->setBoundaryVisibility(renderData.boundary(), visible);
        }
    }

    void BurTrajPlan::setOffsetBoundaryVisibility(RenderData& renderData, bool visible)
    {
        if (boundaryDetector_) {
            boundaryDetector_->setOffsetBoundaryVisibility(renderData.boundary(), visible);
        }
    }



    void BurTrajPlan::clearCuttingPlanes(GeneralData& generalData)
    {
        // [ALGORITHM] Clear cutting planes while preserving STL and boundary state
        // Input: generalData.processingState_ - current state
        // Output: generalData.slicingData_.cuttingPlanes_ - cleared
        //         generalData.processingState_.isSlicingCompleted - set to false

        // Save important state flags
        bool savedSTLProcessed = generalData.processingState_.isSTLProcessed;
        bool savedBoundaryDetected = generalData.processingState_.isBoundaryDetected;

        // Clear data
        generalData.slicingData_.cuttingPlanes_.clear();
        generalData.slicingData_.slicingLocations_.clear();
        generalData.slicingData_.sliceLines_.clear();  // �������������Ƭ�߽���
        generalData.trajData_.trajectoryPoints_.clear();
        generalData.trajData_.sampledBoundaryPoints_.clear();  // ���������������������
        // A new slice set invalidates the connected path as well.  Leaving
        // this cache populated makes a second confirmation display/export the
        // previous trajectory even though its cutting planes were replaced.
        generalData.trajData_.connectionData_.connectedTrajectory_.clear();
        generalData.trajData_.connectionData_.connectionSegments_.clear();
        generalData.trajData_.connectionData_.isConnected_ = false;

        if (boundaryDetector_) {
            boundaryDetector_->clearOffsetBoundaries(generalData.boundaryData_);
        }

        // Restore preserved states, only reset slicing state
        generalData.processingState_.isSTLProcessed = savedSTLProcessed;
        generalData.processingState_.isBoundaryDetected = savedBoundaryDetected;
        generalData.processingState_.isSlicingCompleted = false;
        generalData.processingState_.isTrajectoryGenerated = false;
        generalData.processingState_.isTrajectoryConnected = false;
    }

    // [Addition] Update size of a single cutting plane
    bool BurTrajPlan::updateSingleCuttingPlaneSize(GeneralData& generalData, int layerIndex)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (layerIndex < 0 || layerIndex >= static_cast<int>(generalData.slicingData_.cuttingPlanes_.size())) {
            return false;
        }

        if (!generalData.processingState_.isSTLProcessed) {
            return false;
        }

        // If a previously selected layer exists and is different from current layer, deselect it
        int& lastSelected = generalData.slicingData_.lastSelectedLayerIndex;
        if (lastSelected != -1 &&
            lastSelected != layerIndex &&
            lastSelected < static_cast<int>(generalData.slicingData_.cuttingPlanes_.size())) {
            generalData.slicingData_.cuttingPlanes_[lastSelected].isSelected = 0;
        }

        // Select current layer
        CuttingPlane& plane = generalData.slicingData_.cuttingPlanes_[layerIndex];
        plane.isSelected = 1;

        // Update record
        lastSelected = layerIndex;

        // [Fix] plane.center is already the original center (without translation), use directly
        Eigen::Vector3d originalCenter = plane.center;

        // Prepare ModelData
        ModelData tempModelData;
        tempModelData.mainModelData_ = generalData.modelDataManager_.mainModelData_;

        // Call UniformSlicer static method to update size
        return UniformSlicer::updateSingleCuttingPlaneSize(plane, tempModelData, originalCenter, generalData.slicingData_.slicingisupdata);
    }



    // === Custom Z position set/get ===
    bool BurTrajPlan::setCustomZPositions(GeneralData& generalData, const std::vector<double>& zPositions)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [ALGORITHM] Store custom Z positions in generalData
        generalData.slicingData_.managedParams_.customZPositions = zPositions;
        return true;
    }

    void BurTrajPlan::getCustomZPositions(const GeneralData& generalData, std::vector<double>& zPositions) const
    {
        // [QUERY] Read custom Z positions from generalData
        zPositions = generalData.slicingData_.managedParams_.customZPositions;
    }

    void BurTrajPlan::setKeepOriginalVisibleAfterPreprocess(bool keep)
    {
        // Note: This is a UI preference, could be moved to GeneralData
        // For now, we keep it simple
        std::lock_guard<std::mutex> lock(mutex_);
        // Store in a temporary location or pass through parameters
    }



    // === Trajectory generation interface implementation ===
    bool BurTrajPlan::generateMATLABTrajectory(GeneralData& generalData, int size_n,
        double z1Offset, double z2Offset,
        double offsetDistance,
        PublicData::TrajData& publicTrajData
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

        //
        //
        if (generalData.slicingData_.slicingisupdata)
        {
            // [Modified] convertToCuttingPlanes now only updates sliceResult.internalBoundaries
            // If transforms exist, it re-intersects transformed cutting planes with STL and uses traceContoursFromIntersections to build boundary lines
            if (!UniformSlicer::convertToCuttingPlanes(
                generalData.slicingData_.currentSliceResult_,
                &generalData.slicingData_.cuttingPlanes_,
                &generalData.modelDataManager_)) {
                return false;
            }

            // [Critical Fix] Store updated sliceResult again to ensure trajectory generation uses latest intersection data
            // sliceResult.internalBoundaries has been updated by convertToCuttingPlanes
            updateSlicingState(generalData, generalData.slicingData_.currentSliceResult_);
        }
        //
        //
        try {
            // 1. Check preconditions
            if (!stlProcessor_ || !boundaryDetector_) {
                return false;
            }

            if (!isBoundaryDetected(generalData)) {
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

            // normalVelocity and transitionVelocity should already be set via setnormalVelocity_ and settransitionVelocity_
            // Use defaults if not set
            if (trajParams.normalVelocity == 0.0) {
                trajParams.normalVelocity = 50.0;       // Default normal trajectory speed
            }
            if (trajParams.transitionVelocity == 0.0) {
                trajParams.transitionVelocity = 50.0;   // Default transition speed
            }

            // Save parameters to generalData (no longer includes slicing parameters)
            generalData.trajData_.generationParams_ = trajParams;

            // 3. Call TrajectoryGenerator - using the new interface. Boundary
            // empty-run layers are derived there from adjacent sampled slices,
            // not by applying a global-Z boundary shift.
            if (!trajectoryGenerator_) {
                trajectoryGenerator_ = new TrajectoryGenerator();
            }

            // Prepare required data - [Key] use only original boundaries
            const BoundaryLine& upperBoundary = generalData.boundaryData_.upperBoundary_;
            const BoundaryLine& lowerBoundary = generalData.boundaryData_.lowerBoundary_;
            const SliceResult& sliceResult = generalData.slicingData_.currentSliceResult_;

            if (sliceResult.internalBoundaries.empty()) {
                return false;
            }

            // Prepare model data
            ModelData tempModelData;
            const ModelData* modelData = nullptr;
            if (stlProcessor_ && STLProcessor::hasValidData(generalData.modelDataManager_)) {
                tempModelData.mainModelData_ = generalData.modelDataManager_.mainModelData_;
                modelData = &tempModelData;
            }

            // [Core] Use new interface: explicit input/output and return results via reference parameters
            // Input: upperBoundary, lowerBoundary, sliceResult, trajParams, modelData
            // Output: generalData.trajData_.referenceLines_, trajectoryPoints_, groupInfo_
            // [Addition] Also pass boundaryData, slicingData, and trajData to place sliceLines_ into boundaryLines_ and store sampled points
            if (!trajectoryGenerator_->generateMATLABTrajectory(
                upperBoundary, lowerBoundary,
                sliceResult, trajParams, modelData,
                generalData.trajData_.trajectoryPoints_,
                generalData.trajData_.groupInfo_,
                generalData.boundaryData_,
                generalData.slicingData_,
                generalData.trajData_,
                publicTrajData)) {
                return false;
            }

            if (generalData.trajData_.trajectoryPoints_.empty()) {
                return false;
            }

            // Preserve the existing bow-path topology after a phase-only
            // resample. TrajectoryGenerator keeps even columns bottom-to-top
            // and odd columns top-to-bottom; the connector rebuilds the same
            // cap-to-cap / bottom-to-bottom links from that ordering.
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

            // 6. Update state
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
        // [REFACTOR] Directly call TrajectoryExporter module
        return TrajectoryExporter::exportTrajectoryToFile(generalData.trajData_.trajectoryPoints_, filename);
    }

    bool BurTrajPlan::exportFANUCProgram(const GeneralData& generalData, const std::string& filename) const
    {
        // [REFACTOR] Directly call TrajectoryExporter module
        return TrajectoryExporter::exportFANUCProgram(generalData.trajData_.trajectoryPoints_, filename);
    }

    bool BurTrajPlan::exportABBProgram(const GeneralData& generalData, const std::string& txtFilename, const std::string& modFilename) const
    {
        // [REFACTOR] Directly call TrajectoryExporter module
        return TrajectoryExporter::exportABBProgram(generalData.trajData_.trajectoryPoints_, txtFilename, modFilename);
    }

    bool BurTrajPlan::exportToExcel(const GeneralData& generalData, const std::string& filename) const
    {
        // [REFACTOR] Directly call TrajectoryExporter module
        return TrajectoryExporter::exportToExcel(generalData.trajData_.trajectoryPoints_, filename);
    }

    bool BurTrajPlan::exportToExcelWithVelocity(const GeneralData& generalData, const std::string& filename) const
    {
        // [NEW] Export complete trajectory data including position, orientation, velocity, acceleration, timestamp
        return TrajectoryExporter::exportToExcelWithVelocity(generalData.trajData_.trajectoryPoints_, filename);
    }

    TrajectoryGenerator* BurTrajPlan::getTrajectoryGenerator() const
    {
        return trajectoryGenerator_;
    }

    // === Trajectory connection functionality implementation ===

    // Connect trajectory points to generate complete path
    // strategy options: Sequential, Optimized, ZigZag, LayerByLayer
    bool BurTrajPlan::connectTrajectoryPoints(GeneralData& generalData, ConnectionStrategy strategy)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!trajectoryConnector_) {
            return false;
        }

        if (generalData.trajData_.trajectoryPoints_.empty()) {
            return false;
        }

        // Clear old connection data first to avoid accumulation
        generalData.trajData_.connectionData_.connectedTrajectory_.clear();
        generalData.trajData_.connectionData_.connectionSegments_.clear();
        generalData.trajData_.connectionData_.isConnected_ = false;

        // Clear offset boundary data, no longer needed after trajectory connection
        if (boundaryDetector_) {
            boundaryDetector_->clearOffsetBoundaries(generalData.boundaryData_);
        }

        // [REFACTOR] Call TrajectoryConnector static method for connection

        // 1. Convert trajectory points to grouped format
        std::vector<std::vector<TrajectoryPoint>> groupedTrajectories;

        // ������ʹ�������ݡ���� sampledBoundaryPoints_ ���ڣ�ʹ���������� trajectoryPoints_
        if (!generalData.trajData_.sampledBoundaryPoints_.empty()) {
            groupedTrajectories = TrajectoryConnector::convertTrajectoryPointsToGroupedFormat(
                generalData.trajData_.trajectoryPoints_,
                generalData.trajData_.sampledBoundaryPoints_);
        }
        else {
            // �����û��������Ҳû�о����ݣ��޷�����
            return false;
        }

        if (groupedTrajectories.empty()) {
            return false;
        }

        // 2. Execute connection (static method, all data through parameters)
        bool success = TrajectoryConnector::connectTrajectoryPoints(
            groupedTrajectories,           // Input: grouped trajectories
            strategy,                      // Input: connection strategy
            generalData.trajData_          // Output: connection result
        );

        return success;
    }



    // Control connection line visibility, supports adjustable line width and different colors
    bool BurTrajPlan::setConnectionVisibility(RenderData& renderData, bool visible)
    {
        TrajectoryConnector::setConnectionVisibility(renderData.connection(), visible);
        return true;
    }



    bool BurTrajPlan::exportConnectedTrajectory(const GeneralData& generalData, const std::string& filename) const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Directly use trajectory data from GeneralData for export
        return TrajectoryExporter::exportTrajectoryToFile(generalData.trajData_.trajectoryPoints_, filename);
    }



    // [ALGORITHM] Execute boundary detection
    // Input: generalData.modelDataManager_.mainModelData_ - STL model
    //        numTrajectories - number of boundary points to generate
    // Output: generalData.boundaryData_ - upper/lower boundaries
    //         generalData.processingState_.isBoundaryDetected - state flag
    bool BurTrajPlan::runBoundaryDetection(GeneralData& generalData, RenderData* renderData)
    {
        // Ensure STL is processed and detector is available
        if (!generalData.processingState_.isSTLProcessed || !boundaryDetector_) {
            return false;
        }

        const PrivateModel& modelData = generalData.modelDataManager_.mainModelData_;

        // Execute boundary detection algorithm
        ModelData tempModelData;
        tempModelData.mainModelData_ = modelData;

        if (!boundaryDetector_->detectBoundaries(
            tempModelData,
            generalData.boundaryData_,
            generalData.boundaryData_.intersectionPoints_,
            renderData->boundary().intersectionPointsData_)) {
            return false;
        }

        // [KEY] Set state flag to indicate boundary detection is complete
        generalData.processingState_.isBoundaryDetected = true;

        return true;
    }

    // [NEW] Convert slice data to cutting planes - delegate to UniformSlicer
    void BurTrajPlan::convertSliceDataToCuttingPlanes(GeneralData& generalData)
    {
        // Note: This method is deprecated as sliceResult is now passed directly
        // Slice result should be stored in generalData if needed
    }

    // === Getter function implementation from interface ===
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

    // === Missing interface implementation ===



    bool BurTrajPlan::needsRenderUpdate(const RenderData& renderData) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return renderData.stl().needsUpdate ||
            renderData.boundary().needsUpdate ||
            renderData.slicing().needsUpdate ||
            renderData.trajectory().needsUpdate ||
            renderData.connection().needsUpdate;
    }

    bool BurTrajPlan::hasRenderableData(const GeneralData& generalData, const RenderData& renderData) const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check if original data exists and is visible
        if (renderData.stl().isVisible) {
            if (stlProcessor_ && STLProcessor::hasValidData(generalData.modelDataManager_)) {
                return true;
            }
            if (!generalData.originalData_.empty()) {
                return true;
            }
        }

        // Check if trajectory data exists and is visible
        if (renderData.trajectory().isVisible && !generalData.trajData_.trajectoryPoints_.empty()) {
            return true;
        }

        // Check if cutting plane data exists and is visible
        if (renderData.slicing().isVisible && !generalData.slicingData_.cuttingPlanes_.empty()) {
            return true;
        }

        // Check if connection line data exists and is visible (independent of trajectory point visibility)
        if (trajectoryConnector_ && TrajectoryConnector::hasRenderableData(generalData.trajData_) && renderData.connection().isVisible) {
            return true;
        }

        return false;
    }

    // === Parameter management interface implementation ===
    bool BurTrajPlan::setSlicingSize(GeneralData& generalData, int size)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // [ALGORITHM] Store parameter in generalData only
        // Input: size - slicing size value
        // Output: generalData.slicingData_.managedParams_.numTrajectories
        generalData.slicingData_.managedParams_.numTrajectories = size;

        return true;
    }

    void BurTrajPlan::getSlicingSize(const GeneralData& generalData, int& size) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [REFACTOR] Get from GeneralData first
        size = generalData.slicingData_.managedParams_.numTrajectories;
    }

    bool BurTrajPlan::setZ1Offset(GeneralData& generalData, double offset)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // [Refactor] Store into TrajectoryGenerationParams
        generalData.trajData_.generationParams_.z1Offset = offset;

        return true;
    }

    void BurTrajPlan::getZ1Offset(const GeneralData& generalData, double& offset) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [Refactor] Read from TrajectoryGenerationParams
        offset = generalData.trajData_.generationParams_.z1Offset;
    }

    bool BurTrajPlan::setZ2Offset(GeneralData& generalData, double offset)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // [Refactor] Store into TrajectoryGenerationParams
        generalData.trajData_.generationParams_.z2Offset = offset;

        return true;
    }

    void BurTrajPlan::getZ2Offset(const GeneralData& generalData, double& offset) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [Refactor] Read from TrajectoryGenerationParams
        offset = generalData.trajData_.generationParams_.z2Offset;
    }

    bool BurTrajPlan::setOffsetDistance(GeneralData& generalData, double distance)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // [Refactor] Store into TrajectoryGenerationParams
        generalData.trajData_.generationParams_.offsetDistance = distance;

        return true;
    }

    void BurTrajPlan::getOffsetDistance(const GeneralData& generalData, double& distance) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // [Refactor] Read from TrajectoryGenerationParams
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
        // [Fix] Correctly read normalVelocity (previously offsetDistance was read by mistake)
        Velocity = generalData.trajData_.generationParams_.normalVelocity;
    }


    bool BurTrajPlan::settransitionVelocity_(GeneralData& generalData, double Velocity)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // [Fix] Correctly set transitionVelocity (previously offsetDistance was set by mistake)
        generalData.trajData_.generationParams_.transitionVelocity = Velocity;

        return true;
    }

    void BurTrajPlan::gettransitionVelocity_(const GeneralData& generalData, double& Velocity) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // ���޸�����ȷ��ȡ transitionVelocity��֮ǰ����ض�ȡ�� offsetDistance��
        Velocity = generalData.trajData_.generationParams_.transitionVelocity;
    }





    // === Preset parameter implementation ===
    bool BurTrajPlan::applyCoarsePreset(GeneralData& generalData)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // [ALGORITHM] Apply coarse preset to generalData only
        generalData.slicingData_.managedParams_.applyCoarse();

        return true;
    }

    // === STL processing state interface implementation ===
    bool BurTrajPlan::isSTLProcessed(const GeneralData& generalData) const
    {
        // [QUERY] Read state from generalData, not from internal member
        return generalData.processingState_.isSTLProcessed;
    }

    bool BurTrajPlan::isBoundaryDetected(const GeneralData& generalData) const
    {
        // [QUERY] Read state from generalData, not from internal member
        return generalData.processingState_.isBoundaryDetected;
    }

}
