#include <SlicingTrajPlan/BurTrajPlan.h>
#include <SlicingTrajPlan/TrajectoryGenerator.h>
#include <SlicingTrajPlan/UniformSlicer.h>

#include <cmath>
#include <iostream>

namespace
{
    ModelTrajPlan::ModelData makeClosedBox()
    {
        ModelTrajPlan::ModelData model;
        auto& mesh = model.mainModelData_;
        mesh.m_vertices = {
            -1.0F, -1.0F, 0.0F,  1.0F, -1.0F, 0.0F,  1.0F, 1.0F, 0.0F, -1.0F, 1.0F, 0.0F,
            -1.0F, -1.0F, 6.0F,  1.0F, -1.0F, 6.0F,  1.0F, 1.0F, 6.0F, -1.0F, 1.0F, 6.0F };
        mesh.m_indices = {
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
            0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5,
            2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 };
        return model;
    }
}

int main()
{
    ModelTrajPlan::BurTrajPlan plan;
    ModelTrajPlan::GeneralData document;
    if(!plan.setSlicingSize(document, 13) || !plan.setZ1Offset(document, 2.5) ||
        !plan.setZ2Offset(document, -1.5) || !plan.setOffsetDistance(document, 0.75)) return 1;
    int size = 0; double z1 = 0.0; double z2 = 0.0; double distance = 0.0;
    plan.getSlicingSize(document, size); plan.getZ1Offset(document, z1);
    plan.getZ2Offset(document, z2); plan.getOffsetDistance(document, distance);
    if(size != 13 || std::abs(z1 - 2.5) > 1e-12 || std::abs(z2 + 1.5) > 1e-12 ||
        std::abs(distance - 0.75) > 1e-12) {
        std::cerr << "BurTrajPlan parameters were not persisted in GeneralData.\n";
        return 2;
    }
    plan.clear(document);
    ModelTrajPlan::DataState state = ModelTrajPlan::DataState::Loaded;
    plan.getCurrentState(document, state);
    if(state != ModelTrajPlan::DataState::Empty) return 3;

    // A zero requested count means automatic count-from-spacing.  Regression
    // coverage ensures the generated count, rather than zero, is used by
    // trajectory group statistics.
    const ModelTrajPlan::ModelData model = makeClosedBox();
    ModelTrajPlan::SlicingParams slicingParams;
    slicingParams.mode = ModelTrajPlan::SlicingMode::AA;
    slicingParams.minStep = 1.0;
    slicingParams.trajectorySpacing = 1.0;
    slicingParams.numTrajectories = 0;
    slicingParams.maxSliceCount = 3;
    ModelTrajPlan::UniformSlicer slicer;
    ModelTrajPlan::SlicingLocation locations;
    ModelTrajPlan::Epiphyseal nodes;
    ModelTrajPlan::SliceResult sliceResult;
    std::vector<ModelTrajPlan::CuttingPlane> planes;
    if(!slicer.performSlicing(model, {}, {}, slicingParams, locations, nodes, sliceResult, &planes)) return 4;

    ModelTrajPlan::TrajectoryGenerator generator;
    ModelTrajPlan::TrajectoryGenerationParams generationParams;
    ModelTrajPlan::BoundaryData boundaries;
    ModelTrajPlan::SlicingData slicingData;
    ModelTrajPlan::TrajData trajectoryData;
    std::vector<ModelTrajPlan::TrajectoryPoint> trajectory;
    ModelTrajPlan::TrajectoryGroupInfo groupInfo;
    if(!generator.generateMATLABTrajectory({}, {}, sliceResult, generationParams, &model,
        trajectory, groupInfo, boundaries, slicingData, trajectoryData) ||
        trajectory.empty() || groupInfo.numTrajectories <= 0 || groupInfo.pointsPerTrajectory <= 0) {
        std::cerr << "Automatic trajectory count did not produce valid group statistics.\n";
        return 5;
    }
    // Deletion must update the persistent slice matrix and invalidate every
    // dependent trajectory without changing physical model boundaries.
    document.slicingData_.cuttingPlanes_ = planes;
    document.slicingData_.slicingLocations_ = locations;
    document.slicingData_.currentSliceResult_ = sliceResult;
    document.slicingData_.sliceLines_ = sliceResult.internalBoundaries;
    document.trajData_ = trajectoryData;
    document.trajData_.connectionData_.isConnected_ = true;
    document.processingState_.isTrajectoryGenerated = true;
    document.processingState_.isTrajectoryConnected = true;
    document.boundaryData_.lowerBoundary_.points = sliceResult.internalBoundaries.front().points;
    const auto boundaryCount = document.boundaryData_.lowerBoundary_.points.size();
    const auto originalCount = planes.size();
    const double survivingZ = planes[2].center.z();
    if (plan.deleteCuttingPlane(document, -1) || plan.deleteCuttingPlane(document, int(originalCount))) return 6;
    if (!plan.deleteCuttingPlane(document, 1)) return 7;
    const auto& remaining = document.slicingData_;
    if (remaining.cuttingPlanes_.size() != originalCount - 1 ||
        remaining.currentSliceResult_.internalBoundaries.size() != originalCount - 1 ||
        remaining.slicingLocations_.size() != originalCount - 1 ||
        remaining.cuttingPlanes_[1].center.z() != survivingZ ||
        remaining.cuttingPlanes_[1].layer_index != 1 || !remaining.sliceLines_.empty() ||
        !document.trajData_.trajectoryPoints_.empty() || !document.trajData_.sampledBoundaryPoints_.empty() ||
        document.trajData_.connectionData_.isConnected_ || document.processingState_.isTrajectoryGenerated ||
        document.processingState_.isTrajectoryConnected ||
        document.boundaryData_.lowerBoundary_.points.size() != boundaryCount) return 8;
    if (!generator.generateMATLABTrajectory({}, {}, remaining.currentSliceResult_, generationParams, &model,
        trajectory, groupInfo, document.boundaryData_, document.slicingData_, document.trajData_)) return 9;
    for (const auto& column : document.trajData_.sampledBoundaryPoints_)
        if (column.size() != originalCount - 1) return 10;
    while (!document.slicingData_.cuttingPlanes_.empty())
        if (!plan.deleteCuttingPlane(document, int(document.slicingData_.cuttingPlanes_.size()) - 1)) return 11;
    if (document.slicingData_.lastSelectedLayerIndex != -1 || document.processingState_.isSlicingCompleted ||
        !document.slicingData_.currentSliceResult_.internalBoundaries.empty()) return 12;
    return 0;
}
