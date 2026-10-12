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
    return 0;
}
