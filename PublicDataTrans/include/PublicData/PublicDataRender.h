#pragma once

#include <PublicDataRender/PublicDataRender_global.h>

#include <ModelData/ModelData.h>
#include <TrajData/TrajData.h>

#include <ModelTangQing/iscene.h>
#include <ModelTangQing/imodel.h>
#include <vector>

namespace PublicData
{
    // PublicDataRender's own render-data containers (no dependency on ATrajPlan::RenderData).
    struct  PublicSTLRenderData
    {
        imodel_spointer meshModel;
        MeshData meshData;
        bool isVisible = true;
        bool needsUpdate = true;
    };

    struct  PublicTrajectoryRenderData
    {
        imodel_spointer trajectoryModel;
        MeshData trajectoryData;
        bool isVisible = true;
        bool needsUpdate = true;
    };

    struct PublicThicknessRenderData
    {
    imodel_spointer heatmapModel;
    MeshData heatmapMeshData;
    // Scalar value for each vertex in heatmapMeshData (same indexing).
    std::vector<double> vertexThickness;

    bool isVisible = false;
    bool needsUpdate = true;

    double minValue = 0.0;
    double maxValue = 0.0;
    double targetValue = 0.0;

    std::vector<double> legendValues;
    std::vector<float> legendColors; // RGB triples, size = legendValues.size() * 3
    };


    class PublicDataRender_SHAREDLIB PublicDataRender
    {
    public:
        // 声明构造和析构函数
        PublicDataRender();
        ~PublicDataRender();
    PublicThicknessRenderData m_PublicThicknessRenderData;
    PublicSTLRenderData m_PublicSTLRenderData;
    PublicTrajectoryRenderData m_PublicTrajectoryRenderData;

        /// Clear all MeshData buffers (see MeshData.h: m_vertices, m_normals, m_colors, m_indices).
        static void clearMeshData(MeshData& meshData);

    static void convertPublicModelToMeshData(const PublicModel& model, MeshData& meshData);
        static void convertPublicModelToColoredMeshData(
            const PublicModel& model,
            const std::vector<float>& vertexColors,
            MeshData& meshData);
        /// Same mesh layout as ModelTrajPlan::TrajectoryFrame::convertTrajectoryPointsToModelData:
        /// per pose, three colored axis segments (RGB) from rotation columns, length \p frameScale.
        static void convertPublicTrajToFrameMeshData(
            const PublicData::TrajectoryPoints& pts,
            float frameScale,
            MeshData& out);

        /// Create a COLOR_MESH from \p scene when \p meshModel is null, then bind global green material.
        static bool initializeRender(iscene_spointer scene, imodel_spointer& meshModel, MODELTYPE modelType);

        // When needsUpdate: convert PublicModel -> MeshData, import into meshModel, clear needsUpdate.
        // meshModel must exist (e.g. use initializeRender(scene, meshModel) first).
        static bool drawPublicModel(
            PublicSTLRenderData& renderData,
            const PublicData::PublicModelData& publicModelData);

        static bool drawPublicModelWithVertexColors(
            PublicSTLRenderData& renderData,
            const PublicData::PublicModelData& publicModelData,
            const std::vector<float>& vertexColors);

        // When needsUpdate: TrajData -> per-point RGB frames (TrajectoryFrame-style), import, clear needsUpdate.
        static bool drawPublicTrajectory(
            PublicTrajectoryRenderData& renderData,
            const PublicData::TrajData& publicTrajData,
            float frameScale = 30.0f);

        static bool drawPublicThicknessHeatmap(
            PublicThicknessRenderData& renderData,
            const PublicData::PublicModelData& publicModelData,
            const std::vector<float>& vertexColors);

        static bool drawPublicThicknessHeatmap(
            PublicThicknessRenderData& renderData,
            const PublicData::PublicModelData& publicModelData,
            const std::vector<float>& vertexColors,
            const std::vector<double>& thicknessValues,
            float displacementScale);
    };


}

