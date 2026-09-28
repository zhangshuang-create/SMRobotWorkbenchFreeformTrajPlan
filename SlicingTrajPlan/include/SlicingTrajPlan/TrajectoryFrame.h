#pragma once
#include "TrajectoryFrame_global.h"
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/imodel.h>
#include <ModelTangQing/MeshData.h>
#include <GeneralData/GeneralData.h> // 用于 TrajectoryPoint
#include <RenderData/RenderData.h>
namespace ModelTrajPlan
{
    class TrajectoryFrame_SHAREDLIB TrajectoryFrame
    {
    public:
        TrajectoryFrame();
        virtual ~TrajectoryFrame();

        // === 轨迹点坐标系可视化接口（纯算法）===

        // [ALGORITHM] Get small coordinate frame model (使用 RenderData 管理)
        // Input: renderData - render data container
        //        scale - frame scale
        // Output: imodel_spointer - frame model
        static imodel_spointer getSmallFrame(TrajectoryRenderData& renderData, float scale = 20.0f);

        // [ALGORITHM] Create multiple trajectory coordinate frames (使用 RenderData 管理)
        // Input: renderData - render data container
        //        poses - trajectory poses
        //        scale - frame scale
        // Output: vector<imodel_spointer> - frame models
        static std::vector<imodel_spointer> createTrajectoryFrames(TrajectoryRenderData& renderData,
            const std::vector<Eigen::Matrix4d>& poses,
            float scale = 20.0f);

        // [ALGORITHM] Create trajectory points rendering model
        // Input: trajectoryPoints - trajectory point data
        //        frameScale - coordinate frame scale
        // Output: renderData - updated with trajectory rendering model
        static imodel_spointer createTrajectoryPointsModel(TrajectoryRenderData& renderData,
            const std::vector<TrajectoryPoint>& trajectoryPoints,
            float frameScale = 30.0f);


        // [ALGORITHM] Data conversion method (pure data processing)
        // Input: trajectoryPoints - trajectory point data
        //        frameScale - coordinate frame scale
        // Output: model_data - converted rendering data
        static  void convertTrajectoryPointsToModelData(
            const std::vector<TrajectoryPoint>& trajectoryPoints,
            float frameScale,
            MeshData& outputData
        );
    private:
        // === No internal data storage ===

        // [ALGORITHM] Create coordinate axis data (pure algorithm, static)
        // Input: scale - axis scale
        // Output: data - axis model data
        static void createAxisData(MeshData& data, float scale);

    };
}