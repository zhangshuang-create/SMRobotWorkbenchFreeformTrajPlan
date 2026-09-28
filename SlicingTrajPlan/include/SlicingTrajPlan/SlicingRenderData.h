#pragma once
#include <RenderData/RenderData_global.h>
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/imodel.h>
#include <vector>
namespace ModelTrajPlan
{
    // 切片渲染数据（简化版）
    class RenderData_SHAREDLIB SlicingRenderData
    {
    public:
        // 渲染模型
        imodel_spointer sliceModel;

        // 模型数据（持久化存储）
        MeshData sliceData;

        // 可见性控制
        bool isVisible = true;

        // 更新标志
        bool needsUpdate = true;

        SlicingRenderData();
        ~SlicingRenderData();
    };
}