#include <RenderData/RenderData.h>
#include <RenderData/STLRenderData.h>
#include <RenderData/SlicingRenderData.h>
#include <RenderData/TrajectoryRenderData.h>
#include <RenderData/ConnectionRenderData.h>
namespace ModelTrajPlan
{
    RenderData::RenderData()
        : scene_(nullptr)
        , stlData_(std::make_unique<STLRenderData>())
        , boundaryData_(std::make_unique<BoundaryRenderData>())
        , slicingData_(std::make_unique<SlicingRenderData>())
        , trajectoryData_(std::make_unique<TrajectoryRenderData>())
        , connectionData_(std::make_unique<ConnectionRenderData>())
    {
    }

    RenderData::~RenderData()
    {
    }

    void RenderData::setScene(iscene_spointer scene)
    {
        scene_ = scene;
    }

    void RenderData::initializeAllModels()
    {
        if (!scene_) {
            return;
        }

        // 初始化STL模型
        if (!stlData_->meshModel) {
            stlData_->meshModel = scene_->get_new_model(MODELTYPE::MODELTYPE_COLOR_MESH);
        }

        // 初始化边界模型
        if (!boundaryData_->upperBoundary) {
            boundaryData_->upperBoundary = scene_->get_new_model(MODELTYPE::MODELTYPE_MONOLINE_ARRAY);
        }
        if (!boundaryData_->lowerBoundary) {
            boundaryData_->lowerBoundary = scene_->get_new_model(MODELTYPE::MODELTYPE_MONOLINE_ARRAY);
        }
        if (!boundaryData_->upperBoundaryOffset) {
            boundaryData_->upperBoundaryOffset = scene_->get_new_model(MODELTYPE::MODELTYPE_MONOLINE_ARRAY);
        }
        if (!boundaryData_->lowerBoundaryOffset) {
            boundaryData_->lowerBoundaryOffset = scene_->get_new_model(MODELTYPE::MODELTYPE_MONOLINE_ARRAY);
        }

        // 初始化切片模型
        if (!slicingData_->sliceModel) {
            slicingData_->sliceModel = scene_->get_new_model(MODELTYPE::MODELTYPE_COLOR_MESH);
        }

        // 初始化轨迹模型
        if (!trajectoryData_->trajectoryModel) {
            trajectoryData_->trajectoryModel = scene_->get_new_model(MODELTYPE::MODELTYPE_COLORLINE_ELEMENT);
        }

        // 初始化连接线模型
        if (!connectionData_->connectionModel) {
            connectionData_->connectionModel = scene_->get_new_model(MODELTYPE::MODELTYPE_COLORLINE_ELEMENT);
        }
        if (!boundaryData_->intersectionPoints) {
            // Must match BoundaryDetector::convertLinePointsToMeshData: xyz+rgba per vertex + line indices.
            boundaryData_->intersectionPoints = scene_->get_new_model(MODELTYPE::MODELTYPE_COLORLINE_ELEMENT);
        }
    }

    // STL渲染数据访问器
    STLRenderData& RenderData::stl()
    {
        return *stlData_;
    }

    const STLRenderData& RenderData::stl() const
    {
        return *stlData_;
    }

    // 边界渲染数据访问器
    BoundaryRenderData& RenderData::boundary()
    {
        return *boundaryData_;
    }

    const BoundaryRenderData& RenderData::boundary() const
    {
        return *boundaryData_;
    }

    // 切片渲染数据访问器
    SlicingRenderData& RenderData::slicing()
    {
        return *slicingData_;
    }

    const SlicingRenderData& RenderData::slicing() const
    {
        return *slicingData_;
    }

    // 轨迹渲染数据访问器
    TrajectoryRenderData& RenderData::trajectory()
    {
        return *trajectoryData_;
    }

    const TrajectoryRenderData& RenderData::trajectory() const
    {
        return *trajectoryData_;
    }

    // 连接线渲染数据访问器
    ConnectionRenderData& RenderData::connection()
    {
        return *connectionData_;
    }

    const ConnectionRenderData& RenderData::connection() const
    {
        return *connectionData_;
    }
}