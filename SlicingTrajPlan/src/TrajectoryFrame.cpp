#include <TrajectoryFrame/TrajectoryFrame.h>
#include <RenderData/RenderData.h>
#include <iostream>
namespace ModelTrajPlan
{
    TrajectoryFrame::TrajectoryFrame()
    {
        // [DESIGN] Pure algorithm class - no internal data storage
    }

    TrajectoryFrame::~TrajectoryFrame()
    {
    }

    imodel_spointer TrajectoryFrame::getSmallFrame(TrajectoryRenderData& renderData, float scale)
    {

        // 使用 RenderData 中的 frameModel，如果不存在则创建
        if (!renderData.frameModel) {
            renderData.frameModel = renderData.sp_scene->get_new_model(MODELTYPE::MODELTYPE_COLORLINE_ELEMENT);
            if (!renderData.frameModel) {
                return nullptr;
            }
        }

        // 如果需要更新或缩放改变，重新生成数据
        if (renderData.frameNeedsUpdate || renderData.frameScale != scale) {
            renderData.frameScale = scale;
            createAxisData(renderData.frameData, scale);
            renderData.frameModel->import(renderData.frameData);
            renderData.frameNeedsUpdate = false;
        }

        return renderData.frameModel;
    }

    std::vector<imodel_spointer> TrajectoryFrame::createTrajectoryFrames(TrajectoryRenderData& renderData,
        const std::vector<Eigen::Matrix4d>& poses,
        float scale)
    {

        // 清空之前的模型
        renderData.frameModels.clear();

        for (const auto& pose : poses) {
            // 为每个姿态创建独立的坐标系模型
            imodel_spointer frame = renderData.sp_scene->get_new_model(MODELTYPE::MODELTYPE_COLORLINE_ELEMENT);

            if (frame) {
                // 使用共享的坐标系数据
                if (renderData.frameNeedsUpdate || renderData.frameScale != scale) {
                    renderData.frameScale = scale;
                    createAxisData(renderData.frameData, scale);
                    renderData.frameNeedsUpdate = false;
                }

                frame->import(renderData.frameData);

                // Apply transformation matrix to model
                renderData.frameModels.push_back(frame);
            }
        }

        return renderData.frameModels;
    }

    void TrajectoryFrame::createAxisData(MeshData& data, float scale)
    {
        // Clear data
        data.m_vertices.clear();
        data.m_indices.clear();

        // Create vertex data for three coordinate axes
        // Format: x, y, z, r, g, b, a

        // X-axis (red): origin to (scale, 0, 0)
        data.m_vertices.insert(data.m_vertices.end(), {
            0.0f, 0.0f, 0.0f,      1.0f, 0.0f, 0.0f, 1.0f,  // Origin, red
            scale, 0.0f, 0.0f,     1.0f, 0.0f, 0.0f, 1.0f   // X-axis endpoint, red
            });

        // Y-axis (green): origin to (0, scale, 0)
        data.m_vertices.insert(data.m_vertices.end(), {
            0.0f, 0.0f, 0.0f,      0.0f, 1.0f, 0.0f, 1.0f,  // Origin, green
            0.0f, scale, 0.0f,     0.0f, 1.0f, 0.0f, 1.0f   // Y-axis endpoint, green
            });

        // Z-axis (blue): origin to (0, 0, scale)
        data.m_vertices.insert(data.m_vertices.end(), {
            0.0f, 0.0f, 0.0f,      0.0f, 0.0f, 1.0f, 1.0f,  // Origin, blue
            0.0f, 0.0f, scale,     0.0f, 0.0f, 1.0f, 1.0f   // Z-axis endpoint, blue
            });

        // Create indices (line segments)
        data.m_indices.insert(data.m_indices.end(), {
            0, 1,  // X-axis line
            2, 3,  // Y-axis line  
            4, 5   // Z-axis line
            });
    }

    // === Method implementations (now static or removed) ===

    // Note: These methods are no longer needed as the class is now stateless


    // [REFACTOR] Create rendering data for trajectory points (algorithm moved from BurTrajPlan)
    imodel_spointer TrajectoryFrame::createTrajectoryPointsModel(TrajectoryRenderData& renderData,
        const std::vector<TrajectoryPoint>& trajectoryPoints,
        float frameScale)
    {

        // [REFACTOR] Get already created model (created by RenderData::initializeModels)
        imodel_spointer trajectoryModel = renderData.trajectoryModel;

        if (!trajectoryModel) {
            return nullptr;
        }

        // [REFACTOR] 直接使用 RenderData 中持久化的 model_data，不创建临时变量
        convertTrajectoryPointsToModelData(trajectoryPoints, frameScale, renderData.trajectoryData);
        convertTrajectoryPointsToModelData(trajectoryPoints, frameScale, renderData.trajectoryData);
        // 导入数据到渲染模型
        if (!renderData.trajectoryData.m_vertices.empty()) {
            trajectoryModel->import(renderData.trajectoryData);
        }

        return trajectoryModel;
    }


    // [REFACTOR NEW] Convert trajectory points to model_data format (引用参数版本，避免临时对象)
    void TrajectoryFrame::convertTrajectoryPointsToModelData(const std::vector<TrajectoryPoint>& trajectoryPoints, float frameScale, MeshData& trajectoryData)
    {
        trajectoryData.m_vertices.clear();
        trajectoryData.m_indices.clear();

        // Create small coordinate frame for each trajectory point
        for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
            const auto& trajPoint = trajectoryPoints[i];

            // Extract position
            Eigen::Vector3d position = trajPoint.pose.block<3, 1>(0, 3);

            // Extract rotation matrix
            Eigen::Matrix3d rotation = trajPoint.pose.block<3, 3>(0, 0);

            // Calculate three coordinate axis endpoints
            Eigen::Vector3d xAxis = position + rotation.col(0) * frameScale;  // X-axis (red)
            Eigen::Vector3d yAxis = position + rotation.col(1) * frameScale;  // Y-axis (green)
            Eigen::Vector3d zAxis = position + rotation.col(2) * frameScale;  // Z-axis (blue)

            // Current vertex base index
            unsigned int baseIndex = static_cast<unsigned int>(trajectoryData.m_vertices.size() / 7);

            // Add X-axis vertices (red)
            trajectoryData.m_vertices.insert(trajectoryData.m_vertices.end(), {
                static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()),
                1.0f, 0.0f, 0.0f, 1.0f,  // Red
                static_cast<float>(xAxis.x()), static_cast<float>(xAxis.y()), static_cast<float>(xAxis.z()),
                1.0f, 0.0f, 0.0f, 1.0f   // Red
                });

            // Add Y-axis vertices (green)
            trajectoryData.m_vertices.insert(trajectoryData.m_vertices.end(), {
                static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()),
                0.0f, 1.0f, 0.0f, 1.0f,  // Green
                static_cast<float>(yAxis.x()), static_cast<float>(yAxis.y()), static_cast<float>(yAxis.z()),
                0.0f, 1.0f, 0.0f, 1.0f   // Green
                });

            // Add Z-axis vertices (blue)
            trajectoryData.m_vertices.insert(trajectoryData.m_vertices.end(), {
                static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()),
                0.0f, 0.0f, 1.0f, 1.0f,  // Blue
                static_cast<float>(zAxis.x()), static_cast<float>(zAxis.y()), static_cast<float>(zAxis.z()),
                0.0f, 0.0f, 1.0f, 1.0f   // Blue
                });

            // Add line segment indices
            trajectoryData.m_indices.insert(trajectoryData.m_indices.end(), {
                baseIndex + 0, baseIndex + 1,  // X-axis line
                baseIndex + 2, baseIndex + 3,  // Y-axis line
                baseIndex + 4, baseIndex + 5   // Z-axis line
                });
        }
    }
}