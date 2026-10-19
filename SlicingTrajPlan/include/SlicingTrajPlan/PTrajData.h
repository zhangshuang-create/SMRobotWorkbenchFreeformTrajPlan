#pragma once

#define _USE_MATH_DEFINES  // 必须在 <cmath> 之前定义
#include <cmath>           // 包含 M_PI 定义

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <SlicingTrajPlan/GeneralData_global.h>
#include "SlicingData.h"  // 包含SlicingParams的完整定义
#include <map>
namespace PublicData { enum class CoordinateFrame { World }; }  // 包含 CoordinateFrame 枚举定义

// 如果 M_PI 仍未定义（某些编译器），手动定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ModelTrajPlan
{
    // 【新增】轨迹分组信息（用于返回轨迹统计）- 从 TrajectoryGenerator.h 迁移
    struct TrajectoryGroupInfo {
        int numTrajectories = 0;       // 轨迹条数
        int pointsPerTrajectory = 0;   // 每条轨迹的点数
        bool isConsistent = true;      // 所有轨迹点数是否一致
        TrajectoryGroupInfo() = default;
    };



    // 【重构新增】轨迹生成参数结构 - 从SlicingParams分离出来的偏移参数
    struct TrajectoryGenerationParams {
        int normalAxis = 4;
        int forwardAxis = 0;
        bool previewFirstPoint = false;
        bool upperTransitionSmoothPose = false;
        bool lowerTransitionSmoothPose = false;
        // 偏移参数
        double z1Offset = 0.0;            // 起始Z偏移（下边界偏移）
        double z2Offset = 0.0;            // 结束Z偏移（上边界偏移）
        double offsetDistance = 1.0;      // 法线偏移距离

        // 速度参数
        double normalVelocity = 50.0;     // 正常轨迹速度 (mm/s)
        double transitionVelocity = 50.0; // 过渡段速度 (mm/s)
        double acceleration = 0.0;        // 加速度 (mm/s)

        TrajectoryGenerationParams() = default;
        TrajectoryGenerationParams(double z1, double z2, double offset,
            double normalVel = 50.0, double transVel = 50.0,
            double accel = 0.0, double timeInt = 0.1)
            : z1Offset(z1), z2Offset(z2), offsetDistance(offset),
            normalVelocity(normalVel), transitionVelocity(transVel),
            acceleration(accel) {}
    };


    // 连接策略枚举
    enum class ConnectionStrategy {
        Sequential = 0,    // 顺序连接
        Optimized = 1,     // 优化连接
        ZigZag = 2,        // Z字形连接
        LayerByLayer = 3,  // 逐层连接
        BowPath = 4        // 弓形路径连接
    };

    // 【新增】轨迹连接参数结构 - 从 TrajectoryConnector.h 迁移
    struct TrajectoryConnectionParams {
        int numTrajectories = 0;      // 轨迹数量
        int pointsPerTrajectory = 0;  // 每条轨迹的点数
        ConnectionStrategy strategy = ConnectionStrategy::Sequential; // 连接策略

        TrajectoryConnectionParams() = default;
    };

    // 轨迹结构 - 包含位姿和时间戳
    struct TrajectoryPoint {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();  // 齐次矩阵姿态位置）
        double spray_distance = 0.0;                         // 喷涂距离（mm），工具到工件表面的距离
        // ====== Debug / indexing metadata (non-functional) ======
        // 用于标识“第几条轨迹-第几层”，例如：trajectory_index=0, layer_index=1 表示 "01"
        int trajectory_index = -1;                           // 轨迹索引（trajectoryIndex）
        int layer_index = -1;                                // 层索引（layerIndex）

        double timestamp = 0.0;                              // 时间戳
        double velocity = 0.0;                               // 速度
        double acceleration = 0.0;                           // 加速度
        int point_type = 0;                                  // 点类型（0=普通点，1=边界点，2=偏移过渡点）
    };

    // CoordinateFrame 枚举定义在 PublicData/TrajData/TrajData.h 中

    struct TolerancedTrajectoryPoint {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

        Eigen::Matrix<double, 3, 2> pos_tolerance;
        Eigen::Matrix<double, 3, 2> orient_tolerance;
        Eigen::Vector2d vel_range;//速率范围

        bool is_adjustable;
        PublicData::CoordinateFrame tolerance_frame;
        double spray_distance = 0.0;  // 喷涂距离（mm），工具到工件表面的距离

        TolerancedTrajectoryPoint()
            : pos_tolerance(Eigen::Matrix<double, 3, 2>::Zero()),
            orient_tolerance(Eigen::Matrix<double, 3, 2>::Zero()),
            vel_range(Eigen::Vector2d::Zero()),
            is_adjustable(true),
            tolerance_frame(PublicData::CoordinateFrame::World),
            spray_distance(0.0)
        {
            pose = Eigen::Matrix4d::Identity();
        }
    };

    // 容差搜索配置结构体
    struct ToleranceLimitConfig {
        // ========== 搜索范围 ==========
        Eigen::Vector3d max_translation;  // mm, 每个轴的最大平移搜索范围, e.g. [50, 30, 10]
        Eigen::Vector3d max_rotation;     // rad, 每个轴的最大旋转搜索范围, e.g. [M_PI, M_PI/4, M_PI/3]

        // ========== 二分搜索停止条件 ==========
        double position_tolerance;        // mm, 位置搜索精度（停止条件），默认 0.1mm
        double rotation_tolerance;        // rad, 旋转搜索精度（停止条件），默认 0.5° = 0.00873 rad

        // ========== 构造函数（设置默认值） ==========
        ToleranceLimitConfig()
            : max_translation(50.0, 30.0, 10.0),
            max_rotation(M_PI, M_PI / 4.0, M_PI / 3.0),
            position_tolerance(0.1),           // 0.1mm
            rotation_tolerance(0.00873)        // 0.5° in radians
        {}
    };

    // 连接线段信息结构 - 从 TrajectoryConnector.h 移动
    struct ConnectionSegment {
        Eigen::Vector3d startPoint = Eigen::Vector3d::Zero();        // 起始点
        Eigen::Vector3d endPoint = Eigen::Vector3d::Zero();          // 结束点
        Eigen::Vector3f color = Eigen::Vector3f(1.0f, 1.0f, 1.0f);  // 线段颜色
        float lineWidth = 2.0f;                                      // 线宽
        int segmentType = 0;                                         // 线段类型：0=轨迹内连接，1=轨迹间连接，2=过渡连接
    };

    // 连接数据结构 - 存储轨迹连接的完整结果
    struct ConnectionData {
        std::vector<TrajectoryPoint> connectedTrajectory_;           // 连接后的完整轨迹
        std::vector<ConnectionSegment> connectionSegments_;          // 连接线段
        ConnectionStrategy strategy_ = ConnectionStrategy::Sequential; // 使用的连接策略
        bool isConnected_ = false;                                   // 是否已连接

        // 连接参数
        double transitionSpeed_ = 30.0;
        double normalSpeed_ = 50.0;
        double connectionDistance_ = 5.0;
    };

    // 简化的数据结构定义，来自 TrajectoryGenerator.h - 重命名避免与ThicknessCalculator冲突
    struct TrajVector3D {
        double x = 0.0, y = 0.0, z = 0.0;
        TrajVector3D() = default;
        TrajVector3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    };

    struct Vector4D {
        double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
        Vector4D() = default;
        Vector4D(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}
    };


    class GeneralData_SHAREDLIB TrajData
    {
    public:
        TrajData();
        ~TrajData();

        // 数据成员 - 公开访问，仅数据容器
        std::vector<TrajectoryPoint> trajectoryPoints_;        // 主轨迹点集合
        std::map<double, TolerancedTrajectoryPoint> trajectoryPointsbf_;
        std::map<double, TolerancedTrajectoryPoint> trajectoryPointsaf_;

        ConnectionData connectionData_;                        // 连接数据（完整的连接结果）
        ToleranceLimitConfig tol_limit;   //容差范围

        // 【新增】采样点数据 - 从边界线均匀采样得到的点 [trajectoryIndex][layerIndex]
        std::vector<std::vector<Eigen::Vector3d>> sampledBoundaryPoints_;

        // 【重构】轨迹参数管理 - 从各个算法模块迁移过来
        TrajectoryGenerationParams generationParams_;         // 轨迹生成参数（偏移参数）
        TrajectoryConnectionParams connectionParams_;          // 轨迹连接参数
        TrajectoryGroupInfo groupInfo_;                       // 轨迹分组信息

    private:

    };

}
