#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>
namespace ModelTrajPlan
{
    // Input data type enum
    enum class InputDataType {
        PointCloud,  // Point-cloud data (requires downsampling)
        STLMesh,     // STL mesh data (requires upsampling/densification)
        Mixed        // Mixed data (contains both point cloud and mesh)
    };

    // Axis-aligned bounding box structure
    struct AABB {
        Eigen::Vector3d min_point;
        Eigen::Vector3d max_point;
        Eigen::Matrix3d principal_axes;  // Three principal directions
        Eigen::Vector3d center;          // Centroid / translation (world frame)

        AABB() : min_point(Eigen::Vector3d::Zero()),
            max_point(Eigen::Vector3d::Zero()),
            principal_axes(Eigen::Matrix3d::Identity()),
            center(Eigen::Vector3d::Zero()) {}
    };

    // Triangle-face structure (for STL mesh data)
    struct MeshTriangle {
        Eigen::Vector3d vertex0;
        Eigen::Vector3d vertex1;
        Eigen::Vector3d vertex2;
        Eigen::Vector3d normal;  // Face normal

        MeshTriangle() : vertex0(Eigen::Vector3d::Zero()),
            vertex1(Eigen::Vector3d::Zero()),
            vertex2(Eigen::Vector3d::Zero()),
            normal(Eigen::Vector3d::UnitZ()) {}
    };

}