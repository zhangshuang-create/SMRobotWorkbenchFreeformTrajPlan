#pragma once

#include "PoseTransform_global.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ModelTangQing/imodel.h>
#include <GeneralData/GeneralData.h>
#include <RenderData/RenderData.h>
#include <CameraTangQing/icamera.h>
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/icollision_detector.h>
#include <SMRobotTangQing/robot_factory.h>
#include <vector>
#include <string>
#include <array>
#include <memory>
#include <map>

// Assimp headers for STL loading
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
namespace ModelTrajPlan
{
    // ------------------------------------------------------------
    // 3D point structure
    // ------------------------------------------------------------
    struct Point3D {
        double x, y, z;
        Point3D() : x(0), y(0), z(0) {}
        Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    };

    // ------------------------------------------------------------
    // Mesh data structure
    // ------------------------------------------------------------
    struct PoseMeshData {
        std::vector<Point3D> vertices;
        std::vector<std::array<int, 3>> triangles;

        PoseMeshData() {}
        bool isEmpty() const { return vertices.empty(); }
        void clear() { vertices.clear(); triangles.clear(); }
    };

    // ------------------------------------------------------------
    // Simplified trajectory data structure - avoids conflicts with existing system
    // ------------------------------------------------------------
    struct PoseTransformTrajectory {
        std::vector<Eigen::Matrix4d> poses;  // Pose matrix sequence
        std::vector<double> timestamps;      // Timestamp sequence
        std::string name;

        PoseTransformTrajectory() {}
        PoseTransformTrajectory(const std::string& trajectory_name) : name(trajectory_name) {}
        bool isEmpty() const { return poses.empty(); }
        void clear() { poses.clear(); timestamps.clear(); }

        // Add a trajectory point
        void addPoint(const Eigen::Matrix4d& pose, double timestamp = 0.0) {
            poses.push_back(pose);
            timestamps.push_back(timestamp);
        }

        size_t size() const { return poses.size(); }
    };

    // ------------------------------------------------------------
    // Pose transform module class
    // ------------------------------------------------------------
    class PoseTransform_SHAREDLIB PoseTransform
    {
    public:
        PoseTransform();
        ~PoseTransform();

        // STL file loading functions
        bool loadSTLFile(const std::string& file_path, PoseMeshData& mesh_data);
        bool loadFirstSTLModel(const std::string& file_path);
        bool loadSecondSTLModel(const std::string& file_path);

        // Transformation matrix computation
        Eigen::Matrix4d calculateTransformationMatrix();
        Eigen::Matrix4d calculateTransformationMatrix(const PoseMeshData& source_mesh, const PoseMeshData& target_mesh);

        // Enhanced registration method - uses sampled points and nearest-neighbor matching
        Eigen::Matrix4d calculateTransformationMatrixWithSampling(const PoseMeshData& source_mesh, const PoseMeshData& target_mesh, int sample_count = 1000);

        // Apply transform to data
        PoseMeshData transformMesh(const PoseMeshData& mesh, const Eigen::Matrix4d& transform);
        PoseTransformTrajectory transformTrajectory(const PoseTransformTrajectory& trajectory, const Eigen::Matrix4d& transform);
        std::vector<PoseTransformTrajectory> transformTrajectories(const std::vector<PoseTransformTrajectory>& trajectories, const Eigen::Matrix4d& transform);

        // Get loaded model data
        const PoseMeshData& getFirstModel() const { return first_model_; }
        const PoseMeshData& getSecondModel() const { return second_model_; }
        const Eigen::Matrix4d& getTransformMatrix() const { return transform_matrix_; }

        // Utility functions
        Eigen::Vector3d computeCentroid(const PoseMeshData& mesh);
        bool isModelsLoaded() const { return !first_model_.isEmpty() && !second_model_.isEmpty(); }
        void clearModels();

        // Kabsch algorithm core functions
        Eigen::Matrix4d computeRigidTransformation(const PoseMeshData& source_mesh, const PoseMeshData& target_mesh);
        double evaluateTransformationQuality(const PoseMeshData& source_mesh, const PoseMeshData& target_mesh, const Eigen::Matrix4d& transform);

    private:
        // Member variables
        PoseMeshData first_model_;     // First STL model data
        PoseMeshData second_model_;    // Second STL model data
        Eigen::Matrix4d transform_matrix_;  // Transformation matrix
        bool transform_calculated_;     // Whether transformation matrix has been computed

        // Assimp importer
        std::unique_ptr<Assimp::Importer> importer_;

        // Internal helper functions
        bool loadSTLWithAssimp(const std::string& file_path, PoseMeshData& mesh_data);
        void initializeTransformMatrix();
    };
}