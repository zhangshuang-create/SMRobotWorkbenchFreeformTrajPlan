// PoseTransform.cpp
#include <PoseTransform/PoseTransform.h>
#include <iostream>
#include <cmath>

// Define math constants
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace ModelTrajPlan
{
    // ------------------------------------------------------------
    // Constructor / Destructor
    // ------------------------------------------------------------
    PoseTransform::PoseTransform()
        : transform_calculated_(false)
    {
        // Initialize transformation matrix as identity
        initializeTransformMatrix();

        // Initialize Assimp importer
        importer_ = std::make_unique<Assimp::Importer>();
    }

    PoseTransform::~PoseTransform()
    {
        // Release resources
        clearModels();
    }

    // ------------------------------------------------------------
    // Initialization functions
    // ------------------------------------------------------------
    void PoseTransform::initializeTransformMatrix()
    {
        transform_matrix_ = Eigen::Matrix4d::Identity();
        transform_calculated_ = false;
    }

    void PoseTransform::clearModels()
    {
        first_model_.clear();
        second_model_.clear();
        initializeTransformMatrix();
    }

    // ------------------------------------------------------------
    // STL file loading functions
    // ------------------------------------------------------------
    bool PoseTransform::loadSTLFile(const std::string& file_path, PoseMeshData& mesh_data)
    {
        return loadSTLWithAssimp(file_path, mesh_data);
    }

    bool PoseTransform::loadFirstSTLModel(const std::string& file_path)
    {
        std::cout << "\n=== Load First STL Model ===" << std::endl;
        std::cout << "File path: " << file_path << std::endl;

        bool success = loadSTLWithAssimp(file_path, first_model_);
        if (success) {
            std::cout << "First STL model loaded successfully!" << std::endl;
            std::cout << "Vertex count: " << first_model_.vertices.size() << std::endl;
            std::cout << "Triangle count: " << first_model_.triangles.size() << std::endl;

            // Reset transform-matrix computation state
            transform_calculated_ = false;
        }
        else {
            std::cerr << "Failed to load first STL model!" << std::endl;
        }

        return success;
    }

    bool PoseTransform::loadSecondSTLModel(const std::string& file_path)
    {
        std::cout << "\n=== Load Second STL Model ===" << std::endl;
        std::cout << "File path: " << file_path << std::endl;

        bool success = loadSTLWithAssimp(file_path, second_model_);
        if (success) {
            std::cout << "Second STL model loaded successfully!" << std::endl;
            std::cout << "Vertex count: " << second_model_.vertices.size() << std::endl;
            std::cout << "Triangle count: " << second_model_.triangles.size() << std::endl;

            // Reset transform-matrix computation state
            transform_calculated_ = false;
        }
        else {
            std::cerr << "Failed to load second STL model!" << std::endl;
        }

        return success;
    }

    bool PoseTransform::loadSTLWithAssimp(const std::string& file_path, PoseMeshData& mesh_data)
    {
        std::cout << "Loading STL file with Assimp: " << file_path << std::endl;

        // Clear existing data
        mesh_data.clear();

        if (!importer_) {
            std::cerr << "Error: Assimp importer is not initialized" << std::endl;
            return false;
        }

        // Load file with Assimp
        const aiScene* scene = importer_->ReadFile(file_path,
            aiProcess_Triangulate |
            aiProcess_JoinIdenticalVertices |
            aiProcess_GenNormals);

        if (!scene || !scene->HasMeshes()) {
            std::cerr << "Error: failed to load STL file or file does not contain mesh data" << std::endl;
            std::cerr << "Assimp error: " << importer_->GetErrorString() << std::endl;
            return false;
        }

        std::cout << "STL file loaded successfully!" << std::endl;
        std::cout << "Mesh count: " << scene->mNumMeshes << std::endl;

        // Iterate over all meshes
        for (unsigned int m = 0; m < scene->mNumMeshes; m++) {
            const aiMesh* mesh = scene->mMeshes[m];
            std::cout << "Mesh " << m << ": vertices = " << mesh->mNumVertices
                << ", faces = " << mesh->mNumFaces << std::endl;

            int vertex_offset = mesh_data.vertices.size();

            // Add vertices
            for (unsigned int v = 0; v < mesh->mNumVertices; v++) {
                const aiVector3D& vertex = mesh->mVertices[v];
                mesh_data.vertices.push_back(Point3D(vertex.x, vertex.y, vertex.z));
            }

            // Add triangle faces
            for (unsigned int f = 0; f < mesh->mNumFaces; f++) {
                const aiFace& face = mesh->mFaces[f];
                if (face.mNumIndices == 3) {
                    std::array<int, 3> triangle = {
                        static_cast<int>(vertex_offset + face.mIndices[0]),
                        static_cast<int>(vertex_offset + face.mIndices[1]),
                        static_cast<int>(vertex_offset + face.mIndices[2])
                    };
                    mesh_data.triangles.push_back(triangle);
                }
            }
        }

        std::cout << "Load complete: vertices = " << mesh_data.vertices.size()
            << ", triangles = " << mesh_data.triangles.size() << std::endl;

        return true;
    }

    // ------------------------------------------------------------
    // Transformation matrix computation functions
    // ------------------------------------------------------------
    Eigen::Matrix4d PoseTransform::calculateTransformationMatrix()
    {
        if (!isModelsLoaded()) {
            std::cerr << "Error: both models must be loaded before computing transformation matrix" << std::endl;
            return Eigen::Matrix4d::Identity();
        }

        return calculateTransformationMatrix(first_model_, second_model_);
    }

    Eigen::Matrix4d PoseTransform::calculateTransformationMatrix(const PoseMeshData& source_mesh, const PoseMeshData& target_mesh)
    {
        std::cout << "\n=== Compute Rigid Transformation Matrix ===" << std::endl;
        std::cout << "Source mesh vertex count: " << source_mesh.vertices.size() << std::endl;
        std::cout << "Target mesh vertex count: " << target_mesh.vertices.size() << std::endl;

        if (source_mesh.vertices.empty() || target_mesh.vertices.empty()) {
            std::cerr << "Error: mesh data is empty" << std::endl;
            return Eigen::Matrix4d::Identity();
        }

        // Use Kabsch algorithm to compute optimal rigid transformation
        Eigen::Matrix4d transformation = computeRigidTransformation(source_mesh, target_mesh);

        // Extract rotation and translation for display
        Eigen::Matrix3d rotation = transformation.block<3, 3>(0, 0);
        Eigen::Vector3d translation = transformation.block<3, 1>(0, 3);

        // Compute rotation angle (for debug display)
        Eigen::AngleAxisd angle_axis(rotation);
        double angle_degrees = angle_axis.angle() * 180.0 / M_PI;
        Eigen::Vector3d axis = angle_axis.axis();

        std::cout << "\n=== Transformation Result ===" << std::endl;
        std::cout << "Rotation angle: " << angle_degrees << " deg" << std::endl;
        std::cout << "Rotation axis: [" << axis.x() << ", " << axis.y() << ", " << axis.z() << "]" << std::endl;
        std::cout << "Translation vector: [" << translation.x() << ", " << translation.y() << ", " << translation.z() << "]" << std::endl;

        // Compute transformation quality metric
        double residual_error = evaluateTransformationQuality(source_mesh, target_mesh, transformation);
        std::cout << "Residual transformation error: " << residual_error << std::endl;

        std::cout << "\nFull transformation matrix:" << std::endl;
        std::cout << transformation << std::endl;

        // Save computed result
        transform_matrix_ = transformation;
        transform_calculated_ = true;

        return transformation;
    }

    Eigen::Vector3d PoseTransform::computeCentroid(const PoseMeshData& mesh)
    {
        if (mesh.vertices.empty()) {
            return Eigen::Vector3d::Zero();
        }

        Eigen::Vector3d centroid(0, 0, 0);
        for (const auto& v : mesh.vertices) {
            centroid.x() += v.x;
            centroid.y() += v.y;
            centroid.z() += v.z;
        }
        centroid /= mesh.vertices.size();
        return centroid;
    }

    // ------------------------------------------------------------
    // Transform application functions
    // ------------------------------------------------------------
    PoseMeshData PoseTransform::transformMesh(const PoseMeshData& mesh, const Eigen::Matrix4d& transform)
    {
        PoseMeshData transformed = mesh;

        for (auto& vertex : transformed.vertices) {
            Eigen::Vector4d v(vertex.x, vertex.y, vertex.z, 1.0);
            Eigen::Vector4d v_transformed = transform * v;
            vertex.x = v_transformed.x();
            vertex.y = v_transformed.y();
            vertex.z = v_transformed.z();
        }

        return transformed;
    }

    PoseTransformTrajectory PoseTransform::transformTrajectory(const PoseTransformTrajectory& trajectory, const Eigen::Matrix4d& transform)
    {
        std::cout << "\n=== Transform Trajectory Data ===" << std::endl;
        std::cout << "Trajectory name: " << trajectory.name << std::endl;
        std::cout << "Trajectory point count: " << trajectory.size() << std::endl;

        PoseTransformTrajectory transformed = trajectory;

        // Validate transformation matrix
        if (std::abs(transform(3, 0)) > 1e-6 || std::abs(transform(3, 1)) > 1e-6 ||
            std::abs(transform(3, 2)) > 1e-6 || std::abs(transform(3, 3) - 1.0) > 1e-6) {
            std::cerr << "Warning: invalid transform matrix format; last row should be [0 0 0 1]" << std::endl;
        }

        // Extract rotation and translation for validation
        Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
        double det = rotation.determinant();
        if (std::abs(det - 1.0) > 1e-6) {
            std::cerr << "Warning: rotation matrix determinant = " << det << ", not a standard orthonormal matrix" << std::endl;
        }

        // Apply transform to each pose matrix
        for (auto& pose : transformed.poses) {
            // Apply transform directly on 4x4 pose matrix
            // New pose = transformation matrix * original pose matrix
            pose = transform * pose;

            // Ensure last row remains [0 0 0 1]
            pose(3, 0) = 0.0;
            pose(3, 1) = 0.0;
            pose(3, 2) = 0.0;
            pose(3, 3) = 1.0;
        }

        std::cout << "Trajectory transformation complete, applied rotation determinant: " << det << std::endl;
        return transformed;
    }

    std::vector<PoseTransformTrajectory> PoseTransform::transformTrajectories(const std::vector<PoseTransformTrajectory>& trajectories, const Eigen::Matrix4d& transform)
    {
        std::cout << "\n=== Batch Transform Trajectory Data ===" << std::endl;
        std::cout << "Trajectory count: " << trajectories.size() << std::endl;

        std::vector<PoseTransformTrajectory> transformed_trajectories;
        transformed_trajectories.reserve(trajectories.size());

        for (const auto& trajectory : trajectories) {
            transformed_trajectories.push_back(transformTrajectory(trajectory, transform));
        }

        std::cout << "Batch trajectory transformation complete" << std::endl;
        return transformed_trajectories;
    }

    // Enhanced registration method - using sampled points and nearest-neighbor matching
    Eigen::Matrix4d PoseTransform::calculateTransformationMatrixWithSampling(const PoseMeshData& source_mesh, const PoseMeshData& target_mesh, int sample_count) {
        std::cout << "\n=== Registration with Sampling and Nearest Neighbor Matching ===" << std::endl;
        std::cout << "Source mesh vertex count: " << source_mesh.vertices.size() << std::endl;
        std::cout << "Target mesh vertex count: " << target_mesh.vertices.size() << std::endl;
        std::cout << "Sample count: " << sample_count << std::endl;

        if (source_mesh.vertices.empty() || target_mesh.vertices.empty()) {
            std::cerr << "Error: mesh data is empty" << std::endl;
            return Eigen::Matrix4d::Identity();
        }

        // Sample points from source mesh
        std::vector<Eigen::Vector3d> source_points;
        std::vector<Eigen::Vector3d> target_points;

        int actual_sample_count = std::min(sample_count, (int)source_mesh.vertices.size());
        int step = std::max(1, (int)source_mesh.vertices.size() / actual_sample_count);

        std::cout << "Sampling step: " << step << std::endl;

        for (int i = 0; i < (int)source_mesh.vertices.size(); i += step) {
            if (source_points.size() >= actual_sample_count) break;

            Eigen::Vector3d source_point(source_mesh.vertices[i].x, source_mesh.vertices[i].y, source_mesh.vertices[i].z);
            source_points.push_back(source_point);

            // Find nearest target-mesh point for each source point
            double min_distance = std::numeric_limits<double>::max();
            Eigen::Vector3d closest_target_point;

            for (const auto& target_vertex : target_mesh.vertices) {
                Eigen::Vector3d target_point(target_vertex.x, target_vertex.y, target_vertex.z);
                double distance = (source_point - target_point).norm();

                if (distance < min_distance) {
                    min_distance = distance;
                    closest_target_point = target_point;
                }
            }

            target_points.push_back(closest_target_point);
        }

        std::cout << "Actual paired point count: " << source_points.size() << std::endl;

        if (source_points.size() < 3) {
            std::cerr << "Error: at least 3 points are required for rigid transformation" << std::endl;
            return Eigen::Matrix4d::Identity();
        }

        // Build point-cloud matrices
        int n_points = source_points.size();
        Eigen::Matrix3Xd P(3, n_points);  // Source point cloud
        Eigen::Matrix3Xd Q(3, n_points);  // Target point cloud

        for (int i = 0; i < n_points; ++i) {
            P.col(i) = source_points[i];
            Q.col(i) = target_points[i];
        }

        // Compute centroids
        Eigen::Vector3d centroid_P = P.rowwise().mean();
        Eigen::Vector3d centroid_Q = Q.rowwise().mean();

        std::cout << "Source centroid: [" << centroid_P.transpose() << "]" << std::endl;
        std::cout << "Target centroid: [" << centroid_Q.transpose() << "]" << std::endl;

        // Center point clouds
        Eigen::Matrix3Xd P_centered = P.colwise() - centroid_P;
        Eigen::Matrix3Xd Q_centered = Q.colwise() - centroid_Q;

        // Compute covariance matrix H = P_centered * Q_centered^T
        Eigen::Matrix3d H = P_centered * Q_centered.transpose();

        std::cout << "Covariance matrix H:" << std::endl << H << std::endl;

        // SVD decomposition
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::Vector3d singular_values = svd.singularValues();

        std::cout << "Singular values: [" << singular_values.transpose() << "]" << std::endl;

        // Compute rotation matrix
        Eigen::Matrix3d R = V * U.transpose();

        // Ensure positive determinant (right-handed coordinate system)
        if (R.determinant() < 0) {
            std::cout << "Reflection transform detected, correcting to pure rotation" << std::endl;
            Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
            correction(2, 2) = -1;
            V = V * correction;
            R = V * U.transpose();
        }

        // Validate rotation matrix
        double det = R.determinant();
        std::cout << "Rotation matrix determinant: " << det << std::endl;

        if (std::abs(det - 1.0) > 1e-6) {
            std::cerr << "Warning: rotation matrix is not a standard orthonormal matrix" << std::endl;
        }

        // Compute translation vector
        Eigen::Vector3d t = centroid_Q - R * centroid_P;

        // Build 4x4 transformation matrix
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
        transformation.block<3, 3>(0, 0) = R;
        transformation.block<3, 1>(0, 3) = t;

        // Compute registration error
        double total_error = 0.0;
        for (int i = 0; i < n_points; ++i) {
            Eigen::Vector4d source_homo(source_points[i].x(), source_points[i].y(), source_points[i].z(), 1.0);
            Eigen::Vector4d transformed = transformation * source_homo;
            Eigen::Vector3d error = transformed.head<3>() - target_points[i];
            total_error += error.squaredNorm();
        }
        double rmse = std::sqrt(total_error / n_points);
        std::cout << "Registration RMSE: " << rmse << std::endl;

        std::cout << "\nFull transformation matrix:" << std::endl;
        std::cout << transformation << std::endl;

        return transformation;
    }

    // ------------------------------------------------------------
    // Kabsch algorithm implementation - compute optimal rigid transformation
    // ------------------------------------------------------------
    Eigen::Matrix4d PoseTransform::computeRigidTransformation(const PoseMeshData& source_mesh, const PoseMeshData& target_mesh)
    {
        std::cout << "\n=== Compute Rigid Transformation Using Kabsch Algorithm ===" << std::endl;

        // Check vertex-count match
        size_t n_source = source_mesh.vertices.size();
        size_t n_target = target_mesh.vertices.size();

        if (n_source != n_target) {
            std::cout << "Warning: source and target vertex counts do not match; using minimum count for registration" << std::endl;
        }

        size_t n_points = std::min(n_source, n_target);
        if (n_points < 3) {
            std::cerr << "Error: at least 3 points are required for rigid transformation" << std::endl;
            return Eigen::Matrix4d::Identity();
        }

        // Build point-cloud matrices (3 x n)
        Eigen::Matrix3Xd P(3, n_points);  // Source point cloud
        Eigen::Matrix3Xd Q(3, n_points);  // Target point cloud

        for (size_t i = 0; i < n_points; ++i) {
            P.col(i) << source_mesh.vertices[i].x, source_mesh.vertices[i].y, source_mesh.vertices[i].z;
            Q.col(i) << target_mesh.vertices[i].x, target_mesh.vertices[i].y, target_mesh.vertices[i].z;
        }

        // Compute centroids
        Eigen::Vector3d centroid_P = P.rowwise().mean();
        Eigen::Vector3d centroid_Q = Q.rowwise().mean();

        std::cout << "Source centroid: [" << centroid_P.transpose() << "]" << std::endl;
        std::cout << "Target centroid: [" << centroid_Q.transpose() << "]" << std::endl;

        // Center point clouds
        Eigen::Matrix3Xd P_centered = P.colwise() - centroid_P;
        Eigen::Matrix3Xd Q_centered = Q.colwise() - centroid_Q;

        // Compute covariance matrix H = P_centered * Q_centered^T
        Eigen::Matrix3d H = P_centered * Q_centered.transpose();

        std::cout << "Covariance matrix H:" << std::endl << H << std::endl;

        // SVD decomposition
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::Vector3d singular_values = svd.singularValues();

        std::cout << "Singular values: [" << singular_values.transpose() << "]" << std::endl;

        // Compute rotation matrix
        Eigen::Matrix3d R = V * U.transpose();

        // Ensure positive determinant (right-handed coordinate system)
        if (R.determinant() < 0) {
            std::cout << "Reflection transform detected, correcting to pure rotation" << std::endl;
            Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
            correction(2, 2) = -1;
            V = V * correction;
            R = V * U.transpose();
        }

        // Validate rotation matrix
        double det = R.determinant();
        std::cout << "Rotation matrix determinant: " << det << std::endl;

        if (std::abs(det - 1.0) > 1e-6) {
            std::cerr << "Warning: rotation matrix is not a standard orthonormal matrix" << std::endl;
        }

        // Compute translation vector
        Eigen::Vector3d t = centroid_Q - R * centroid_P;

        // Build 4x4 transformation matrix
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
        transformation.block<3, 3>(0, 0) = R;
        transformation.block<3, 1>(0, 3) = t;

        return transformation;
    }

    // ------------------------------------------------------------
    // Transformation quality evaluation
    // ------------------------------------------------------------
    double PoseTransform::evaluateTransformationQuality(const PoseMeshData& source_mesh, const PoseMeshData& target_mesh, const Eigen::Matrix4d& transform)
    {
        size_t n_points = std::min(source_mesh.vertices.size(), target_mesh.vertices.size());
        if (n_points == 0) return 1e10;  // Return large value to indicate invalid evaluation

        double total_error = 0.0;
        double max_error = 0.0;

        for (size_t i = 0; i < n_points; ++i) {
            // Transform source point
            Eigen::Vector4d source_point(source_mesh.vertices[i].x, source_mesh.vertices[i].y, source_mesh.vertices[i].z, 1.0);
            Eigen::Vector4d transformed_point = transform * source_point;

            // Compute distance to target point
            Eigen::Vector3d target_point(target_mesh.vertices[i].x, target_mesh.vertices[i].y, target_mesh.vertices[i].z);
            Eigen::Vector3d error_vector = transformed_point.head<3>() - target_point;

            double point_error = error_vector.norm();
            total_error += point_error * point_error;  // Squared error
            max_error = std::max(max_error, point_error);
        }

        double rmse = std::sqrt(total_error / n_points);
        std::cout << "RMSE: " << rmse << ", max error: " << max_error << std::endl;

        return rmse;
    }
}