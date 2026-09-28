#include <STLProcessor/STLProcessor.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
namespace ModelTrajPlan
{
    // STLProcessor static utility methods - operate on Model data
    size_t STLProcessor::getTriangleCount(const PrivateModel& modelData) {
        return modelData.m_indices.size() / 3;
    }

    size_t STLProcessor::getVertexCount(const PrivateModel& modelData) {
        return modelData.m_vertices.size() / 3;
    }

    void STLProcessor::calculateBounds(const PrivateModel& modelData, Eigen::Vector3d& bounds_min, Eigen::Vector3d& bounds_max) {
        if (modelData.m_vertices.empty()) return;

        bounds_min = Eigen::Vector3d(modelData.m_vertices[0], modelData.m_vertices[1], modelData.m_vertices[2]);
        bounds_max = bounds_min;

        for (size_t i = 0; i < modelData.m_vertices.size(); i += 3) {
            Eigen::Vector3d vertex(modelData.m_vertices[i], modelData.m_vertices[i + 1], modelData.m_vertices[i + 2]);
            bounds_min = bounds_min.cwiseMin(vertex);
            bounds_max = bounds_max.cwiseMax(vertex);
        }
    }

    void STLProcessor::getTriangleVertices(const PrivateModel& modelData, size_t triangleIndex, Eigen::Vector3d vertices[3]) {
        if (triangleIndex >= STLProcessor::getTriangleCount(modelData)) return;

        for (int i = 0; i < 3; ++i) {
            unsigned int vertexIndex = modelData.m_indices[triangleIndex * 3 + i];
            size_t baseIdx = vertexIndex * 3;
            vertices[i] = Eigen::Vector3d(
                modelData.m_vertices[baseIdx],
                modelData.m_vertices[baseIdx + 1],
                modelData.m_vertices[baseIdx + 2]
            );
        }
    }

    Eigen::Vector3d STLProcessor::getTriangleNormal(const PrivateModel& modelData, size_t triangleIndex) {
        if (triangleIndex >= STLProcessor::getTriangleCount(modelData)) return Eigen::Vector3d::Zero();

        unsigned int vertexIndex = modelData.m_indices[triangleIndex * 3];
        size_t baseIdx = vertexIndex * 3;
        return Eigen::Vector3d(
            modelData.m_normals[baseIdx],
            modelData.m_normals[baseIdx + 1],
            modelData.m_normals[baseIdx + 2]
        );
    }

    STLProcessor::STLProcessor()
    {
        // [DESIGN] Pure algorithm class - no internal data storage
    }

    STLProcessor::~STLProcessor()
    {
    }



    bool STLProcessor::preprocessSTL(const std::string& filePath,
        ModelData& modelData)
    {
        // 1. Parse STL file
        PrivateModel newModel = parseSTLFile(filePath);

        // 2. Store into all-model collection
        modelData.allModelsData_.push_back(newModel);

        // 3. Record file path
        modelData.loadedFilePaths_.push_back(filePath);

        // 4. Update statistics
        size_t triangleCount = STLProcessor::getTriangleCount(newModel);
        size_t vertexCount = STLProcessor::getVertexCount(newModel);

        if (modelData.hasValidModelData_) {
            // Existing main model -> accumulate statistics
            modelData.totalTriangleCount_ += triangleCount;
            modelData.totalVertexCount_ += vertexCount;
        }
        else {
            // First load -> initialize main model
            modelData.mainModelData_ = newModel;
            modelData.totalTriangleCount_ = triangleCount;
            modelData.totalVertexCount_ = vertexCount;

            modelData.hasValidModelData_ = true;
            modelData.isModelLoaded_ = true;
        }

        return true;
    }

    // [Addition] Copy data from PublicModel to GeneralData
    bool STLProcessor::copyFromPublicModel(const PublicData::PublicModel& publicModel,
        ModelData& modelData)
    {
        // 1. Build PrivateModel
        PrivateModel privateModel;
        privateModel.m_vertices = publicModel.m_vertices;
        privateModel.m_normals = publicModel.m_normals;
        privateModel.m_indices = publicModel.m_indices;
        privateModel.m_file_path = publicModel.m_file_path;
        privateModel.m_file_name = publicModel.m_file_name;

        // 2. Store into data container
        modelData.mainModelData_ = privateModel;
        modelData.allModelsData_.push_back(privateModel);

        // 3. Record file path (if present)
        if (!publicModel.m_file_path.empty()) {
            modelData.loadedFilePaths_.push_back(publicModel.m_file_path);
        }

        // 4. Update statistics
        modelData.totalTriangleCount_ = STLProcessor::getTriangleCount(privateModel);
        modelData.totalVertexCount_ = STLProcessor::getVertexCount(privateModel);

        // 5. Update status flags (within ModelData only)
        modelData.hasValidModelData_ = true;
        modelData.isModelLoaded_ = true;

        return true;
    }

    PrivateModel STLProcessor::parseSTLFile(const std::string& filePath)
    {
        PrivateModel modelData;
        modelData.m_file_path = filePath;
        // Extract filename from path
        size_t lastSlash = filePath.find_last_of("/\\");
        modelData.m_file_name = (lastSlash != std::string::npos) ? filePath.substr(lastSlash + 1) : filePath;

        std::ifstream file(filePath, std::ios::binary);

        if (!file.is_open()) {
            throw std::runtime_error("Cannot open STL file: " + filePath);
        }

        // Read STL header (80 bytes)
        char header[80];
        file.read(header, 80);

        // Read triangle count
        uint32_t triangleCount;
        file.read(reinterpret_cast<char*>(&triangleCount), sizeof(uint32_t));

        // Reserve memory for vertices, normals and indices
        modelData.m_vertices.reserve(triangleCount * 9);  // 3 vertices * 3 coordinates
        modelData.m_normals.reserve(triangleCount * 9);   // 3 vertices * 3 normal components
        modelData.m_indices.reserve(triangleCount * 3);   // 3 indices per triangle

        // Read triangle data
        for (uint32_t i = 0; i < triangleCount; ++i) {
            // Read normal vector
            float normal[3];
            file.read(reinterpret_cast<char*>(normal), 3 * sizeof(float));

            // Read three vertices
            float vertices[9]; // 3 vertices * 3 coordinates
            for (int j = 0; j < 3; ++j) {
                float vertex[3];
                file.read(reinterpret_cast<char*>(vertex), 3 * sizeof(float));
                vertices[j * 3 + 0] = vertex[0];
                vertices[j * 3 + 1] = vertex[1];
                vertices[j * 3 + 2] = vertex[2];
            }

            // Add vertices to vertex list
            for (int j = 0; j < 9; ++j) {
                modelData.m_vertices.push_back(vertices[j]);
            }

            // Add normals for each vertex (same normal for all 3 vertices)
            for (int j = 0; j < 3; ++j) {
                modelData.m_normals.push_back(normal[0]);
                modelData.m_normals.push_back(normal[1]);
                modelData.m_normals.push_back(normal[2]);
            }

            // Add triangle indices (3 consecutive vertices)
            unsigned int baseIndex = i * 3;
            modelData.m_indices.push_back(baseIndex + 0);
            modelData.m_indices.push_back(baseIndex + 1);
            modelData.m_indices.push_back(baseIndex + 2);

            // Read attribute byte count (2 bytes)
            uint16_t attributeByteCount;
            file.read(reinterpret_cast<char*>(&attributeByteCount), sizeof(uint16_t));
        }

        file.close();
        return modelData;
    }
    // ===================== STLProcessor (Refactored) =====================

    void STLProcessor::calculateBounds(const ModelData& modelData,
        Eigen::Vector3d& bounds_min,
        Eigen::Vector3d& bounds_max)
    {
        if (!modelData.hasValidData()) {
            return;
        }

        const PrivateModel& mainModel = modelData.mainModelData_;
        STLProcessor::calculateBounds(mainModel, bounds_min, bounds_max);
    }


    // ===================== Renderer =====================

    bool STLProcessor::initializeRenderer(STLRenderData& renderData)
    {
        if (!renderData.meshModel) {
            return false;
        }

        renderData.needsUpdate = true;
        return true;
    }


    bool STLProcessor::draw(STLRenderData& renderData,
        const ModelData& modelData)
    {
        if (!renderData.meshModel || !renderData.isVisible) {
            return false;
        }

        if (renderData.needsUpdate) {
            updateRenderData(renderData, modelData);
            renderData.needsUpdate = false;
        }

        try {
            renderData.meshModel->draw();
            return true;
        }
        catch (const std::exception&) {
            return false;
        }
    }


    void STLProcessor::setVisibility(STLRenderData& renderData, bool visible)
    {
        renderData.isVisible = visible;
    }


    // ===================== Core Render Update =====================

    void STLProcessor::updateRenderData(STLRenderData& renderData,
        const ModelData& modelData)
    {
        if (!modelData.hasValidData()) {
            return;
        }

        const PrivateModel& model = modelData.mainModelData_;

        try {
            // Fill persistent meshData directly
            STLProcessor::convertModelToMeshData(model, renderData.meshData);

            // Import to GPU
            if (renderData.meshModel && !renderData.meshData.m_vertices.empty()) {
                renderData.meshModel->import(renderData.meshData);

                // Use default material (keep original behavior)
                renderData.meshModel->use_material(MATERIAL_TYPE::GLOBAL,
                    material_attr::MaterialGreen);
            }

        }
        catch (const std::exception&) {
            // Optional: add logging here
        }
    }

    bool STLProcessor::hasValidData(const ModelData& modelData)
    {
        // 1. Check whether the main model has vertices
        if (modelData.mainModelData_.m_vertices.empty()) {
            return false;
        }

        return true;
    }
    // ===================== Clear =====================

    void STLProcessor::clear(ModelData& modelData)
    {
        // Clear main model
        modelData.mainModelData_.m_vertices.clear();
        modelData.mainModelData_.m_normals.clear();
        modelData.mainModelData_.m_indices.clear();
        modelData.mainModelData_.m_file_path.clear();
        modelData.mainModelData_.m_file_name.clear();

        // Clear collections
        modelData.allModelsData_.clear();
        modelData.loadedFilePaths_.clear();

        // Reset statistics
        modelData.totalTriangleCount_ = 0;
        modelData.totalVertexCount_ = 0;

        // Reset state
        modelData.hasValidModelData_ = false;
        modelData.isModelLoaded_ = false;
    }

    // [STATIC] Static version of convertModelToMeshData for external use (return-by-value)
    MeshData STLProcessor::convertModelToMeshData(const PrivateModel& modelData)
    {
        MeshData meshData;
        convertModelToMeshData(modelData, meshData);
        return meshData;
    }

    // [STATIC] Static version of convertModelToMeshData (reference-parameter version, avoids temporaries)
    void STLProcessor::convertModelToMeshData(const PrivateModel& modelData, MeshData& meshData)
    {
        meshData.m_vertices.clear();
        meshData.m_normals.clear();
        meshData.m_colors.clear();
        meshData.m_indices.clear();

        // COLOR_MESH vertex data format: position + color (x,y,z,r,g,b,a)
        size_t vertexCount = modelData.m_vertices.size() / 3;
        meshData.m_vertices.reserve(vertexCount * 7); // 7 floats per vertex: x,y,z,r,g,b,a

        // Set default STL mesh color (gray)
        float r = 0.7f, g = 0.7f, b = 0.7f, a = 1.0f;

        for (size_t i = 0; i < vertexCount; ++i) {
            // Position data (x, y, z)
            meshData.m_vertices.push_back(modelData.m_vertices[i * 3 + 0]);
            meshData.m_vertices.push_back(modelData.m_vertices[i * 3 + 1]);
            meshData.m_vertices.push_back(modelData.m_vertices[i * 3 + 2]);

            // Color data (r, g, b, a)
            meshData.m_vertices.push_back(r);
            meshData.m_vertices.push_back(g);
            meshData.m_vertices.push_back(b);
            meshData.m_vertices.push_back(a);
        }

        // Copy index data
        meshData.m_indices = modelData.m_indices;

        if (modelData.m_normals.size() == vertexCount * 3) {
            meshData.m_normals = modelData.m_normals;
        } else {
            meshData.m_normals.reserve(vertexCount * 3);
            for (size_t i = 0; i < vertexCount; ++i) {
                meshData.m_normals.push_back(0.0f);
                meshData.m_normals.push_back(1.0f);
                meshData.m_normals.push_back(0.0f);
            }
        }
    }
}