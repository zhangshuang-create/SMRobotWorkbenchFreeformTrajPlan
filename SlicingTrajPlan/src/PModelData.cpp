#include <GeneralData/PModelData.h>
#include <fstream>
#include <iostream>
namespace ModelTrajPlan
{
    ModelData::ModelData()
        : totalTriangleCount_(0)
        , totalVertexCount_(0)
        , hasValidModelData_(false)
        , isModelLoaded_(false)
    {
    }

    ModelData::~ModelData()
    {
    }

    bool ModelData::loadFromFile(const std::string& filePath)
    {
        // Clear existing data
        clear();

        // Parse STL file
        PrivateModel model = parseSTLFile(filePath);

        // Check whether loading succeeded
        if (model.m_vertices.empty()) {
            std::cerr << "Error: Failed to load model from " << filePath << std::endl;
            return false;
        }

        // Store into mainModelData_
        mainModelData_ = model;
        allModelsData_.push_back(model);
        loadedFilePaths_.push_back(filePath);

        // Update statistics
        totalTriangleCount_ = model.m_indices.size() / 3;
        totalVertexCount_ = model.m_vertices.size() / 3;
        hasValidModelData_ = true;
        isModelLoaded_ = true;

        std::cout << "ModelData: Loaded successfully - " << model.m_file_name << std::endl;
        std::cout << "  Triangles: " << totalTriangleCount_ << std::endl;
        std::cout << "  Vertices: " << totalVertexCount_ << std::endl;

        return true;
    }

    void ModelData::clear()
    {
        mainModelData_.m_vertices.clear();
        mainModelData_.m_normals.clear();
        mainModelData_.m_indices.clear();
        mainModelData_.m_file_path.clear();
        mainModelData_.m_file_name.clear();

        allModelsData_.clear();
        loadedFilePaths_.clear();

        totalTriangleCount_ = 0;
        totalVertexCount_ = 0;
        hasValidModelData_ = false;
        isModelLoaded_ = false;
    }

    PrivateModel ModelData::parseSTLFile(const std::string& filePath)
    {
        PrivateModel model;
        model.m_file_path = filePath;

        // Extract filename
        size_t lastSlash = filePath.find_last_of("/\\");
        model.m_file_name = (lastSlash != std::string::npos) ? filePath.substr(lastSlash + 1) : filePath;

        // Open STL binary file
        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Error: Cannot open file - " << filePath << std::endl;
            return model;
        }

        // Read STL file header (80 bytes)
        char header[80];
        file.read(header, 80);

        // Read triangle count
        uint32_t triangleCount;
        file.read(reinterpret_cast<char*>(&triangleCount), sizeof(uint32_t));

        // Pre-allocate memory
        model.m_vertices.reserve(triangleCount * 9);   // 3 vertices * 3 coordinates
        model.m_normals.reserve(triangleCount * 9);    // 3 vertices * 3 normal components
        model.m_indices.reserve(triangleCount * 3);    // 3 indices

        // Read each triangle
        for (uint32_t i = 0; i < triangleCount; ++i) {
            // Read normal vector
            float normal[3];
            file.read(reinterpret_cast<char*>(normal), 3 * sizeof(float));

            // Read 3 vertices
            float vertices[9];
            for (int j = 0; j < 3; ++j) {
                float vertex[3];
                file.read(reinterpret_cast<char*>(vertex), 3 * sizeof(float));
                vertices[j * 3 + 0] = vertex[0];
                vertices[j * 3 + 1] = vertex[1];
                vertices[j * 3 + 2] = vertex[2];
            }

            // Append vertices
            for (int j = 0; j < 9; ++j) {
                model.m_vertices.push_back(vertices[j]);
            }

            // Append normal vector for each vertex (3 vertices share one normal)
            for (int j = 0; j < 3; ++j) {
                model.m_normals.push_back(normal[0]);
                model.m_normals.push_back(normal[1]);
                model.m_normals.push_back(normal[2]);
            }

            // Append indices
            unsigned int baseIndex = i * 3;
            model.m_indices.push_back(baseIndex + 0);
            model.m_indices.push_back(baseIndex + 1);
            model.m_indices.push_back(baseIndex + 2);

            // Read attribute byte count (2 bytes)
            uint16_t attributeByteCount;
            file.read(reinterpret_cast<char*>(&attributeByteCount), sizeof(uint16_t));
        }

        file.close();
        return model;
    }

}