#include<ModelData/ModelData.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstring>

namespace PublicData
{
    PublicModelData::PublicModelData()
    {
    }

    PublicModelData::~PublicModelData()
    {
    }

    // Detect STL file format (ASCII or Binary)
    bool PublicModelData::isAsciiSTL(const std::string& filePath)
    {
        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open()) {
            return false;
        }

        // Read first 80 bytes
        char buffer[80];
        file.read(buffer, 80);
        std::streamsize bytesRead = file.gcount();
        file.close();

        if (bytesRead < 5) {
            return false;
        }

        // Check if file starts with "solid"
        if (std::strncmp(buffer, "solid", 5) != 0) {
            return false; // Binary format
        }

        // If starts with "solid", verify it's actually ASCII by checking file size
        std::ifstream sizeCheck(filePath, std::ios::binary | std::ios::ate);
        std::streamsize fileSize = sizeCheck.tellg();
        sizeCheck.close();

        // Binary STL size should be: 80 (header) + 4 (count) + 50*n (triangles)
        // If (fileSize - 84) % 50 == 0, it might be binary with "solid" in header
        if (fileSize >= 84 && (fileSize - 84) % 50 == 0) {
            return false; // Likely binary format
        }

        return true; // ASCII format
    }

    // Load ASCII STL format
    bool PublicModelData::loadAsciiSTL(const std::string& filePath)
    {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Error: Failed to open ASCII STL file - " << filePath << std::endl;
            return false;
        }

        std::string line;
        unsigned int triangleIndex = 0;

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string keyword;
            iss >> keyword;

            if (keyword == "facet") {
                // Read normal
                std::string normalKeyword;
                float nx, ny, nz;
                iss >> normalKeyword >> nx >> ny >> nz;

                // Read outer loop
                std::getline(file, line); // "outer loop"

                // Read three vertices
                float vertices[9];
                for (int i = 0; i < 3; ++i) {
                    std::getline(file, line);
                    std::istringstream viss(line);
                    std::string vertexKeyword;
                    viss >> vertexKeyword >> vertices[i * 3 + 0] >> vertices[i * 3 + 1] >> vertices[i * 3 + 2];
                }

                // Read endloop and endfacet
                std::getline(file, line); // "endloop"
                std::getline(file, line); // "endfacet"

                // Add vertices to the vertex list
                for (int j = 0; j < 9; ++j) {
                    m_model.m_vertices.push_back(vertices[j]);
                }

                // Add normal for each vertex
                for (int j = 0; j < 3; ++j) {
                    m_model.m_normals.push_back(nx);
                    m_model.m_normals.push_back(ny);
                    m_model.m_normals.push_back(nz);
                }

                // Add triangle indices
                unsigned int baseIndex = triangleIndex * 3;
                m_model.m_indices.push_back(baseIndex + 0);
                m_model.m_indices.push_back(baseIndex + 1);
                m_model.m_indices.push_back(baseIndex + 2);

                triangleIndex++;
            }
        }

        file.close();
        return triangleIndex > 0;
    }

    // Load Binary STL format
    bool PublicModelData::loadBinarySTL(const std::string& filePath)
    {
        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Error: Failed to open binary STL file - " << filePath << std::endl;
            return false;
        }

        // Read STL header (80 bytes)
        char header[80];
        file.read(header, 80);

        // Read triangle count
        uint32_t triangleCount;
        file.read(reinterpret_cast<char*>(&triangleCount), sizeof(uint32_t));

        // Pre-allocate memory
        m_model.m_vertices.reserve(triangleCount * 9);  // 3 vertices * 3 coordinates
        m_model.m_normals.reserve(triangleCount * 9);   // 3 vertices * 3 normal components
        m_model.m_indices.reserve(triangleCount * 3);   // 3 indices per triangle

        // Read triangle data
        for (uint32_t i = 0; i < triangleCount; ++i) {
            // Read face normal
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

            // Add vertices to the vertex list
            for (int j = 0; j < 9; ++j) {
                m_model.m_vertices.push_back(vertices[j]);
            }

            // Add normal for each vertex (all three vertices share the same face normal)
            for (int j = 0; j < 3; ++j) {
                m_model.m_normals.push_back(normal[0]);
                m_model.m_normals.push_back(normal[1]);
                m_model.m_normals.push_back(normal[2]);
            }

            // Add triangle indices (3 consecutive vertices)
            unsigned int baseIndex = i * 3;
            m_model.m_indices.push_back(baseIndex + 0);
            m_model.m_indices.push_back(baseIndex + 1);
            m_model.m_indices.push_back(baseIndex + 2);

            // Read attribute byte count (2 bytes)
            uint16_t attributeByteCount;
            file.read(reinterpret_cast<char*>(&attributeByteCount), sizeof(uint16_t));
        }

        file.close();
        return true;
    }

    bool PublicModelData::loadModel(const std::string& filePath)
    {
        // Clear existing data
        deleteModel();

        // Save file path and extract file name
        m_model.m_file_path = filePath;
        size_t lastSlash = filePath.find_last_of("/\\");
        m_model.m_file_name = (lastSlash != std::string::npos) ? filePath.substr(lastSlash + 1) : filePath;

        // Detect file format and load accordingly
        if (isAsciiSTL(filePath)) {
            std::cout << "Loading ASCII STL format..." << std::endl;
            return loadAsciiSTL(filePath);
        } else {
            std::cout << "Loading Binary STL format..." << std::endl;
            return loadBinarySTL(filePath);
        }
    }

    void PublicModelData::deleteModel()
    {
        clear();
    }

    void PublicModelData::clear()
    {
        // Clear all data
        m_model.clear();

        // Release memory
        m_model.m_vertices.shrink_to_fit();
        m_model.m_normals.shrink_to_fit();
        m_model.m_indices.shrink_to_fit();
    }
}