#pragma once

#include <vector>
#include <string>
#include <GeneralData/GeneralData_global.h>
// Model data structure - vertices, normals, indices
// [Algorithm-layer model data structure] - used for algorithm processing
// Note: although PublicModel and PrivateModel share the same structure, they belong to different modules
// PublicModel is defined in PublicData/ModelData for global data management
// PrivateModel is defined here for algorithm-layer processing

namespace ModelTrajPlan
{
    struct PrivateModel {
        std::vector<float> m_vertices;         // Vertex coordinate data, stored continuously in x, y, z order
        std::vector<float> m_normals;          // Normal-vector data, stored continuously in x, y, z order
        std::vector<unsigned int> m_indices;   // Vertex indices for indexing vertices in m_vertices
        std::string m_file_path;               // Model file path
        std::string m_file_name;               // Model file name
    };

    class GeneralData_SHAREDLIB ModelData
    {
    public:
        ModelData();
        ~ModelData();

        // === Model-loading methods (addition) ===
        // Load model from STL file into mainModelData_
        bool loadFromFile(const std::string& filePath);

        // Clear all model data
        void clear();

        // Check whether valid data exists
        bool hasValidData() const { return hasValidModelData_; }

        // Data members - public access, pure data container
        PrivateModel mainModelData_;              // Main model data
        std::vector<PrivateModel> allModelsData_; // Collection of all model data

        // Model file info
        std::vector<std::string> loadedFilePaths_;  // List of loaded file paths

        // Model statistics
        size_t totalTriangleCount_;            // Total triangle count
        size_t totalVertexCount_;              // Total vertex count

        // State flags
        bool hasValidModelData_;               // Whether valid model data exists
        bool isModelLoaded_;                   // Whether model is loaded

    private:
        // Internal helper: parse STL binary file
        PrivateModel parseSTLFile(const std::string& filePath);
    };

}