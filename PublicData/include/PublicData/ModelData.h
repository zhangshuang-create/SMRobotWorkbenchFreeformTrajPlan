#pragma once
#include "ModelData_global.h"
#include <vector>
#include <string>
#include <memory>

namespace PublicData
{
    // Public Model Data Structure¡¿ - Independent of the algorithm layer's PrivateModel
    // Used for global model data management and UI layer interaction
    struct PublicModel {
        std::vector<float> m_vertices;         // Vertex coordinates, stored sequentially as x, y, z
        std::vector<float> m_normals;          // Normal vectors, stored sequentially as x, y, z
        std::vector<unsigned int> m_indices;   // Vertex indices for indexing vertices in m_vertices
        std::string m_file_path;               // Model file path
        std::string m_file_name;               // Model file name

        void clear() {
            m_vertices.clear();
            m_normals.clear();
            m_indices.clear();
            m_file_path.clear();
            m_file_name.clear();
        }
    };

    // ¡¾Public Model Data Manager¡¿
    // Responsibilities: STL file loading, model data storage, provision for UI and algorithm layers
    class ModelData_SHAREDLIB PublicModelData
    {
    public:
        PublicModelData();
        ~PublicModelData();

        // Load model data from an STL file
        bool loadModel(const std::string& filePath);

        // Clear model data
        void clear();
        void deleteModel();

        // Get model data (read-only)
        const PublicModel& getModel() const { return m_model; }

        // Get model data (modifiable)
        PublicModel& getModel() { return m_model; }

        // Check if valid data exists
        bool hasValidData() const { return !m_model.m_vertices.empty(); }

    private:
        PublicModel m_model;  // Model data instance

        // Helper functions for STL format detection and loading
        bool isAsciiSTL(const std::string& filePath);
        bool loadAsciiSTL(const std::string& filePath);
        bool loadBinarySTL(const std::string& filePath);
    };
}