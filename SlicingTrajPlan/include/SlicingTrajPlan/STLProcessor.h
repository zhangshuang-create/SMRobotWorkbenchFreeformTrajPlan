#pragma once
#include "STLProcessor_global.h"
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <ModelTangQing/iscene.h>
#include <ModelTangQing/imodel.h>
#include <GeneralData/GeneralData.h>
#include <RenderData/RenderData.h>
#include <ModelData/ModelData.h>  // Include PublicModel definition
namespace ModelTrajPlan
{
    // Forward declaration
    struct PublicModel;

    // STL processor class
    class STLProcessor_SHAREDLIB STLProcessor
    {
    public:
        STLProcessor();
        virtual ~STLProcessor();

        // === Static Model helper functions ===
        static size_t getTriangleCount(const PrivateModel& modelData);
        static size_t getVertexCount(const PrivateModel& modelData);
        static void calculateBounds(const PrivateModel& modelData, Eigen::Vector3d& bounds_min, Eigen::Vector3d& bounds_max);
        static void getTriangleVertices(const PrivateModel& modelData, size_t triangleIndex, Eigen::Vector3d vertices[3]);
        static Eigen::Vector3d getTriangleNormal(const PrivateModel& modelData, size_t triangleIndex);


        // === STL file processing ===
    // [Deprecated] File loading API - prefer PublicModelData::loadModelFromFile() and then call the function below
        bool preprocessSTL(const std::string& filePath, ModelData& modelData);

        // [Recommended] Copy data from PublicModel to GeneralData
        static bool copyFromPublicModel(const PublicData::PublicModel& publicModel, ModelData& modelData);
        // === Data query (read from parameters) ===
        static const PrivateModel& getModelData(const ModelData& modelData)
        {
            return modelData.mainModelData_;
        }
        static bool hasValidData(const ModelData& modelData);

        // === Rendering support (accepts RenderData as parameter) ===
        bool initializeRenderer(STLRenderData& renderData);
        bool draw(STLRenderData& renderData, const ModelData& modelData);
        void setVisibility(STLRenderData& renderData, bool visible);

        // === Data cleanup ===
        void clear(ModelData& modelData);

        // === Helper methods ===
        void calculateBounds(const ModelData& modelData, Eigen::Vector3d& min, Eigen::Vector3d& max);
        void updateRenderData(STLRenderData& renderData, const ModelData& modelData);

    private:
        // === No internal data storage ===

        // Internal processing methods (pure algorithms)
        PrivateModel parseSTLFile(const std::string& filePath);

        // [REFACTOR] Data conversion methods (pure data processing, no rendering)
        static MeshData convertModelToMeshData(const PrivateModel& modelData);  // Return-by-value version
        static void convertModelToMeshData(const PrivateModel& modelData, MeshData& meshData);  // Reference-parameter version (avoids temporaries)
    };
}