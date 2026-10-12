#pragma once
#include "STLProcessor_global.h"
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <SlicingTrajPlan/GeneralData.h>
namespace ModelTrajPlan
{
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
        // Load directly into the algorithm document's model data.
        bool preprocessSTL(const std::string& filePath, ModelData& modelData);
        // === Data query (read from parameters) ===
        static const PrivateModel& getModelData(const ModelData& modelData)
        {
            return modelData.mainModelData_;
        }
        static bool hasValidData(const ModelData& modelData);

        // === Data cleanup ===
        void clear(ModelData& modelData);

        // === Helper methods ===
        void calculateBounds(const ModelData& modelData, Eigen::Vector3d& min, Eigen::Vector3d& max);

    private:
        // === No internal data storage ===

        // Internal processing methods (pure algorithms)
        PrivateModel parseSTLFile(const std::string& filePath);

    };
}
