#pragma once

#include <PublicThicknessData/PublicThicknessData_global.h>

#include <string>
#include <vector>

namespace PublicData
{
    struct PublicThicknessModel
    {
        std::vector<float> m_vertices;
        std::vector<float> m_normals;
        std::vector<unsigned int> m_indices;
        std::vector<float> m_thickness;
        std::string m_file_path;
        std::string m_file_name;

        void clear()
        {
            m_vertices.clear();
            m_normals.clear();
            m_indices.clear();
            m_thickness.clear();
            m_file_path.clear();
            m_file_name.clear();
        }
    };

    class PublicThicknessData_SHAREDLIB PublicThicknessPredictData
    {
    public:
        PublicThicknessPredictData();
        ~PublicThicknessPredictData();

        void clear();
        void setModel(const PublicThicknessModel& model);

        const PublicThicknessModel& getModel() const { return m_model; }
        PublicThicknessModel& getModel() { return m_model; }

        bool hasValidData() const
        {
            return !m_model.m_vertices.empty() && !m_model.m_thickness.empty();
        }

    private:
        PublicThicknessModel m_model;
    };

    PublicThicknessData_SHAREDLIB PublicThicknessPredictData& globalThicknessPredictData();
}
