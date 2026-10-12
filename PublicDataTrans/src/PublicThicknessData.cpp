#include <PublicThicknessData/PublicThicknessData.h>

namespace PublicData
{
    PublicThicknessPredictData::PublicThicknessPredictData() = default;

    PublicThicknessPredictData::~PublicThicknessPredictData() = default;

    void PublicThicknessPredictData::clear()
    {
        m_model.clear();
    }

    void PublicThicknessPredictData::setModel(const PublicThicknessModel& model)
    {
        m_model = model;
    }

    PublicThicknessPredictData& globalThicknessPredictData()
    {
        static PublicThicknessPredictData instance;
        return instance;
    }
}
