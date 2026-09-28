#include <GeneralData/GeneralData.h>
namespace ModelTrajPlan
{
    GeneralData::GeneralData()
        : transformMatrix_(Eigen::Matrix4d::Identity()) 
    {
    }

    GeneralData::~GeneralData()
    {
    }

}