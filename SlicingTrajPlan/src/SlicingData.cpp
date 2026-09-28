#include <GeneralData/SlicingData.h>
namespace ModelTrajPlan
{
    SlicingData::SlicingData()
        : minZLevel_(0.0)
        , maxZLevel_(0.0)
    {
        // Note: CuttingPlane objects will be initialized with default member initializers
    }

    SlicingData::~SlicingData()
    {
    }

}