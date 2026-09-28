#include <GeneralData/BoundaryData.h>
#include <algorithm>
namespace ModelTrajPlan
{
    BoundaryData::BoundaryData()
        : z_max_boundary_(0.0)
        , z_min_boundary_(0.0)
        , epsilon_(5.0)
        , minStep_(50.0)
    {
    }

    BoundaryData::~BoundaryData()
    {
    }

    BoundaryLine* BoundaryData::findBoundaryLineById(BoundaryLineType Type)
    {
        auto it = std::find_if(boundaryLines_.begin(), boundaryLines_.end(),
            [Type](const BoundaryLine& line) {
                return line.Type == Type;
            });

        if (it != boundaryLines_.end()) {
            return &(*it);
        }
        return nullptr;
    }

    const BoundaryLine* BoundaryData::findBoundaryLineById(BoundaryLineType Type) const
    {
        auto it = std::find_if(boundaryLines_.begin(), boundaryLines_.end(),
            [Type](const BoundaryLine& line) {
                return line.Type == Type;
            });

        if (it != boundaryLines_.end()) {
            return &(*it);
        }
        return nullptr;
    }

}
