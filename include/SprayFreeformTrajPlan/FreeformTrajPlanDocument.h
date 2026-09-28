#pragma once

#include "FreeformTrajPlanTypes.h"

#include <string>

namespace spray_freeform_traj_plan
{
    class FreeformTrajPlanDocument
    {
    public:
        const std::string& modelPath() const;
        SlicingMethod slicingMethod() const;
        const FreeformTrajPlanParameters& parameters() const;

        FreeformTrajPlanCommandResult setModelPath(std::string path);
        FreeformTrajPlanCommandResult setSlicingMethod(SlicingMethod method);
        FreeformTrajPlanCommandResult setParameters(FreeformTrajPlanParameters parameters);
        FreeformTrajPlanCommandResult requestPlanning();
        void clear();

    private:
        std::string m_modelPath;
        SlicingMethod m_slicingMethod = SlicingMethod::Adaptive;
        FreeformTrajPlanParameters m_parameters;
    };
}
