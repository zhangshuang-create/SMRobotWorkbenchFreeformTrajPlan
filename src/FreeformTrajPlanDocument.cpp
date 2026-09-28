#include <SprayFreeformTrajPlan/FreeformTrajPlanDocument.h>

#include <utility>

namespace spray_freeform_traj_plan
{
    const std::string& FreeformTrajPlanDocument::modelPath() const
    {
        return m_modelPath;
    }

    SlicingMethod FreeformTrajPlanDocument::slicingMethod() const
    {
        return m_slicingMethod;
    }

    const FreeformTrajPlanParameters& FreeformTrajPlanDocument::parameters() const
    {
        return m_parameters;
    }

    FreeformTrajPlanCommandResult FreeformTrajPlanDocument::setModelPath(std::string path)
    {
        if(path.empty()) {
            return { false, "Select a surface model before importing." };
        }
        m_modelPath = std::move(path);
        return { true, "Surface model selected." };
    }

    FreeformTrajPlanCommandResult FreeformTrajPlanDocument::setSlicingMethod(
        SlicingMethod method)
    {
        m_slicingMethod = method;
        return { true, "Slicing method updated." };
    }

    FreeformTrajPlanCommandResult FreeformTrajPlanDocument::setParameters(
        FreeformTrajPlanParameters parameters)
    {
        if(parameters.trajectorySpacingMm <= 0.0 || parameters.chordErrorTolerance <= 0.0 ||
            parameters.minimumSpacingMm <= 0.0 || parameters.optimalSprayDistanceMm <= 0.0 ||
            parameters.optimalSpeedMmPerSecond < 0.0) {
            return { false, "Trajectory parameters must use valid positive distances." };
        }
        m_parameters = parameters;
        return { true, "Trajectory parameters updated." };
    }

    FreeformTrajPlanCommandResult FreeformTrajPlanDocument::requestPlanning()
    {
        if(m_modelPath.empty()) {
            return { false, "Import a surface model before generating a trajectory." };
        }
        return { true, "Planning request accepted. The algorithm adapter will generate the trajectory." };
    }

    void FreeformTrajPlanDocument::clear()
    {
        m_modelPath.clear();
        m_slicingMethod = SlicingMethod::Adaptive;
        m_parameters = {};
    }
}
