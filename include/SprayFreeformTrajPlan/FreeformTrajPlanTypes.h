#pragma once

#include <string>

namespace spray_freeform_traj_plan
{
    enum class SlicingMethod
    {
        Adaptive,
        Uniform
    };

    struct FreeformTrajPlanParameters
    {
        double trajectorySpacingMm = 1.0;
        double chordErrorTolerance = 5.0;
        double minimumSpacingMm = 50.0;
        double largeEndOffsetMm = 0.0;
        double smallEndOffsetMm = 0.0;
        double optimalSprayDistanceMm = 1.0;
        double optimalSpeedMmPerSecond = 0.0;
    };

    struct FreeformTrajPlanCommandResult
    {
        bool success = false;
        std::string message;
    };
}
