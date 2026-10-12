#pragma once

#include <string>
#include <vector>

namespace spray_freeform_traj_plan
{
    enum class SlicingMethod
    {
        Adaptive,
        Uniform
    };

    enum class PlanningMethod
    {
        Slicing,
        Uv
    };

    enum class WorkflowAction
    {
        ExportTrajectory,
        TransformTrajectory,
        ConfigureTrajectoryFrame,
        StartSlicing,
        GenerateTrajectory,
        ConnectTrajectory,
        AdjustSlicing,
        StartUvMap,
        GenerateUvTrajectory,
        OptimizeUv,
        ClearModelData,
        ClearTrajectoryData,
        ClearConnectionData,
        ClearOtherData,
        ClearAllData
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

    struct UvTrajPlanParameters
    {
        double targetSpacingMm = 30.0;
        double trajectorySpeedMmPerSecond = 300.0;
        double sprayDistanceMm = 0.0;
        double maximumSeamSprayDistanceMm = 78.0;
        double maximumAccelerationMmPerSecondSquared = 1000.0;
        double maximumJerkMmPerSecondCubed = 5000.0;
        double overtravelLength = 4.0;
        double arcSpeedLossPercent = 0.0;
        bool showUvSplitView = false;
    };

    struct FreeformTrajectoryPoint
    {
        double positionX = 0.0;
        double positionY = 0.0;
        double positionZ = 0.0;
        double directionX = 0.0;
        double directionY = 0.0;
        double directionZ = 1.0;
        double frameXAxisX = 1.0;
        double frameXAxisY = 0.0;
        double frameXAxisZ = 0.0;
        double frameYAxisX = 0.0;
        double frameYAxisY = 1.0;
        double frameYAxisZ = 0.0;
        double frameZAxisX = 0.0;
        double frameZAxisY = 0.0;
        double frameZAxisZ = 1.0;
        bool sprayEnabled = false;
        bool startsNewSegment = false;
    };

    using FreeformTrajectory = std::vector<FreeformTrajectoryPoint>;

    struct FreeformVec3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct FreeformTriangle
    {
        FreeformVec3 vertices[3];
    };

    struct FreeformSurfaceMesh
    {
        std::vector<FreeformTriangle> triangles;
        FreeformVec3 minimum;
        FreeformVec3 maximum;
    };

    struct FreeformPolyline
    {
        std::vector<FreeformVec3> points;
        std::size_t layerIndex = 0;
        bool closed = false;
    };

    struct FreeformSlicePlane
    {
        FreeformVec3 center;
        FreeformVec3 normal{ 0.0, 0.0, 1.0 };
        FreeformVec3 translation;
        FreeformVec3 rotationDegrees;
        bool selected = false;
    };

    struct FreeformSlicingResult
    {
        std::vector<FreeformPolyline> contours;
        std::vector<double> layerHeights;
        std::vector<FreeformSlicePlane> planes;
        int selectedLayerIndex = -1;
    };

    struct FreeformConnectionSegment
    {
        FreeformVec3 start;
        FreeformVec3 end;
    };

    struct FreeformColor
    {
        double red = 1.0;
        double green = 1.0;
        double blue = 1.0;
        double alpha = 1.0;
    };

    struct FreeformBoundaryLine
    {
        std::vector<FreeformVec3> points;
        FreeformColor color;
        bool closed = true;
    };

    struct FreeformBoundaryResult
    {
        FreeformBoundaryLine upper;
        FreeformBoundaryLine lower;
        FreeformBoundaryLine upperOffset;
        FreeformBoundaryLine lowerOffset;
    };

    struct FreeformTrajPlanCommandResult
    {
        bool success = false;
        std::string message;
    };
}
