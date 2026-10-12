cmake_minimum_required(VERSION 3.20)

add_executable(FreeformTrajPlanBoundarylessUniformSlicingTest main.cpp)
target_link_libraries(FreeformTrajPlanBoundarylessUniformSlicingTest
    PRIVATE
        SMRobotSpray::SprayFreeformTrajPlan)
target_compile_features(FreeformTrajPlanBoundarylessUniformSlicingTest PRIVATE cxx_std_17)
set_target_properties(FreeformTrajPlanBoundarylessUniformSlicingTest PROPERTIES
    FOLDER tests/SMRobotSpray/SprayFreeformTrajPlan)
add_test(
    NAME FreeformTrajPlanBoundarylessUniformSlicingTest
    COMMAND $<TARGET_FILE:FreeformTrajPlanBoundarylessUniformSlicingTest>)
set_tests_properties(FreeformTrajPlanBoundarylessUniformSlicingTest PROPERTIES
    LABELS "SMRobotSpray;SprayFreeformTrajPlan;BoundarylessSlicing")
