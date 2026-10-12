cmake_minimum_required(VERSION 3.20)

add_executable(FreeformTrajPlanBoundaryDetectionTest main.cpp)
target_link_libraries(FreeformTrajPlanBoundaryDetectionTest
    PRIVATE
        SMRobotSpray::SprayFreeformTrajPlan)
target_compile_features(FreeformTrajPlanBoundaryDetectionTest PRIVATE cxx_std_17)
set_target_properties(FreeformTrajPlanBoundaryDetectionTest PROPERTIES
    FOLDER tests/SMRobotSpray/SprayFreeformTrajPlan)
add_test(
    NAME FreeformTrajPlanBoundaryDetectionTest
    COMMAND $<TARGET_FILE:FreeformTrajPlanBoundaryDetectionTest>)
set_tests_properties(FreeformTrajPlanBoundaryDetectionTest PROPERTIES
    LABELS "SMRobotSpray;SprayFreeformTrajPlan;BoundaryDetection")
