cmake_minimum_required(VERSION 3.20)

add_executable(FreeformTrajPlanSlicePlaneDocumentTest main.cpp)
target_link_libraries(FreeformTrajPlanSlicePlaneDocumentTest
    PRIVATE
        SMRobotSpray::SprayFreeformTrajPlan)
target_compile_features(FreeformTrajPlanSlicePlaneDocumentTest PRIVATE cxx_std_17)
set_target_properties(FreeformTrajPlanSlicePlaneDocumentTest PROPERTIES
    FOLDER tests/SMRobotSpray/SprayFreeformTrajPlan)
add_test(
    NAME FreeformTrajPlanSlicePlaneDocumentTest
    COMMAND $<TARGET_FILE:FreeformTrajPlanSlicePlaneDocumentTest>)
set_tests_properties(FreeformTrajPlanSlicePlaneDocumentTest PROPERTIES
    LABELS "SMRobotSpray;SprayFreeformTrajPlan;SlicePlaneDocument")
