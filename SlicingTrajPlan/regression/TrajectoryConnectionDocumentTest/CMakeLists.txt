cmake_minimum_required(VERSION 3.20)

add_executable(FreeformTrajPlanTrajectoryConnectionDocumentTest main.cpp)
target_link_libraries(FreeformTrajPlanTrajectoryConnectionDocumentTest
    PRIVATE
        SMRobotSpray::SprayFreeformTrajPlan)
target_compile_features(FreeformTrajPlanTrajectoryConnectionDocumentTest PRIVATE cxx_std_17)
set_target_properties(FreeformTrajPlanTrajectoryConnectionDocumentTest PROPERTIES
    FOLDER tests/SMRobotSpray/SprayFreeformTrajPlan)
add_test(
    NAME FreeformTrajPlanTrajectoryConnectionDocumentTest
    COMMAND $<TARGET_FILE:FreeformTrajPlanTrajectoryConnectionDocumentTest>)
set_tests_properties(FreeformTrajPlanTrajectoryConnectionDocumentTest PROPERTIES
    LABELS "SMRobotSpray;SprayFreeformTrajPlan;Trajectory;Connection;Document")
