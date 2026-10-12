cmake_minimum_required(VERSION 3.20)

add_executable(FreeformTrajPlanModelLoadingTest main.cpp)
target_link_libraries(FreeformTrajPlanModelLoadingTest
    PRIVATE
        SMRobotSpray::SprayFreeformTrajPlan)
target_compile_features(FreeformTrajPlanModelLoadingTest PRIVATE cxx_std_17)
set_target_properties(FreeformTrajPlanModelLoadingTest PROPERTIES
    FOLDER tests/SMRobotSpray/SprayFreeformTrajPlan)
add_test(
    NAME FreeformTrajPlanModelLoadingTest
    COMMAND $<TARGET_FILE:FreeformTrajPlanModelLoadingTest>)
set_tests_properties(FreeformTrajPlanModelLoadingTest PROPERTIES
    LABELS "SMRobotSpray;SprayFreeformTrajPlan;ModelLoading")
