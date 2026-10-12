cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME SprayFreeformTrajPlan)

set(SOURCES
    SlicingTrajPlan/src/BoundaryData.cpp
    SlicingTrajPlan/src/BoundaryDetector.cpp
    SlicingTrajPlan/src/BurTrajPlan.cpp
    SlicingTrajPlan/src/GenralData.cpp
    SlicingTrajPlan/src/OptiAlgorithm.cpp
    SlicingTrajPlan/src/PModelData.cpp
    SlicingTrajPlan/src/PoseTransform.cpp
    SlicingTrajPlan/src/PTrajData.cpp
    SlicingTrajPlan/src/SlicingData.cpp
    SlicingTrajPlan/src/STLProcessor.cpp
    SlicingTrajPlan/src/TrajectoryConnector.cpp
    SlicingTrajPlan/src/TrajectoryExporter.cpp
    SlicingTrajPlan/src/TrajectoryGenerator.cpp
    SlicingTrajPlan/src/UniformSlicer.cpp
)
set(PUBLIC_HEADERS
    SlicingTrajPlan/include/SlicingTrajPlan/BoundaryData.h
    SlicingTrajPlan/include/SlicingTrajPlan/BoundaryDetector.h
    SlicingTrajPlan/include/SlicingTrajPlan/BurTrajPlan.h
    SlicingTrajPlan/include/SlicingTrajPlan/CommonDataTypes.h
    SlicingTrajPlan/include/SlicingTrajPlan/GeneralData.h
    SlicingTrajPlan/include/SlicingTrajPlan/IBurTrajPlan.h
    SlicingTrajPlan/include/SlicingTrajPlan/FreeformTrajPlanTypes.h
    SlicingTrajPlan/include/SlicingTrajPlan/OptiAlgorithm.h
    SlicingTrajPlan/include/SlicingTrajPlan/PModelData.h
    SlicingTrajPlan/include/SlicingTrajPlan/PoseTransform.h
    SlicingTrajPlan/include/SlicingTrajPlan/SlicingData.h
    SlicingTrajPlan/include/SlicingTrajPlan/STLProcessor.h
    SlicingTrajPlan/include/SlicingTrajPlan/TrajectoryConnector.h
    SlicingTrajPlan/include/SlicingTrajPlan/TrajectoryExporter.h
    SlicingTrajPlan/include/SlicingTrajPlan/TrajectoryGenerator.h
    SlicingTrajPlan/include/SlicingTrajPlan/UniformSlicer.h
    SlicingTrajPlan/include/SlicingTrajPlan/TrajTypes.h
)

add_library(${TARGET_NAME} SHARED ${SOURCES} ${PUBLIC_HEADERS})
add_library(SMRobotSpray::${TARGET_NAME} ALIAS ${TARGET_NAME})

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/SlicingTrajPlan" FILES
    ${SOURCES}
    ${PUBLIC_HEADERS}
)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${TARGET_NAME}
    PRIVATE
        BurTrajPlan_EXPORTS
        GeneralData_EXPORTS
        STLProcessor_EXPORTS
        TrajectoryConnector_EXPORTS
        TrajectoryGenerator_EXPORTS
        UniformSlicer_EXPORTS)
target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/SlicingTrajPlan/include>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include>
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        assimp::assimp
)

set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER SMRobotSpray
    WINDOWS_EXPORT_ALL_SYMBOLS TRUE
    DEBUG_POSTFIX _shared_${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX _shared_${CMAKE_RELEASE_POSTFIX}
)

if(ExportPacakge${PACKAGE_NAME}_${TARGET_NAME})
    function_InstallTarget(
        TARGET_NAME ${TARGET_NAME}
        PACKAGE_NAME ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/SlicingTrajPlan/include
    )
endif()

if(BUILD_TESTING)
    add_subdirectory(SlicingTrajPlan/regression)
endif()

# UV planning is a separate algorithm module.  It owns the OptCuts build
# target and must participate in the same build graph as this package.
add_subdirectory(UVTrajPlan)
