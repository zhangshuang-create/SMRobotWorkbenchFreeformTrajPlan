cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME SprayFreeformTrajPlan)

add_library(${TARGET_NAME} SHARED
    src/FreeformTrajPlanDocument.cpp
    include/SprayFreeformTrajPlan/FreeformTrajPlanDocument.h
    include/SprayFreeformTrajPlan/FreeformTrajPlanTypes.h
)
add_library(SMRobotSpray::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)
target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include>
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
        PACKAGE_INTERFACE_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
endif()
