#############################################################
#	CMake requirement (From Modern CMake)
#############################################################
cmake_minimum_required(VERSION 3.20)

get_filename_component(TARGET_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "\n-------------  Config ${PACKAGE_NAME}::${TARGET_NAME}: ${CMAKE_CURRENT_SOURCE_DIR}  -------------")

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp src/*.c)
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS src/*.h src/*.hpp)
file(GLOB_RECURSE INTERFACE_HEADERS CONFIGURE_DEPENDS include/${TARGET_NAME}/*.h include/${TARGET_NAME}/*.hpp)

cmake_path(SET abs_include_dir
    NORMALIZE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
message(STATUS "abs_include_dir is ${abs_include_dir}")

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS} ${INTERFACE_HEADERS})
add_library(${PACKAGE_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPublic}>
    PRIVATE
        $<BUILD_INTERFACE:${${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPrivate}>
)

set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER          ${PACKAGE_NAME}
    DEBUG_POSTFIX   _shared_${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX _shared_${CMAKE_RELEASE_POSTFIX}
)

if(ExportGroup${GROUP_NAME}_${TARGET_NAME})
    function_InstallTarget(
        TARGET_NAME ${TARGET_NAME}
        PACKAGE_NAME ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${abs_include_dir}
    )
endif()

if(BuildExample_${PACKAGE_NAME})
    add_subdirectory(example)
endif()

message(STATUS "-------------  Config ${PACKAGE_NAME}::${TARGET_NAME}: ${CMAKE_CURRENT_SOURCE_DIR}  -------------\n")
