cmake_minimum_required(VERSION 3.20)

# The original OptCuts program is hosted by TrajModule as a DLL.  Its command
# line entry has been promoted to the exported runOptCuts() function.
set(_optcuts_default_external_root "${CMAKE_CURRENT_SOURCE_DIR}/ext")
set(OPTCUTS_EXTERNAL_ROOT "${_optcuts_default_external_root}" CACHE PATH
    "Directory containing the OptCuts libigl and tbb source trees")

if(NOT EXISTS "${OPTCUTS_EXTERNAL_ROOT}/libigl/include/igl/readOBJ.h")
    message(FATAL_ERROR
        "OptCuts requires libigl under OPTCUTS_EXTERNAL_ROOT/libigl. "
        "Set -DOPTCUTS_EXTERNAL_ROOT=<OptCuts ext directory>. "
        "For the checked reference tree this is E:/prebuilt/Windows/common/vs2019/OptCuts/ext.")
endif()

if(NOT EXISTS "${OPTCUTS_EXTERNAL_ROOT}/tbb/CMakeLists.txt")
    message(FATAL_ERROR
        "OptCuts requires the reference TBB source tree under "
        "OPTCUTS_EXTERNAL_ROOT/tbb.")
endif()

list(APPEND CMAKE_MODULE_PATH
    "${OPTCUTS_EXTERNAL_ROOT}/libigl/shared/cmake")

# Match the dependency surface used by the checked OptCuts build.  In
# particular, do not let an old libigl enable optional modules whose source is
# absent from the vendored ext tree.
set(LIBIGL_USE_STATIC_LIBRARY OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_ANTTWEAKBAR OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_CGAL OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_COMISO OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_CORK OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_EMBREE OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_LIM OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_MATLAB OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_MOSEK OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_OPENGL ON CACHE BOOL "" FORCE)
set(LIBIGL_WITH_OPENGL_GLFW ON CACHE BOOL "" FORCE)
set(LIBIGL_WITH_OPENGL_GLFW_IMGUI OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_PNG ON CACHE BOOL "" FORCE)
set(LIBIGL_WITH_PYTHON OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_TETGEN OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_TRIANGLE ON CACHE BOOL "" FORCE)
set(LIBIGL_WITH_VIEWER ON CACHE BOOL "" FORCE)
set(LIBIGL_WITH_XML OFF CACHE BOOL "" FORCE)
include(libigl)

if(NOT TARGET TBB::tbb)
    set(TBB_BUILD_STATIC ON CACHE BOOL "" FORCE)
    set(TBB_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(TBB_BUILD_TBBMALLOC OFF CACHE BOOL "" FORCE)
    set(TBB_BUILD_TBBMALLOC_PROXY OFF CACHE BOOL "" FORCE)
    set(TBB_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory("${OPTCUTS_EXTERNAL_ROOT}/tbb" "${CMAKE_CURRENT_BINARY_DIR}/tbb" EXCLUDE_FROM_ALL)
    add_library(TBB::tbb ALIAS tbb_static)
endif()

set(OPTCUTS_SOURCES
    main.cpp
    Optimizer.cpp
    TriMesh.cpp
    Scaffold.cpp
    Energy/Energy.cpp
    Energy/SymDirichletEnergy.cpp
    Utils/IglUtils.cpp
    LinSysSolver/EigenLibSolver.cpp
)

add_library(OptCutsCore SHARED ${OPTCUTS_SOURCES} OptCutsApi.h WindowsCompat.hpp)

target_include_directories(OptCutsCore
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/Energy"
    "${CMAKE_CURRENT_SOURCE_DIR}/Utils"
    "${CMAKE_CURRENT_SOURCE_DIR}/LinSysSolver"
    "${CMAKE_CURRENT_SOURCE_DIR}/UnitTest")

if(MSVC)
    target_compile_options(OptCutsCore PRIVATE
        "/FI${CMAKE_CURRENT_SOURCE_DIR}/WindowsCompat.hpp"
        /FS)
endif()

target_compile_definitions(OptCutsCore PRIVATE OPTCUTS_CORE_EXPORTS)

target_link_libraries(OptCutsCore PRIVATE
    igl::core
    igl::opengl_glfw
    igl::triangle
    igl::png
    TBB::tbb)

set_target_properties(OptCutsCore PROPERTIES
    FOLDER "SMRobotSpray/UVTrajPlan")
