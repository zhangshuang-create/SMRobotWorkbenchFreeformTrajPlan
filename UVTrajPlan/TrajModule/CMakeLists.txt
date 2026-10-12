cmake_minimum_required(VERSION 3.20)

get_filename_component( TARGET_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME )
message( STATUS "\n-------------  Config ${PACKAGE_NAME}::${TARGET_NAME}: ${CMAKE_CURRENT_SOURCE_DIR}  -------------" )

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS ../src/*.cpp ../src/*.c )
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../src/main.cpp")
if(SPRAYTRAJPLAN_HAS_CUDA)
    list(APPEND SOURCES ../src/GpuMeshProjection.cu)
endif()
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS ../src/*.h ../src/*.hpp )
file(GLOB_RECURSE INTERFACE_HEADERS CONFIGURE_DEPENDS ../include/${PACKAGE_NAME}/*.h ../include/${PACKAGE_NAME}/*.hpp )

cmake_path(SET abs_include_dir NORMALIZE "${CMAKE_CURRENT_SOURCE_DIR}/../include")

add_library( ${TARGET_NAME} SHARED ${SOURCES} ${HEADERS} ${INTERFACE_HEADERS} )
add_library( ${PACKAGE_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_include_directories(${TARGET_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/${PACKAGE_NAME}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
)

target_link_libraries ( ${TARGET_NAME}
    PUBLIC $<BUILD_INTERFACE:${${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPublic}>
    PRIVATE $<BUILD_INTERFACE:${${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPrivate}>
)

if(SPRAYTRAJPLAN_HAS_CUDA)
    find_package(CUDAToolkit REQUIRED)
    target_link_libraries(${TARGET_NAME} PRIVATE CUDA::cudart)
    target_compile_definitions(${TARGET_NAME} PRIVATE SPRAYTRAJPLAN_HAS_CUDA=1)
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:CUDA::cudart> $<TARGET_FILE_DIR:${TARGET_NAME}>
    )
endif()

set_target_properties( ${TARGET_NAME} PROPERTIES
    FOLDER ${PACKAGE_NAME}
    WINDOWS_EXPORT_ALL_SYMBOLS TRUE
    DEBUG_POSTFIX _shared_${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX _shared_${CMAKE_RELEASE_POSTFIX}
)

message( STATUS "-------------  Config ${PACKAGE_NAME}::${TARGET_NAME}: ${CMAKE_CURRENT_SOURCE_DIR}  -------------\n" )
