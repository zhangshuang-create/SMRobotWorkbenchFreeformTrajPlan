cmake_minimum_required(VERSION 3.20)
project(triangle)

add_library(triangle STATIC triangle.c)
target_compile_definitions(triangle PRIVATE -DTRILIBRARY -DANSI_DECLARATORS)
if(WIN32)
   target_compile_definitions(triangle PRIVATE -DNO_TIMER)
endif()
