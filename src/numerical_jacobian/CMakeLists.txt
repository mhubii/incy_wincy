cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(numericalJacobian)

set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(rbdl REQUIRED)
find_package(rbdl_luamodel REQUIRED)
find_package(rbdl_geometry REQUIRED)

include_directories(${rbdl_INCLUDE_DIR})
include_directories(${rbdl_luamodel_INCLUDE_DIR})
include_directories(${rbdl_geometry_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(numericalJacobian numerical_jacobian.cpp csvtools.cc)
target_link_libraries(numericalJacobian ${rbdl_LIBRARY} ${rbdl_luamodel_LIBRARY} ${rbdl_geometry_LIBRARY})