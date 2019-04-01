cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(incy)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-fext-numeric-literals) # https://stackoverflow.com/questions/25365160/boostmultiprecisionfloat128-and-c11

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(rbdl REQUIRED)
find_package(rbdl_luamodel REQUIRED)
find_package(rbdl_geometry REQUIRED)
find_package(rbdl_muscle REQUIRED)
find_package(Torch REQUIRED)
find_package(OpenMP)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

include_directories(${rbdl_INCLUDE_DIR})
include_directories(${rbdl_luamodel_INCLUDE_DIR})
include_directories(${rbdl_geometry_INCLUDE_DIR})
include_directories(${rbdl_muscle_INCLUDE_DIR})
include_directories(${TORCH_INCLUDE_DIRS})
include_directories(${OpenMP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

#add_executable(incyWincy src/IncyWincy.cpp src/csvtools.cpp) 
#target_link_libraries(incyWincy ${rbdl_LIBRARY} ${rbdl_luamodel_LIBRARY} ${rbdl_geometry_LIBRARY} ${rbdl_muscle_LIBRARY} ${TORCH_LIBRARIES})

#add_executable(incyWincyDev src/IncyWincyDev.cpp src/csvtools.cpp) 
#target_link_libraries(incyWincyDev ${rbdl_LIBRARY} ${rbdl_luamodel_LIBRARY} ${rbdl_geometry_LIBRARY} ${rbdl_muscle_LIBRARY} ${TORCH_LIBRARIES} ${OpenMP_LIBRARIES})

add_executable(incyWincySimplified src/IncyWincySimplified.cpp src/csvtools.cpp) 
target_link_libraries(incyWincySimplified ${rbdl_LIBRARY} ${rbdl_luamodel_LIBRARY} ${rbdl_geometry_LIBRARY} ${rbdl_muscle_LIBRARY} ${TORCH_LIBRARIES})
