CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )

SET(CMAKE_CXX_STANDARD 11)

PROJECT( "bouncingBallBenchmark" CXX C )
SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")

LIST( APPEND CMAKE_MODULE_PATH
  ${PROJECT_SOURCE_DIR}/CMake
  ${PROJECT_SOURCE_DIR}/CMake/Modules)

SET(TARGETS    
  bouncingBallBenchmark
  incyWincyBenchmark
  CACHE STRING
  "Target to build"  
)

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLE} PROPERTIES
  LINKER_LANGUAGE CXX
)

FIND_PACKAGE (RBDL COMPONENTS LUAMODEL REQUIRED)
FIND_PACKAGE (Lua51 REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 )

SET(BOOST_INC_DIR
  /usr/include
  CACHE STRING
  "Location of the BOOST header files")


INCLUDE_DIRECTORIES ( 
  ${BOOST_INC_DIR} 
  ${RBDL_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIR} 
  ${LUA_INCLUDE_DIR}  
  ${RBDL_GEOMETRY_INCLUDE_DIR}      
)

LIST(REMOVE_DUPLICATES TARGETS)

FOREACH (TARGET ${TARGETS})
  ADD_EXECUTABLE(${TARGET} 
    src/${TARGET}.cc 
    src/ContactToolkit.h
    src/csvtools.h
    src/csvtools.cc
  )  

  TARGET_LINK_LIBRARIES ( 
    ${TARGET}
    ${RBDL_LIBRARY}
    ${RBDL_LUAMODEL_LIBRARY}  
    ${LUA_LIBRARIES}
    ${RBDL_GEOMETRY_LIBRARY}    
  )
ENDFOREACH ()

message("CUSTOM_RBDL_PATH         :" ${CUSTOM_RBDL_PATH})
message("RBDL_INCLUDE_DIR         :" ${RBDL_INCLUDE_DIR})
message("RBDL_LIBRARY             :" ${RBDL_LIBRARY})
message("RBDL_LUAMODEL_LIBRARY    :" ${RBDL_LUAMODEL_LIBRARY})
message("RBDL_MUSCLE_INCLUDE      :" ${RBDL_MUSCLE_INCLUDE_DIR})
message("RBDL_MUSCLE_LIBRARY      :" ${RBDL_MUSCLE_LIBRARY})
message("RBDL_GEOMETRY_INCLUDE    :" ${RBDL_GEOMETRY_INCLUDE_DIR})
message("RBDL_GEOMETRY_LIBRARY    :" ${RBDL_GEOMETRY_LIBRARY})
message("LUA_LIBRARIES            :" ${LUA_LIBRARIES})
message("EIGEN3_INCLUDE_DIR       :" ${EIGEN3_INCLUDE_DIR})