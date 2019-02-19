# Find the rbdl geometry addon.
find_path(rbdl_geometry_INCLUDE_DIR geometry.h /usr/include/rbdl/addons/geometry /usr/local/include/rbdl/addons/geometry)

find_library(rbdl_geometry_LIBRARY
    NAMES rbdl_geometry
    PATHS /usr/lib /usr/local/lib
) 

if (rbdl_geometry_INCLUDE_DIR AND rbdl_geometry_LIBRARY)
   set(rbdl_geometry_FOUND TRUE)
endif (rbdl_geometry_INCLUDE_DIR AND rbdl_geometry_LIBRARY)

if (rbdl_geometry_FOUND)
  if (NOT rbdl_geometry_FIND_QUIETLY)
    message(STATUS "Found rbdl_geometry: ${rbdl_geometry_LIBRARY}")
  endif (NOT rbdl_geometry_FIND_QUIETLY)
else (rbdl_geometry_FOUND)
  if (rbdl_geometry_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find rbdl_geometry")
  endif (rbdl_geometry_FIND_REQUIRED)
endif (rbdl_geometry_FOUND)
