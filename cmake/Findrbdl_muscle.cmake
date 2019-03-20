# Find the rbdl library.
find_path(rbdl_muscle_INCLUDE_DIR muscle.h /usr/include/rbdl/addons/muscle /usr/local/include/rbdl/addons/muscle)

find_library(rbdl_muscle_LIBRARY
    NAMES rbdl_muscle
    PATHS /usr/lib /usr/local/lib
) 

if (rbdl_muscle_INCLUDE_DIR AND rbdl_muscle_LIBRARY)
   set(rbdl_muscle_FOUND TRUE)
endif (rbdl_muscle_INCLUDE_DIR AND rbdl_muscle_LIBRARY)

if (rbdl_muscle_FOUND)
  if (NOT rbdl_muscle_FIND_QUIETLY)
    message(STATUS "Found rbdl_muscle: ${rbdl_muscle_LIBRARY}")
  endif (NOT rbdl_muscle_FIND_QUIETLY)
else (rbdl_muscle_FOUND)
  if (rbdl_muscle_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find rbdl_muscle")
  endif (rbdl_muscle_FIND_REQUIRED)
endif (rbdl_muscle_FOUND)
