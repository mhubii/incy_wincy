# Find the rbdl lua model addon.
find_path(rbdl_luamodel_INCLUDE_DIR luamodel.h /usr/include/rbdl/addons/luamodel /usr/local/include/rbdl/addons/luamodel)

find_library(rbdl_luamodel_LIBRARY
    NAMES rbdl_luamodel
    PATHS /usr/lib /usr/local/lib
) 

if (rbdl_luamodel_INCLUDE_DIR AND rbdl_luamodel_LIBRARY)
   set(rbdl_luamodel_FOUND TRUE)
endif (rbdl_luamodel_INCLUDE_DIR AND rbdl_luamodel_LIBRARY)

if (rbdl_luamodel_FOUND)
  if (NOT rbdl_luamodel_FIND_QUIETLY)
    message(STATUS "Found rbdl_luamodel: ${rbdl_luamodel_LIBRARY}")
  endif (NOT rbdl_luamodel_FIND_QUIETLY)
else (rbdl_luamodel_FOUND)
  if (rbdl_luamodel_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find rbdl_luamodel")
  endif (rbdl_luamodel_FIND_REQUIRED)
endif (rbdl_luamodel_FOUND)
