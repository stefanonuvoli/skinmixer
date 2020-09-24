#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "igl::core" for configuration "Release"
set_property(TARGET igl::core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(igl::core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libigl.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS igl::core )
list(APPEND _IMPORT_CHECK_FILES_FOR_igl::core "${_IMPORT_PREFIX}/lib/libigl.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
