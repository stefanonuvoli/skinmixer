#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "sys" for configuration "Release"
set_property(TARGET sys APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(sys PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "-lpthread;dl"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsys.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS sys )
list(APPEND _IMPORT_CHECK_FILES_FOR_sys "${_IMPORT_PREFIX}/lib/libsys.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
