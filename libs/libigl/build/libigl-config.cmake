
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was libigl-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(${CMAKE_CURRENT_LIST_DIR}/libigl-export.cmake)

# Check if Eigen3 target is avaiable, if not try to locate it
# with find_package.
message(STATUS "[libigl] Looking for Eigen3")
if (NOT TARGET Eigen3::Eigen)
  # Try if Eigen3 can be found with find_package
  find_package(Eigen3 CONFIG REQUIRED)
endif()


if (TARGET igl::core)
  if (TARGET Eigen3::Eigen)
    # Inject dependency
    set_target_properties(igl::core PROPERTIES INTERFACE_LINK_LIBRARIES Eigen3::Eigen)
    set(libigl_core_FOUND TRUE)
  endif()
endif()

if (TARGET igl::common)
  if (TARGET Eigen3::Eigen)
    # Inject dependency
    set_target_properties(igl::common PROPERTIES INTERFACE_LINK_LIBRARIES Eigen3::Eigen)
    set(libigl_common_FOUND TRUE)
  endif()
endif()

check_required_components(libigl)

