# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /usr/include/libigl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /usr/include/libigl/build

# Utility rule file for ExperimentalMemCheck.

# Include the progress variables for this target.
include embree/CMakeFiles/ExperimentalMemCheck.dir/progress.make

embree/CMakeFiles/ExperimentalMemCheck:
	cd /usr/include/libigl/build/embree && /usr/bin/ctest -D ExperimentalMemCheck

ExperimentalMemCheck: embree/CMakeFiles/ExperimentalMemCheck
ExperimentalMemCheck: embree/CMakeFiles/ExperimentalMemCheck.dir/build.make

.PHONY : ExperimentalMemCheck

# Rule to build all files generated by this target.
embree/CMakeFiles/ExperimentalMemCheck.dir/build: ExperimentalMemCheck

.PHONY : embree/CMakeFiles/ExperimentalMemCheck.dir/build

embree/CMakeFiles/ExperimentalMemCheck.dir/clean:
	cd /usr/include/libigl/build/embree && $(CMAKE_COMMAND) -P CMakeFiles/ExperimentalMemCheck.dir/cmake_clean.cmake
.PHONY : embree/CMakeFiles/ExperimentalMemCheck.dir/clean

embree/CMakeFiles/ExperimentalMemCheck.dir/depend:
	cd /usr/include/libigl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/include/libigl /usr/include/libigl/external/embree /usr/include/libigl/build /usr/include/libigl/build/embree /usr/include/libigl/build/embree/CMakeFiles/ExperimentalMemCheck.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : embree/CMakeFiles/ExperimentalMemCheck.dir/depend

