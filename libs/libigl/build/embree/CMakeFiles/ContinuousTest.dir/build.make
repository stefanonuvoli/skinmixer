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

# Utility rule file for ContinuousTest.

# Include the progress variables for this target.
include embree/CMakeFiles/ContinuousTest.dir/progress.make

embree/CMakeFiles/ContinuousTest:
	cd /usr/include/libigl/build/embree && /usr/bin/ctest -D ContinuousTest

ContinuousTest: embree/CMakeFiles/ContinuousTest
ContinuousTest: embree/CMakeFiles/ContinuousTest.dir/build.make

.PHONY : ContinuousTest

# Rule to build all files generated by this target.
embree/CMakeFiles/ContinuousTest.dir/build: ContinuousTest

.PHONY : embree/CMakeFiles/ContinuousTest.dir/build

embree/CMakeFiles/ContinuousTest.dir/clean:
	cd /usr/include/libigl/build/embree && $(CMAKE_COMMAND) -P CMakeFiles/ContinuousTest.dir/cmake_clean.cmake
.PHONY : embree/CMakeFiles/ContinuousTest.dir/clean

embree/CMakeFiles/ContinuousTest.dir/depend:
	cd /usr/include/libigl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/include/libigl /usr/include/libigl/external/embree /usr/include/libigl/build /usr/include/libigl/build/embree /usr/include/libigl/build/embree/CMakeFiles/ContinuousTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : embree/CMakeFiles/ContinuousTest.dir/depend

