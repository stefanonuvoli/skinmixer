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

# Include any dependencies generated for this target.
include tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/depend.make

# Include the progress variables for this target.
include tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/progress.make

# Include the compile flags for this target's objects.
include tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/flags.make

tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.o: tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/flags.make
tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.o: ../tutorial/403_BoundedBiharmonicWeights/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/usr/include/libigl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.o"
	cd /usr/include/libigl/build/tutorial/403_BoundedBiharmonicWeights && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.o -c /usr/include/libigl/tutorial/403_BoundedBiharmonicWeights/main.cpp

tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.i"
	cd /usr/include/libigl/build/tutorial/403_BoundedBiharmonicWeights && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/include/libigl/tutorial/403_BoundedBiharmonicWeights/main.cpp > CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.i

tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.s"
	cd /usr/include/libigl/build/tutorial/403_BoundedBiharmonicWeights && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/include/libigl/tutorial/403_BoundedBiharmonicWeights/main.cpp -o CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.s

# Object files for target 403_BoundedBiharmonicWeights_bin
403_BoundedBiharmonicWeights_bin_OBJECTS = \
"CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.o"

# External object files for target 403_BoundedBiharmonicWeights_bin
403_BoundedBiharmonicWeights_bin_EXTERNAL_OBJECTS =

tutorial/403_BoundedBiharmonicWeights_bin: tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/main.cpp.o
tutorial/403_BoundedBiharmonicWeights_bin: tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/build.make
tutorial/403_BoundedBiharmonicWeights_bin: libigl.a
tutorial/403_BoundedBiharmonicWeights_bin: libigl_opengl.a
tutorial/403_BoundedBiharmonicWeights_bin: libigl_opengl_glfw.a
tutorial/403_BoundedBiharmonicWeights_bin: libigl_opengl.a
tutorial/403_BoundedBiharmonicWeights_bin: libigl.a
tutorial/403_BoundedBiharmonicWeights_bin: /usr/lib/x86_64-linux-gnu/libGLX.so
tutorial/403_BoundedBiharmonicWeights_bin: /usr/lib/x86_64-linux-gnu/libOpenGL.so
tutorial/403_BoundedBiharmonicWeights_bin: libglad.a
tutorial/403_BoundedBiharmonicWeights_bin: libglfw3.a
tutorial/403_BoundedBiharmonicWeights_bin: /usr/lib/x86_64-linux-gnu/librt.so
tutorial/403_BoundedBiharmonicWeights_bin: /usr/lib/x86_64-linux-gnu/libm.so
tutorial/403_BoundedBiharmonicWeights_bin: /usr/lib/x86_64-linux-gnu/libX11.so
tutorial/403_BoundedBiharmonicWeights_bin: tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/usr/include/libigl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../403_BoundedBiharmonicWeights_bin"
	cd /usr/include/libigl/build/tutorial/403_BoundedBiharmonicWeights && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/build: tutorial/403_BoundedBiharmonicWeights_bin

.PHONY : tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/build

tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/clean:
	cd /usr/include/libigl/build/tutorial/403_BoundedBiharmonicWeights && $(CMAKE_COMMAND) -P CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/cmake_clean.cmake
.PHONY : tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/clean

tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/depend:
	cd /usr/include/libigl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/include/libigl /usr/include/libigl/tutorial/403_BoundedBiharmonicWeights /usr/include/libigl/build /usr/include/libigl/build/tutorial/403_BoundedBiharmonicWeights /usr/include/libigl/build/tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tutorial/403_BoundedBiharmonicWeights/CMakeFiles/403_BoundedBiharmonicWeights_bin.dir/depend

