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
include CMakeFiles/igl_opengl_glfw_imgui.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/igl_opengl_glfw_imgui.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/igl_opengl_glfw_imgui.dir/flags.make

CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.o: CMakeFiles/igl_opengl_glfw_imgui.dir/flags.make
CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.o: ../include/igl/opengl/glfw/imgui/ImGuiMenu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/usr/include/libigl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.o -c /usr/include/libigl/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp

CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/include/libigl/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp > CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.i

CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/include/libigl/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp -o CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.s

# Object files for target igl_opengl_glfw_imgui
igl_opengl_glfw_imgui_OBJECTS = \
"CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.o"

# External object files for target igl_opengl_glfw_imgui
igl_opengl_glfw_imgui_EXTERNAL_OBJECTS =

libigl_opengl_glfw_imgui.a: CMakeFiles/igl_opengl_glfw_imgui.dir/include/igl/opengl/glfw/imgui/ImGuiMenu.cpp.o
libigl_opengl_glfw_imgui.a: CMakeFiles/igl_opengl_glfw_imgui.dir/build.make
libigl_opengl_glfw_imgui.a: CMakeFiles/igl_opengl_glfw_imgui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/usr/include/libigl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libigl_opengl_glfw_imgui.a"
	$(CMAKE_COMMAND) -P CMakeFiles/igl_opengl_glfw_imgui.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/igl_opengl_glfw_imgui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/igl_opengl_glfw_imgui.dir/build: libigl_opengl_glfw_imgui.a

.PHONY : CMakeFiles/igl_opengl_glfw_imgui.dir/build

CMakeFiles/igl_opengl_glfw_imgui.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/igl_opengl_glfw_imgui.dir/cmake_clean.cmake
.PHONY : CMakeFiles/igl_opengl_glfw_imgui.dir/clean

CMakeFiles/igl_opengl_glfw_imgui.dir/depend:
	cd /usr/include/libigl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/include/libigl /usr/include/libigl /usr/include/libigl/build /usr/include/libigl/build /usr/include/libigl/build/CMakeFiles/igl_opengl_glfw_imgui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/igl_opengl_glfw_imgui.dir/depend

