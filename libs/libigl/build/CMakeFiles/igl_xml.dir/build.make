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
include CMakeFiles/igl_xml.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/igl_xml.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/igl_xml.dir/flags.make

CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.o: CMakeFiles/igl_xml.dir/flags.make
CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.o: ../include/igl/xml/serialize_xml.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/usr/include/libigl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.o -c /usr/include/libigl/include/igl/xml/serialize_xml.cpp

CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/include/libigl/include/igl/xml/serialize_xml.cpp > CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.i

CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/include/libigl/include/igl/xml/serialize_xml.cpp -o CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.s

CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.o: CMakeFiles/igl_xml.dir/flags.make
CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.o: ../include/igl/xml/writeDAE.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/usr/include/libigl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.o -c /usr/include/libigl/include/igl/xml/writeDAE.cpp

CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/include/libigl/include/igl/xml/writeDAE.cpp > CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.i

CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/include/libigl/include/igl/xml/writeDAE.cpp -o CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.s

CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.o: CMakeFiles/igl_xml.dir/flags.make
CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.o: ../include/igl/xml/write_triangle_mesh.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/usr/include/libigl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.o -c /usr/include/libigl/include/igl/xml/write_triangle_mesh.cpp

CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/include/libigl/include/igl/xml/write_triangle_mesh.cpp > CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.i

CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/include/libigl/include/igl/xml/write_triangle_mesh.cpp -o CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.s

# Object files for target igl_xml
igl_xml_OBJECTS = \
"CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.o" \
"CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.o" \
"CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.o"

# External object files for target igl_xml
igl_xml_EXTERNAL_OBJECTS =

libigl_xml.a: CMakeFiles/igl_xml.dir/include/igl/xml/serialize_xml.cpp.o
libigl_xml.a: CMakeFiles/igl_xml.dir/include/igl/xml/writeDAE.cpp.o
libigl_xml.a: CMakeFiles/igl_xml.dir/include/igl/xml/write_triangle_mesh.cpp.o
libigl_xml.a: CMakeFiles/igl_xml.dir/build.make
libigl_xml.a: CMakeFiles/igl_xml.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/usr/include/libigl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libigl_xml.a"
	$(CMAKE_COMMAND) -P CMakeFiles/igl_xml.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/igl_xml.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/igl_xml.dir/build: libigl_xml.a

.PHONY : CMakeFiles/igl_xml.dir/build

CMakeFiles/igl_xml.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/igl_xml.dir/cmake_clean.cmake
.PHONY : CMakeFiles/igl_xml.dir/clean

CMakeFiles/igl_xml.dir/depend:
	cd /usr/include/libigl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/include/libigl /usr/include/libigl /usr/include/libigl/build /usr/include/libigl/build /usr/include/libigl/build/CMakeFiles/igl_xml.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/igl_xml.dir/depend

