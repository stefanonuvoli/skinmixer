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
CMAKE_SOURCE_DIR = /usr/include/libigl/external/.cache/tutorial_data

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /usr/include/libigl/external/.cache/tutorial_data

# Utility rule file for tutorial_data-download.

# Include the progress variables for this target.
include CMakeFiles/tutorial_data-download.dir/progress.make

CMakeFiles/tutorial_data-download: CMakeFiles/tutorial_data-download-complete


CMakeFiles/tutorial_data-download-complete: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-install
CMakeFiles/tutorial_data-download-complete: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-mkdir
CMakeFiles/tutorial_data-download-complete: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-download
CMakeFiles/tutorial_data-download-complete: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-update
CMakeFiles/tutorial_data-download-complete: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-patch
CMakeFiles/tutorial_data-download-complete: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-configure
CMakeFiles/tutorial_data-download-complete: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-build
CMakeFiles/tutorial_data-download-complete: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-install
CMakeFiles/tutorial_data-download-complete: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/usr/include/libigl/external/.cache/tutorial_data/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'tutorial_data-download'"
	/usr/bin/cmake -E make_directory /usr/include/libigl/external/.cache/tutorial_data/CMakeFiles
	/usr/bin/cmake -E touch /usr/include/libigl/external/.cache/tutorial_data/CMakeFiles/tutorial_data-download-complete
	/usr/bin/cmake -E touch /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-done

tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-install: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/usr/include/libigl/external/.cache/tutorial_data/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No install step for 'tutorial_data-download'"
	cd /usr/include/libigl/build/tutorial_data-build && /usr/bin/cmake -E echo_append
	cd /usr/include/libigl/build/tutorial_data-build && /usr/bin/cmake -E touch /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-install

tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/usr/include/libigl/external/.cache/tutorial_data/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'tutorial_data-download'"
	/usr/bin/cmake -E make_directory /usr/include/libigl/cmake/../external/../tutorial/data
	/usr/bin/cmake -E make_directory /usr/include/libigl/build/tutorial_data-build
	/usr/bin/cmake -E make_directory /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix
	/usr/bin/cmake -E make_directory /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/tmp
	/usr/bin/cmake -E make_directory /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp
	/usr/bin/cmake -E make_directory /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src
	/usr/bin/cmake -E touch /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-mkdir

tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-download: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-gitinfo.txt
tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-download: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/usr/include/libigl/external/.cache/tutorial_data/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'tutorial_data-download'"
	cd /usr/include/libigl/tutorial && /usr/bin/cmake -P /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/tmp/tutorial_data-download-gitclone.cmake
	cd /usr/include/libigl/tutorial && /usr/bin/cmake -E touch /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-download

tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-update: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/usr/include/libigl/external/.cache/tutorial_data/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing update step for 'tutorial_data-download'"
	cd /usr/include/libigl/tutorial/data && /usr/bin/cmake -P /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/tmp/tutorial_data-download-gitupdate.cmake

tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-patch: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/usr/include/libigl/external/.cache/tutorial_data/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'tutorial_data-download'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-patch

tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-configure: tutorial_data-download-prefix/tmp/tutorial_data-download-cfgcmd.txt
tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-configure: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-update
tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-configure: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/usr/include/libigl/external/.cache/tutorial_data/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No configure step for 'tutorial_data-download'"
	cd /usr/include/libigl/build/tutorial_data-build && /usr/bin/cmake -E echo_append
	cd /usr/include/libigl/build/tutorial_data-build && /usr/bin/cmake -E touch /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-configure

tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-build: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/usr/include/libigl/external/.cache/tutorial_data/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No build step for 'tutorial_data-download'"
	cd /usr/include/libigl/build/tutorial_data-build && /usr/bin/cmake -E echo_append
	cd /usr/include/libigl/build/tutorial_data-build && /usr/bin/cmake -E touch /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-build

tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-test: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/usr/include/libigl/external/.cache/tutorial_data/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "No test step for 'tutorial_data-download'"
	cd /usr/include/libigl/build/tutorial_data-build && /usr/bin/cmake -E echo_append
	cd /usr/include/libigl/build/tutorial_data-build && /usr/bin/cmake -E touch /usr/include/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-test

tutorial_data-download: CMakeFiles/tutorial_data-download
tutorial_data-download: CMakeFiles/tutorial_data-download-complete
tutorial_data-download: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-install
tutorial_data-download: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-mkdir
tutorial_data-download: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-download
tutorial_data-download: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-update
tutorial_data-download: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-patch
tutorial_data-download: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-configure
tutorial_data-download: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-build
tutorial_data-download: tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-test
tutorial_data-download: CMakeFiles/tutorial_data-download.dir/build.make

.PHONY : tutorial_data-download

# Rule to build all files generated by this target.
CMakeFiles/tutorial_data-download.dir/build: tutorial_data-download

.PHONY : CMakeFiles/tutorial_data-download.dir/build

CMakeFiles/tutorial_data-download.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tutorial_data-download.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tutorial_data-download.dir/clean

CMakeFiles/tutorial_data-download.dir/depend:
	cd /usr/include/libigl/external/.cache/tutorial_data && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/include/libigl/external/.cache/tutorial_data /usr/include/libigl/external/.cache/tutorial_data /usr/include/libigl/external/.cache/tutorial_data /usr/include/libigl/external/.cache/tutorial_data /usr/include/libigl/external/.cache/tutorial_data/CMakeFiles/tutorial_data-download.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tutorial_data-download.dir/depend

