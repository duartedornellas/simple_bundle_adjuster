# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset

# Include any dependencies generated for this target.
include CMakeFiles/generate_bal_dataset.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/generate_bal_dataset.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/generate_bal_dataset.dir/flags.make

CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o: CMakeFiles/generate_bal_dataset.dir/flags.make
CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o: generate_bal_dataset.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o -c /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset/generate_bal_dataset.cpp

CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset/generate_bal_dataset.cpp > CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.i

CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset/generate_bal_dataset.cpp -o CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.s

CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o.requires:
.PHONY : CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o.requires

CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o.provides: CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o.requires
	$(MAKE) -f CMakeFiles/generate_bal_dataset.dir/build.make CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o.provides.build
.PHONY : CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o.provides

CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o.provides.build: CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o

# Object files for target generate_bal_dataset
generate_bal_dataset_OBJECTS = \
"CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o"

# External object files for target generate_bal_dataset
generate_bal_dataset_EXTERNAL_OBJECTS =

generate_bal_dataset: CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o
generate_bal_dataset: CMakeFiles/generate_bal_dataset.dir/build.make
generate_bal_dataset: CMakeFiles/generate_bal_dataset.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable generate_bal_dataset"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/generate_bal_dataset.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/generate_bal_dataset.dir/build: generate_bal_dataset
.PHONY : CMakeFiles/generate_bal_dataset.dir/build

CMakeFiles/generate_bal_dataset.dir/requires: CMakeFiles/generate_bal_dataset.dir/generate_bal_dataset.cpp.o.requires
.PHONY : CMakeFiles/generate_bal_dataset.dir/requires

CMakeFiles/generate_bal_dataset.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/generate_bal_dataset.dir/cmake_clean.cmake
.PHONY : CMakeFiles/generate_bal_dataset.dir/clean

CMakeFiles/generate_bal_dataset.dir/depend:
	cd /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset /home/ddornellas/Desktop/Programming/test_ceres/simple_bundle_adjuster/generate_bal_dataset/CMakeFiles/generate_bal_dataset.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/generate_bal_dataset.dir/depend

