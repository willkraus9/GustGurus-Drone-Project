# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3

# Include any dependencies generated for this target.
include test/CMakeFiles/mapped_matrix_1.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/mapped_matrix_1.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/mapped_matrix_1.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/mapped_matrix_1.dir/flags.make

test/CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.o: test/CMakeFiles/mapped_matrix_1.dir/flags.make
test/CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.o: /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/mapped_matrix.cpp
test/CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.o: test/CMakeFiles/mapped_matrix_1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.o"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.o -MF CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.o.d -o CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.o -c /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/mapped_matrix.cpp

test/CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.i"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/mapped_matrix.cpp > CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.i

test/CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.s"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/mapped_matrix.cpp -o CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.s

# Object files for target mapped_matrix_1
mapped_matrix_1_OBJECTS = \
"CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.o"

# External object files for target mapped_matrix_1
mapped_matrix_1_EXTERNAL_OBJECTS =

test/mapped_matrix_1: test/CMakeFiles/mapped_matrix_1.dir/mapped_matrix.cpp.o
test/mapped_matrix_1: test/CMakeFiles/mapped_matrix_1.dir/build.make
test/mapped_matrix_1: test/CMakeFiles/mapped_matrix_1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mapped_matrix_1"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mapped_matrix_1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/mapped_matrix_1.dir/build: test/mapped_matrix_1
.PHONY : test/CMakeFiles/mapped_matrix_1.dir/build

test/CMakeFiles/mapped_matrix_1.dir/clean:
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && $(CMAKE_COMMAND) -P CMakeFiles/mapped_matrix_1.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/mapped_matrix_1.dir/clean

test/CMakeFiles/mapped_matrix_1.dir/depend:
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3 /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test/CMakeFiles/mapped_matrix_1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/mapped_matrix_1.dir/depend

