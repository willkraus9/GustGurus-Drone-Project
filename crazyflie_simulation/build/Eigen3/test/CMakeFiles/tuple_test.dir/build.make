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
include test/CMakeFiles/tuple_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/tuple_test.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/tuple_test.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/tuple_test.dir/flags.make

test/CMakeFiles/tuple_test.dir/tuple_test.cpp.o: test/CMakeFiles/tuple_test.dir/flags.make
test/CMakeFiles/tuple_test.dir/tuple_test.cpp.o: /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/tuple_test.cpp
test/CMakeFiles/tuple_test.dir/tuple_test.cpp.o: test/CMakeFiles/tuple_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/tuple_test.dir/tuple_test.cpp.o"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/tuple_test.dir/tuple_test.cpp.o -MF CMakeFiles/tuple_test.dir/tuple_test.cpp.o.d -o CMakeFiles/tuple_test.dir/tuple_test.cpp.o -c /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/tuple_test.cpp

test/CMakeFiles/tuple_test.dir/tuple_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tuple_test.dir/tuple_test.cpp.i"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/tuple_test.cpp > CMakeFiles/tuple_test.dir/tuple_test.cpp.i

test/CMakeFiles/tuple_test.dir/tuple_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tuple_test.dir/tuple_test.cpp.s"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/tuple_test.cpp -o CMakeFiles/tuple_test.dir/tuple_test.cpp.s

# Object files for target tuple_test
tuple_test_OBJECTS = \
"CMakeFiles/tuple_test.dir/tuple_test.cpp.o"

# External object files for target tuple_test
tuple_test_EXTERNAL_OBJECTS =

test/tuple_test: test/CMakeFiles/tuple_test.dir/tuple_test.cpp.o
test/tuple_test: test/CMakeFiles/tuple_test.dir/build.make
test/tuple_test: test/CMakeFiles/tuple_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable tuple_test"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tuple_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/tuple_test.dir/build: test/tuple_test
.PHONY : test/CMakeFiles/tuple_test.dir/build

test/CMakeFiles/tuple_test.dir/clean:
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && $(CMAKE_COMMAND) -P CMakeFiles/tuple_test.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/tuple_test.dir/clean

test/CMakeFiles/tuple_test.dir/depend:
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3 /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test/CMakeFiles/tuple_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/tuple_test.dir/depend

