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
include test/CMakeFiles/nomalloc_5.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/nomalloc_5.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/nomalloc_5.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/nomalloc_5.dir/flags.make

test/CMakeFiles/nomalloc_5.dir/nomalloc.cpp.o: test/CMakeFiles/nomalloc_5.dir/flags.make
test/CMakeFiles/nomalloc_5.dir/nomalloc.cpp.o: /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/nomalloc.cpp
test/CMakeFiles/nomalloc_5.dir/nomalloc.cpp.o: test/CMakeFiles/nomalloc_5.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/nomalloc_5.dir/nomalloc.cpp.o"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/nomalloc_5.dir/nomalloc.cpp.o -MF CMakeFiles/nomalloc_5.dir/nomalloc.cpp.o.d -o CMakeFiles/nomalloc_5.dir/nomalloc.cpp.o -c /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/nomalloc.cpp

test/CMakeFiles/nomalloc_5.dir/nomalloc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nomalloc_5.dir/nomalloc.cpp.i"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/nomalloc.cpp > CMakeFiles/nomalloc_5.dir/nomalloc.cpp.i

test/CMakeFiles/nomalloc_5.dir/nomalloc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nomalloc_5.dir/nomalloc.cpp.s"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/nomalloc.cpp -o CMakeFiles/nomalloc_5.dir/nomalloc.cpp.s

# Object files for target nomalloc_5
nomalloc_5_OBJECTS = \
"CMakeFiles/nomalloc_5.dir/nomalloc.cpp.o"

# External object files for target nomalloc_5
nomalloc_5_EXTERNAL_OBJECTS =

test/nomalloc_5: test/CMakeFiles/nomalloc_5.dir/nomalloc.cpp.o
test/nomalloc_5: test/CMakeFiles/nomalloc_5.dir/build.make
test/nomalloc_5: test/CMakeFiles/nomalloc_5.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable nomalloc_5"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nomalloc_5.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/nomalloc_5.dir/build: test/nomalloc_5
.PHONY : test/CMakeFiles/nomalloc_5.dir/build

test/CMakeFiles/nomalloc_5.dir/clean:
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && $(CMAKE_COMMAND) -P CMakeFiles/nomalloc_5.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/nomalloc_5.dir/clean

test/CMakeFiles/nomalloc_5.dir/depend:
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3 /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test/CMakeFiles/nomalloc_5.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/nomalloc_5.dir/depend

