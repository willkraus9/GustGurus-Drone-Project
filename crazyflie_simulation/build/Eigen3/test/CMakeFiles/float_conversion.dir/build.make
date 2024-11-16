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
include test/CMakeFiles/float_conversion.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/CMakeFiles/float_conversion.dir/compiler_depend.make

# Include the progress variables for this target.
include test/CMakeFiles/float_conversion.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/float_conversion.dir/flags.make

test/CMakeFiles/float_conversion.dir/float_conversion.cpp.o: test/CMakeFiles/float_conversion.dir/flags.make
test/CMakeFiles/float_conversion.dir/float_conversion.cpp.o: /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/float_conversion.cpp
test/CMakeFiles/float_conversion.dir/float_conversion.cpp.o: test/CMakeFiles/float_conversion.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/float_conversion.dir/float_conversion.cpp.o"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/CMakeFiles/float_conversion.dir/float_conversion.cpp.o -MF CMakeFiles/float_conversion.dir/float_conversion.cpp.o.d -o CMakeFiles/float_conversion.dir/float_conversion.cpp.o -c /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/float_conversion.cpp

test/CMakeFiles/float_conversion.dir/float_conversion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/float_conversion.dir/float_conversion.cpp.i"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/float_conversion.cpp > CMakeFiles/float_conversion.dir/float_conversion.cpp.i

test/CMakeFiles/float_conversion.dir/float_conversion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/float_conversion.dir/float_conversion.cpp.s"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test/float_conversion.cpp -o CMakeFiles/float_conversion.dir/float_conversion.cpp.s

# Object files for target float_conversion
float_conversion_OBJECTS = \
"CMakeFiles/float_conversion.dir/float_conversion.cpp.o"

# External object files for target float_conversion
float_conversion_EXTERNAL_OBJECTS =

test/float_conversion: test/CMakeFiles/float_conversion.dir/float_conversion.cpp.o
test/float_conversion: test/CMakeFiles/float_conversion.dir/build.make
test/float_conversion: test/CMakeFiles/float_conversion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable float_conversion"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/float_conversion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/float_conversion.dir/build: test/float_conversion
.PHONY : test/CMakeFiles/float_conversion.dir/build

test/CMakeFiles/float_conversion.dir/clean:
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test && $(CMAKE_COMMAND) -P CMakeFiles/float_conversion.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/float_conversion.dir/clean

test/CMakeFiles/float_conversion.dir/depend:
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/test /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3 /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/test/CMakeFiles/float_conversion.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/float_conversion.dir/depend

