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
include unsupported/test/CMakeFiles/FFT.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include unsupported/test/CMakeFiles/FFT.dir/compiler_depend.make

# Include the progress variables for this target.
include unsupported/test/CMakeFiles/FFT.dir/progress.make

# Include the compile flags for this target's objects.
include unsupported/test/CMakeFiles/FFT.dir/flags.make

unsupported/test/CMakeFiles/FFT.dir/FFT.cpp.o: unsupported/test/CMakeFiles/FFT.dir/flags.make
unsupported/test/CMakeFiles/FFT.dir/FFT.cpp.o: /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/unsupported/test/FFT.cpp
unsupported/test/CMakeFiles/FFT.dir/FFT.cpp.o: unsupported/test/CMakeFiles/FFT.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object unsupported/test/CMakeFiles/FFT.dir/FFT.cpp.o"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/unsupported/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT unsupported/test/CMakeFiles/FFT.dir/FFT.cpp.o -MF CMakeFiles/FFT.dir/FFT.cpp.o.d -o CMakeFiles/FFT.dir/FFT.cpp.o -c /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/unsupported/test/FFT.cpp

unsupported/test/CMakeFiles/FFT.dir/FFT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FFT.dir/FFT.cpp.i"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/unsupported/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/unsupported/test/FFT.cpp > CMakeFiles/FFT.dir/FFT.cpp.i

unsupported/test/CMakeFiles/FFT.dir/FFT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FFT.dir/FFT.cpp.s"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/unsupported/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/unsupported/test/FFT.cpp -o CMakeFiles/FFT.dir/FFT.cpp.s

# Object files for target FFT
FFT_OBJECTS = \
"CMakeFiles/FFT.dir/FFT.cpp.o"

# External object files for target FFT
FFT_EXTERNAL_OBJECTS =

unsupported/test/FFT: unsupported/test/CMakeFiles/FFT.dir/FFT.cpp.o
unsupported/test/FFT: unsupported/test/CMakeFiles/FFT.dir/build.make
unsupported/test/FFT: unsupported/test/CMakeFiles/FFT.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable FFT"
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/unsupported/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FFT.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
unsupported/test/CMakeFiles/FFT.dir/build: unsupported/test/FFT
.PHONY : unsupported/test/CMakeFiles/FFT.dir/build

unsupported/test/CMakeFiles/FFT.dir/clean:
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/unsupported/test && $(CMAKE_COMMAND) -P CMakeFiles/FFT.dir/cmake_clean.cmake
.PHONY : unsupported/test/CMakeFiles/FFT.dir/clean

unsupported/test/CMakeFiles/FFT.dir/depend:
	cd /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/eigen/unsupported/test /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3 /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/unsupported/test /home/denis/Desktop/GustGurus-Drone-Project/crazyflie_simulation/build/Eigen3/unsupported/test/CMakeFiles/FFT.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : unsupported/test/CMakeFiles/FFT.dir/depend

