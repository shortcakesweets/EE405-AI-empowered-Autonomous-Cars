# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/team2/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/team2/catkin_ws/build

# Include any dependencies generated for this target.
include urg_c/CMakeFiles/open_urg_sensor.dir/depend.make

# Include the progress variables for this target.
include urg_c/CMakeFiles/open_urg_sensor.dir/progress.make

# Include the compile flags for this target's objects.
include urg_c/CMakeFiles/open_urg_sensor.dir/flags.make

urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o: urg_c/CMakeFiles/open_urg_sensor.dir/flags.make
urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o: /home/team2/catkin_ws/src/urg_c/current/samples/open_urg_sensor.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/team2/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o"
	cd /home/team2/catkin_ws/build/urg_c && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o   -c /home/team2/catkin_ws/src/urg_c/current/samples/open_urg_sensor.c

urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.i"
	cd /home/team2/catkin_ws/build/urg_c && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/team2/catkin_ws/src/urg_c/current/samples/open_urg_sensor.c > CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.i

urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.s"
	cd /home/team2/catkin_ws/build/urg_c && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/team2/catkin_ws/src/urg_c/current/samples/open_urg_sensor.c -o CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.s

urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o.requires:

.PHONY : urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o.requires

urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o.provides: urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o.requires
	$(MAKE) -f urg_c/CMakeFiles/open_urg_sensor.dir/build.make urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o.provides.build
.PHONY : urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o.provides

urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o.provides.build: urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o


# Object files for target open_urg_sensor
open_urg_sensor_OBJECTS = \
"CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o"

# External object files for target open_urg_sensor
open_urg_sensor_EXTERNAL_OBJECTS =

/home/team2/catkin_ws/devel/lib/libopen_urg_sensor.so: urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o
/home/team2/catkin_ws/devel/lib/libopen_urg_sensor.so: urg_c/CMakeFiles/open_urg_sensor.dir/build.make
/home/team2/catkin_ws/devel/lib/libopen_urg_sensor.so: /home/team2/catkin_ws/devel/lib/libliburg_c.so
/home/team2/catkin_ws/devel/lib/libopen_urg_sensor.so: urg_c/CMakeFiles/open_urg_sensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/team2/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared library /home/team2/catkin_ws/devel/lib/libopen_urg_sensor.so"
	cd /home/team2/catkin_ws/build/urg_c && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/open_urg_sensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
urg_c/CMakeFiles/open_urg_sensor.dir/build: /home/team2/catkin_ws/devel/lib/libopen_urg_sensor.so

.PHONY : urg_c/CMakeFiles/open_urg_sensor.dir/build

urg_c/CMakeFiles/open_urg_sensor.dir/requires: urg_c/CMakeFiles/open_urg_sensor.dir/current/samples/open_urg_sensor.c.o.requires

.PHONY : urg_c/CMakeFiles/open_urg_sensor.dir/requires

urg_c/CMakeFiles/open_urg_sensor.dir/clean:
	cd /home/team2/catkin_ws/build/urg_c && $(CMAKE_COMMAND) -P CMakeFiles/open_urg_sensor.dir/cmake_clean.cmake
.PHONY : urg_c/CMakeFiles/open_urg_sensor.dir/clean

urg_c/CMakeFiles/open_urg_sensor.dir/depend:
	cd /home/team2/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/team2/catkin_ws/src /home/team2/catkin_ws/src/urg_c /home/team2/catkin_ws/build /home/team2/catkin_ws/build/urg_c /home/team2/catkin_ws/build/urg_c/CMakeFiles/open_urg_sensor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : urg_c/CMakeFiles/open_urg_sensor.dir/depend

