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
CMAKE_SOURCE_DIR = /home/sw/homo_geo/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sw/homo_geo/build

# Include any dependencies generated for this target.
include control/uav_state/CMakeFiles/uav_state.dir/depend.make

# Include the progress variables for this target.
include control/uav_state/CMakeFiles/uav_state.dir/progress.make

# Include the compile flags for this target's objects.
include control/uav_state/CMakeFiles/uav_state.dir/flags.make

control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o: control/uav_state/CMakeFiles/uav_state.dir/flags.make
control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o: /home/sw/homo_geo/src/control/uav_state/src/UAVState.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sw/homo_geo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o"
	cd /home/sw/homo_geo/build/control/uav_state && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/uav_state.dir/src/UAVState.cpp.o -c /home/sw/homo_geo/src/control/uav_state/src/UAVState.cpp

control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uav_state.dir/src/UAVState.cpp.i"
	cd /home/sw/homo_geo/build/control/uav_state && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sw/homo_geo/src/control/uav_state/src/UAVState.cpp > CMakeFiles/uav_state.dir/src/UAVState.cpp.i

control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uav_state.dir/src/UAVState.cpp.s"
	cd /home/sw/homo_geo/build/control/uav_state && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sw/homo_geo/src/control/uav_state/src/UAVState.cpp -o CMakeFiles/uav_state.dir/src/UAVState.cpp.s

control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o.requires:

.PHONY : control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o.requires

control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o.provides: control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o.requires
	$(MAKE) -f control/uav_state/CMakeFiles/uav_state.dir/build.make control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o.provides.build
.PHONY : control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o.provides

control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o.provides.build: control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o


# Object files for target uav_state
uav_state_OBJECTS = \
"CMakeFiles/uav_state.dir/src/UAVState.cpp.o"

# External object files for target uav_state
uav_state_EXTERNAL_OBJECTS =

/home/sw/homo_geo/devel/lib/libuav_state.so: control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o
/home/sw/homo_geo/devel/lib/libuav_state.so: control/uav_state/CMakeFiles/uav_state.dir/build.make
/home/sw/homo_geo/devel/lib/libuav_state.so: /opt/ros/melodic/lib/libroscpp.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /opt/ros/melodic/lib/librosconsole.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /opt/ros/melodic/lib/librostime.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /opt/ros/melodic/lib/libcpp_common.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/sw/homo_geo/devel/lib/libuav_state.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/sw/homo_geo/devel/lib/libuav_state.so: control/uav_state/CMakeFiles/uav_state.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sw/homo_geo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/sw/homo_geo/devel/lib/libuav_state.so"
	cd /home/sw/homo_geo/build/control/uav_state && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/uav_state.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
control/uav_state/CMakeFiles/uav_state.dir/build: /home/sw/homo_geo/devel/lib/libuav_state.so

.PHONY : control/uav_state/CMakeFiles/uav_state.dir/build

control/uav_state/CMakeFiles/uav_state.dir/requires: control/uav_state/CMakeFiles/uav_state.dir/src/UAVState.cpp.o.requires

.PHONY : control/uav_state/CMakeFiles/uav_state.dir/requires

control/uav_state/CMakeFiles/uav_state.dir/clean:
	cd /home/sw/homo_geo/build/control/uav_state && $(CMAKE_COMMAND) -P CMakeFiles/uav_state.dir/cmake_clean.cmake
.PHONY : control/uav_state/CMakeFiles/uav_state.dir/clean

control/uav_state/CMakeFiles/uav_state.dir/depend:
	cd /home/sw/homo_geo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sw/homo_geo/src /home/sw/homo_geo/src/control/uav_state /home/sw/homo_geo/build /home/sw/homo_geo/build/control/uav_state /home/sw/homo_geo/build/control/uav_state/CMakeFiles/uav_state.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : control/uav_state/CMakeFiles/uav_state.dir/depend

