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
include homography_geometric_control/CMakeFiles/homo_geo.dir/depend.make

# Include the progress variables for this target.
include homography_geometric_control/CMakeFiles/homo_geo.dir/progress.make

# Include the compile flags for this target's objects.
include homography_geometric_control/CMakeFiles/homo_geo.dir/flags.make

homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o: homography_geometric_control/CMakeFiles/homo_geo.dir/flags.make
homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o: /home/sw/homo_geo/src/homography_geometric_control/test/homo_geo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sw/homo_geo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o"
	cd /home/sw/homo_geo/build/homography_geometric_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o -c /home/sw/homo_geo/src/homography_geometric_control/test/homo_geo.cpp

homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/homo_geo.dir/test/homo_geo.cpp.i"
	cd /home/sw/homo_geo/build/homography_geometric_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sw/homo_geo/src/homography_geometric_control/test/homo_geo.cpp > CMakeFiles/homo_geo.dir/test/homo_geo.cpp.i

homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/homo_geo.dir/test/homo_geo.cpp.s"
	cd /home/sw/homo_geo/build/homography_geometric_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sw/homo_geo/src/homography_geometric_control/test/homo_geo.cpp -o CMakeFiles/homo_geo.dir/test/homo_geo.cpp.s

homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o.requires:

.PHONY : homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o.requires

homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o.provides: homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o.requires
	$(MAKE) -f homography_geometric_control/CMakeFiles/homo_geo.dir/build.make homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o.provides.build
.PHONY : homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o.provides

homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o.provides.build: homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o


# Object files for target homo_geo
homo_geo_OBJECTS = \
"CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o"

# External object files for target homo_geo
homo_geo_EXTERNAL_OBJECTS =

/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: homography_geometric_control/CMakeFiles/homo_geo.dir/build.make
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/libroscpp.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/librosconsole.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /home/sw/homo_geo/devel/lib/libmavros_interaction.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /home/sw/homo_geo/devel/lib/libuav_state.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/librostime.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/libcpp_common.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /home/sw/homo_geo/devel/lib/libhomography_geometric_control.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/libroscpp.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/librosconsole.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/librostime.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /opt/ros/melodic/lib/libcpp_common.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo: homography_geometric_control/CMakeFiles/homo_geo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sw/homo_geo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo"
	cd /home/sw/homo_geo/build/homography_geometric_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/homo_geo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
homography_geometric_control/CMakeFiles/homo_geo.dir/build: /home/sw/homo_geo/devel/lib/homography_geometric_control/homo_geo

.PHONY : homography_geometric_control/CMakeFiles/homo_geo.dir/build

homography_geometric_control/CMakeFiles/homo_geo.dir/requires: homography_geometric_control/CMakeFiles/homo_geo.dir/test/homo_geo.cpp.o.requires

.PHONY : homography_geometric_control/CMakeFiles/homo_geo.dir/requires

homography_geometric_control/CMakeFiles/homo_geo.dir/clean:
	cd /home/sw/homo_geo/build/homography_geometric_control && $(CMAKE_COMMAND) -P CMakeFiles/homo_geo.dir/cmake_clean.cmake
.PHONY : homography_geometric_control/CMakeFiles/homo_geo.dir/clean

homography_geometric_control/CMakeFiles/homo_geo.dir/depend:
	cd /home/sw/homo_geo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sw/homo_geo/src /home/sw/homo_geo/src/homography_geometric_control /home/sw/homo_geo/build /home/sw/homo_geo/build/homography_geometric_control /home/sw/homo_geo/build/homography_geometric_control/CMakeFiles/homo_geo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : homography_geometric_control/CMakeFiles/homo_geo.dir/depend

