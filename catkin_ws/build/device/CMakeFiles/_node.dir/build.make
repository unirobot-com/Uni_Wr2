# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/uni/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/uni/catkin_ws/build

# Include any dependencies generated for this target.
include device/CMakeFiles/_node.dir/depend.make

# Include the progress variables for this target.
include device/CMakeFiles/_node.dir/progress.make

# Include the compile flags for this target's objects.
include device/CMakeFiles/_node.dir/flags.make

device/CMakeFiles/_node.dir/src/motor.cpp.o: device/CMakeFiles/_node.dir/flags.make
device/CMakeFiles/_node.dir/src/motor.cpp.o: /home/uni/catkin_ws/src/device/src/motor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/uni/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object device/CMakeFiles/_node.dir/src/motor.cpp.o"
	cd /home/uni/catkin_ws/build/device && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/_node.dir/src/motor.cpp.o -c /home/uni/catkin_ws/src/device/src/motor.cpp

device/CMakeFiles/_node.dir/src/motor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/_node.dir/src/motor.cpp.i"
	cd /home/uni/catkin_ws/build/device && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/uni/catkin_ws/src/device/src/motor.cpp > CMakeFiles/_node.dir/src/motor.cpp.i

device/CMakeFiles/_node.dir/src/motor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/_node.dir/src/motor.cpp.s"
	cd /home/uni/catkin_ws/build/device && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/uni/catkin_ws/src/device/src/motor.cpp -o CMakeFiles/_node.dir/src/motor.cpp.s

# Object files for target _node
_node_OBJECTS = \
"CMakeFiles/_node.dir/src/motor.cpp.o"

# External object files for target _node
_node_EXTERNAL_OBJECTS =

/home/uni/catkin_ws/devel/lib/device/_node: device/CMakeFiles/_node.dir/src/motor.cpp.o
/home/uni/catkin_ws/devel/lib/device/_node: device/CMakeFiles/_node.dir/build.make
/home/uni/catkin_ws/devel/lib/device/_node: /opt/ros/noetic/lib/libroscpp.so
/home/uni/catkin_ws/devel/lib/device/_node: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/uni/catkin_ws/devel/lib/device/_node: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so.1.71.0
/home/uni/catkin_ws/devel/lib/device/_node: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so.1.71.0
/home/uni/catkin_ws/devel/lib/device/_node: /opt/ros/noetic/lib/librosconsole.so
/home/uni/catkin_ws/devel/lib/device/_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/uni/catkin_ws/devel/lib/device/_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/uni/catkin_ws/devel/lib/device/_node: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/uni/catkin_ws/devel/lib/device/_node: /usr/lib/arm-linux-gnueabihf/libboost_regex.so.1.71.0
/home/uni/catkin_ws/devel/lib/device/_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/uni/catkin_ws/devel/lib/device/_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/uni/catkin_ws/devel/lib/device/_node: /opt/ros/noetic/lib/librostime.so
/home/uni/catkin_ws/devel/lib/device/_node: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so.1.71.0
/home/uni/catkin_ws/devel/lib/device/_node: /opt/ros/noetic/lib/libcpp_common.so
/home/uni/catkin_ws/devel/lib/device/_node: /usr/lib/arm-linux-gnueabihf/libboost_system.so.1.71.0
/home/uni/catkin_ws/devel/lib/device/_node: /usr/lib/arm-linux-gnueabihf/libboost_thread.so.1.71.0
/home/uni/catkin_ws/devel/lib/device/_node: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so.0.4
/home/uni/catkin_ws/devel/lib/device/_node: device/CMakeFiles/_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/uni/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/uni/catkin_ws/devel/lib/device/_node"
	cd /home/uni/catkin_ws/build/device && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
device/CMakeFiles/_node.dir/build: /home/uni/catkin_ws/devel/lib/device/_node

.PHONY : device/CMakeFiles/_node.dir/build

device/CMakeFiles/_node.dir/clean:
	cd /home/uni/catkin_ws/build/device && $(CMAKE_COMMAND) -P CMakeFiles/_node.dir/cmake_clean.cmake
.PHONY : device/CMakeFiles/_node.dir/clean

device/CMakeFiles/_node.dir/depend:
	cd /home/uni/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/uni/catkin_ws/src /home/uni/catkin_ws/src/device /home/uni/catkin_ws/build /home/uni/catkin_ws/build/device /home/uni/catkin_ws/build/device/CMakeFiles/_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : device/CMakeFiles/_node.dir/depend

