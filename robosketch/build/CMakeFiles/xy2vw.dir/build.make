# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/nb950/Desktop/robosketch/robosketch

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nb950/Desktop/robosketch/robosketch/build

# Include any dependencies generated for this target.
include CMakeFiles/xy2vw.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/xy2vw.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/xy2vw.dir/flags.make

CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: CMakeFiles/xy2vw.dir/flags.make
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: ../src/xy2vw.cpp
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: ../manifest.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/cpp_common/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/rostime/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/roscpp_traits/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/roscpp_serialization/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/catkin/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/genmsg/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/genpy/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/message_runtime/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/std_msgs/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/gencpp/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/geneus/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/gennodejs/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/genlisp/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/message_generation/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/rosbuild/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/rosconsole/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/rosgraph_msgs/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/xmlrpcpp/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/roscpp/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/rosgraph/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/rospack/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/roslib/package.xml
CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o: /opt/ros/kinetic/share/rospy/package.xml
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nb950/Desktop/robosketch/robosketch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o -c /home/nb950/Desktop/robosketch/robosketch/src/xy2vw.cpp

CMakeFiles/xy2vw.dir/src/xy2vw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xy2vw.dir/src/xy2vw.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nb950/Desktop/robosketch/robosketch/src/xy2vw.cpp > CMakeFiles/xy2vw.dir/src/xy2vw.cpp.i

CMakeFiles/xy2vw.dir/src/xy2vw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xy2vw.dir/src/xy2vw.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nb950/Desktop/robosketch/robosketch/src/xy2vw.cpp -o CMakeFiles/xy2vw.dir/src/xy2vw.cpp.s

CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o.requires:

.PHONY : CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o.requires

CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o.provides: CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o.requires
	$(MAKE) -f CMakeFiles/xy2vw.dir/build.make CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o.provides.build
.PHONY : CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o.provides

CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o.provides.build: CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o


# Object files for target xy2vw
xy2vw_OBJECTS = \
"CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o"

# External object files for target xy2vw
xy2vw_EXTERNAL_OBJECTS =

../bin/xy2vw: CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o
../bin/xy2vw: CMakeFiles/xy2vw.dir/build.make
../bin/xy2vw: CMakeFiles/xy2vw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nb950/Desktop/robosketch/robosketch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/xy2vw"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xy2vw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/xy2vw.dir/build: ../bin/xy2vw

.PHONY : CMakeFiles/xy2vw.dir/build

CMakeFiles/xy2vw.dir/requires: CMakeFiles/xy2vw.dir/src/xy2vw.cpp.o.requires

.PHONY : CMakeFiles/xy2vw.dir/requires

CMakeFiles/xy2vw.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/xy2vw.dir/cmake_clean.cmake
.PHONY : CMakeFiles/xy2vw.dir/clean

CMakeFiles/xy2vw.dir/depend:
	cd /home/nb950/Desktop/robosketch/robosketch/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nb950/Desktop/robosketch/robosketch /home/nb950/Desktop/robosketch/robosketch /home/nb950/Desktop/robosketch/robosketch/build /home/nb950/Desktop/robosketch/robosketch/build /home/nb950/Desktop/robosketch/robosketch/build/CMakeFiles/xy2vw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/xy2vw.dir/depend
