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
CMAKE_SOURCE_DIR = /home/bearli/competition_catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bearli/competition_catkin_ws/build

# Include any dependencies generated for this target.
include demopkg/CMakeFiles/boost_serial.dir/depend.make

# Include the progress variables for this target.
include demopkg/CMakeFiles/boost_serial.dir/progress.make

# Include the compile flags for this target's objects.
include demopkg/CMakeFiles/boost_serial.dir/flags.make

demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o: demopkg/CMakeFiles/boost_serial.dir/flags.make
demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o: /home/bearli/competition_catkin_ws/src/demopkg/src/boost_serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bearli/competition_catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o"
	cd /home/bearli/competition_catkin_ws/build/demopkg && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o -c /home/bearli/competition_catkin_ws/src/demopkg/src/boost_serial.cpp

demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/boost_serial.dir/src/boost_serial.cpp.i"
	cd /home/bearli/competition_catkin_ws/build/demopkg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bearli/competition_catkin_ws/src/demopkg/src/boost_serial.cpp > CMakeFiles/boost_serial.dir/src/boost_serial.cpp.i

demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/boost_serial.dir/src/boost_serial.cpp.s"
	cd /home/bearli/competition_catkin_ws/build/demopkg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bearli/competition_catkin_ws/src/demopkg/src/boost_serial.cpp -o CMakeFiles/boost_serial.dir/src/boost_serial.cpp.s

demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o.requires:

.PHONY : demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o.requires

demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o.provides: demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o.requires
	$(MAKE) -f demopkg/CMakeFiles/boost_serial.dir/build.make demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o.provides.build
.PHONY : demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o.provides

demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o.provides.build: demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o


# Object files for target boost_serial
boost_serial_OBJECTS = \
"CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o"

# External object files for target boost_serial
boost_serial_EXTERNAL_OBJECTS =

/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: demopkg/CMakeFiles/boost_serial.dir/build.make
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /home/bearli/competition_catkin_ws/devel/lib/libserial.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/librt.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /opt/ros/kinetic/lib/libroscpp.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /opt/ros/kinetic/lib/librosconsole.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /opt/ros/kinetic/lib/librostime.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /opt/ros/kinetic/lib/libcpp_common.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial: demopkg/CMakeFiles/boost_serial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bearli/competition_catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial"
	cd /home/bearli/competition_catkin_ws/build/demopkg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/boost_serial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
demopkg/CMakeFiles/boost_serial.dir/build: /home/bearli/competition_catkin_ws/devel/lib/demopkg/boost_serial

.PHONY : demopkg/CMakeFiles/boost_serial.dir/build

demopkg/CMakeFiles/boost_serial.dir/requires: demopkg/CMakeFiles/boost_serial.dir/src/boost_serial.cpp.o.requires

.PHONY : demopkg/CMakeFiles/boost_serial.dir/requires

demopkg/CMakeFiles/boost_serial.dir/clean:
	cd /home/bearli/competition_catkin_ws/build/demopkg && $(CMAKE_COMMAND) -P CMakeFiles/boost_serial.dir/cmake_clean.cmake
.PHONY : demopkg/CMakeFiles/boost_serial.dir/clean

demopkg/CMakeFiles/boost_serial.dir/depend:
	cd /home/bearli/competition_catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bearli/competition_catkin_ws/src /home/bearli/competition_catkin_ws/src/demopkg /home/bearli/competition_catkin_ws/build /home/bearli/competition_catkin_ws/build/demopkg /home/bearli/competition_catkin_ws/build/demopkg/CMakeFiles/boost_serial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : demopkg/CMakeFiles/boost_serial.dir/depend

