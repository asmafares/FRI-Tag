# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/fri/catkin_ws_bob/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fri/catkin_ws_bob/build

# Include any dependencies generated for this target.
include kinect_demo/CMakeFiles/detect_maybe.dir/depend.make

# Include the progress variables for this target.
include kinect_demo/CMakeFiles/detect_maybe.dir/progress.make

# Include the compile flags for this target's objects.
include kinect_demo/CMakeFiles/detect_maybe.dir/flags.make

kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o: kinect_demo/CMakeFiles/detect_maybe.dir/flags.make
kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o: /home/fri/catkin_ws_bob/src/kinect_demo/src/detect_maybe.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/fri/catkin_ws_bob/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o"
	cd /home/fri/catkin_ws_bob/build/kinect_demo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o -c /home/fri/catkin_ws_bob/src/kinect_demo/src/detect_maybe.cpp

kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.i"
	cd /home/fri/catkin_ws_bob/build/kinect_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/fri/catkin_ws_bob/src/kinect_demo/src/detect_maybe.cpp > CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.i

kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.s"
	cd /home/fri/catkin_ws_bob/build/kinect_demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/fri/catkin_ws_bob/src/kinect_demo/src/detect_maybe.cpp -o CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.s

kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o.requires:
.PHONY : kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o.requires

kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o.provides: kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o.requires
	$(MAKE) -f kinect_demo/CMakeFiles/detect_maybe.dir/build.make kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o.provides.build
.PHONY : kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o.provides

kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o.provides.build: kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o

# Object files for target detect_maybe
detect_maybe_OBJECTS = \
"CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o"

# External object files for target detect_maybe
detect_maybe_EXTERNAL_OBJECTS =

/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libpcl_ros_filters.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libpcl_ros_io.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libpcl_ros_tf.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_common.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_kdtree.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_octree.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_search.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_io.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_sample_consensus.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_filters.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_visualization.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_outofcore.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_features.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_segmentation.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_people.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_registration.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_recognition.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_keypoints.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_surface.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_tracking.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libpcl_apps.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libboost_iostreams-mt.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libboost_serialization-mt.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libqhull.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libOpenNI.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libflann_cpp_s.a
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libvtkCommon.so.5.8.0
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libvtkRendering.so.5.8.0
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libvtkHybrid.so.5.8.0
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libvtkCharts.so.5.8.0
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libdynamic_reconfigure_config_init_mutex.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libnodeletlib.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libbondcpp.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libtinyxml.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libclass_loader.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libPocoFoundation.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/x86_64-linux-gnu/libdl.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libroslib.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/librosbag.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/librosbag_storage.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libboost_program_options-mt.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libtopic_tools.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libtf.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libtf2_ros.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libactionlib.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libmessage_filters.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libtf2.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libroscpp.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libboost_signals-mt.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libboost_filesystem-mt.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/librosconsole.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/librosconsole_log4cxx.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/librosconsole_backend_interface.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/liblog4cxx.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libboost_regex-mt.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libroscpp_serialization.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/librostime.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libboost_date_time-mt.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libboost_system-mt.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/libboost_thread-mt.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libxmlrpcpp.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libcpp_common.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: /opt/ros/hydro/lib/libconsole_bridge.so
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: kinect_demo/CMakeFiles/detect_maybe.dir/build.make
/home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe: kinect_demo/CMakeFiles/detect_maybe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe"
	cd /home/fri/catkin_ws_bob/build/kinect_demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/detect_maybe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
kinect_demo/CMakeFiles/detect_maybe.dir/build: /home/fri/catkin_ws_bob/devel/lib/kinect_demo/detect_maybe
.PHONY : kinect_demo/CMakeFiles/detect_maybe.dir/build

kinect_demo/CMakeFiles/detect_maybe.dir/requires: kinect_demo/CMakeFiles/detect_maybe.dir/src/detect_maybe.cpp.o.requires
.PHONY : kinect_demo/CMakeFiles/detect_maybe.dir/requires

kinect_demo/CMakeFiles/detect_maybe.dir/clean:
	cd /home/fri/catkin_ws_bob/build/kinect_demo && $(CMAKE_COMMAND) -P CMakeFiles/detect_maybe.dir/cmake_clean.cmake
.PHONY : kinect_demo/CMakeFiles/detect_maybe.dir/clean

kinect_demo/CMakeFiles/detect_maybe.dir/depend:
	cd /home/fri/catkin_ws_bob/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fri/catkin_ws_bob/src /home/fri/catkin_ws_bob/src/kinect_demo /home/fri/catkin_ws_bob/build /home/fri/catkin_ws_bob/build/kinect_demo /home/fri/catkin_ws_bob/build/kinect_demo/CMakeFiles/detect_maybe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : kinect_demo/CMakeFiles/detect_maybe.dir/depend

