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

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/villa/PCL

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/villa/PCL/build

# Include any dependencies generated for this target.
include CMakeFiles/cloud_viewer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cloud_viewer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cloud_viewer.dir/flags.make

CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o: CMakeFiles/cloud_viewer.dir/flags.make
CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o: ../cloud_viewer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/villa/PCL/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o -c /home/villa/PCL/cloud_viewer.cpp

CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/villa/PCL/cloud_viewer.cpp > CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.i

CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/villa/PCL/cloud_viewer.cpp -o CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.s

CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o.requires:
.PHONY : CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o.requires

CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o.provides: CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/cloud_viewer.dir/build.make CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o.provides.build
.PHONY : CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o.provides

CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o.provides.build: CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o

# Object files for target cloud_viewer
cloud_viewer_OBJECTS = \
"CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o"

# External object files for target cloud_viewer
cloud_viewer_EXTERNAL_OBJECTS =

cloud_viewer: CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o
cloud_viewer: CMakeFiles/cloud_viewer.dir/build.make
cloud_viewer: /usr/lib/libboost_system-mt.so
cloud_viewer: /usr/lib/libboost_filesystem-mt.so
cloud_viewer: /usr/lib/libboost_thread-mt.so
cloud_viewer: /usr/lib/libboost_date_time-mt.so
cloud_viewer: /usr/lib/libboost_iostreams-mt.so
cloud_viewer: /usr/lib/libpcl_common.so
cloud_viewer: /usr/lib/libpcl_octree.so
cloud_viewer: /usr/lib/libOpenNI.so
cloud_viewer: /usr/lib/libvtkCommon.so.5.8.0
cloud_viewer: /usr/lib/libvtkRendering.so.5.8.0
cloud_viewer: /usr/lib/libvtkHybrid.so.5.8.0
cloud_viewer: /usr/lib/libpcl_io.so
cloud_viewer: /usr/lib/libflann_cpp_s.a
cloud_viewer: /usr/lib/libpcl_kdtree.so
cloud_viewer: /usr/lib/libpcl_search.so
cloud_viewer: /usr/lib/libpcl_sample_consensus.so
cloud_viewer: /usr/lib/libpcl_filters.so
cloud_viewer: /usr/lib/libpcl_segmentation.so
cloud_viewer: /usr/lib/libpcl_features.so
cloud_viewer: /usr/lib/libqhull.so
cloud_viewer: /usr/lib/libpcl_surface.so
cloud_viewer: /usr/lib/libpcl_registration.so
cloud_viewer: /usr/lib/libpcl_visualization.so
cloud_viewer: /usr/lib/libpcl_keypoints.so
cloud_viewer: /usr/lib/libpcl_tracking.so
cloud_viewer: /usr/lib/libpcl_apps.so
cloud_viewer: /usr/lib/libvtkParallel.so.5.8.0
cloud_viewer: /usr/lib/libvtkRendering.so.5.8.0
cloud_viewer: /usr/lib/libvtkGraphics.so.5.8.0
cloud_viewer: /usr/lib/libvtkImaging.so.5.8.0
cloud_viewer: /usr/lib/libvtkIO.so.5.8.0
cloud_viewer: /usr/lib/libvtkFiltering.so.5.8.0
cloud_viewer: /usr/lib/libvtkCommon.so.5.8.0
cloud_viewer: /usr/lib/libvtksys.so.5.8.0
cloud_viewer: CMakeFiles/cloud_viewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable cloud_viewer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cloud_viewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cloud_viewer.dir/build: cloud_viewer
.PHONY : CMakeFiles/cloud_viewer.dir/build

CMakeFiles/cloud_viewer.dir/requires: CMakeFiles/cloud_viewer.dir/cloud_viewer.cpp.o.requires
.PHONY : CMakeFiles/cloud_viewer.dir/requires

CMakeFiles/cloud_viewer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cloud_viewer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cloud_viewer.dir/clean

CMakeFiles/cloud_viewer.dir/depend:
	cd /home/villa/PCL/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/villa/PCL /home/villa/PCL /home/villa/PCL/build /home/villa/PCL/build /home/villa/PCL/build/CMakeFiles/cloud_viewer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cloud_viewer.dir/depend

