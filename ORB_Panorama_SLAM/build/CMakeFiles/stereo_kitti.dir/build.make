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
CMAKE_SOURCE_DIR = /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM/build

# Include any dependencies generated for this target.
include CMakeFiles/stereo_kitti.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_kitti.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_kitti.dir/flags.make

CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o: CMakeFiles/stereo_kitti.dir/flags.make
CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o: ../Examples/Stereo/stereo_kitti.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o -c /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM/Examples/Stereo/stereo_kitti.cpp

CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM/Examples/Stereo/stereo_kitti.cpp > CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.i

CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM/Examples/Stereo/stereo_kitti.cpp -o CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.s

CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o.requires:

.PHONY : CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o.requires

CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o.provides: CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o.requires
	$(MAKE) -f CMakeFiles/stereo_kitti.dir/build.make CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o.provides.build
.PHONY : CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o.provides

CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o.provides.build: CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o


# Object files for target stereo_kitti
stereo_kitti_OBJECTS = \
"CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o"

# External object files for target stereo_kitti
stereo_kitti_EXTERNAL_OBJECTS =

stereo_kitti: CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o
stereo_kitti: CMakeFiles/stereo_kitti.dir/build.make
stereo_kitti: ../lib/libORB_Panorama_SLAM.so
stereo_kitti: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_superres3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_face3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_plot3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_reg3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_viz3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_phase_unwrapping3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_text3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_shape3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_video3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_flann3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_ml3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_photo3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.2.0
stereo_kitti: /opt/ros/kinetic/lib/libopencv_core3.so.3.2.0
stereo_kitti: /home/ustc/nullmax/Pangolin/build/src/libpangolin.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libGLU.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libGL.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libGLEW.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libSM.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libICE.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libX11.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libXext.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libpython2.7.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libavcodec.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libavformat.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libavutil.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libswscale.so
stereo_kitti: /usr/lib/libOpenNI.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libpng.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libz.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libjpeg.so
stereo_kitti: /usr/lib/x86_64-linux-gnu/libtiff.so
stereo_kitti: ../Thirdparty/DBoW2/lib/libDBoW2.so
stereo_kitti: ../Thirdparty/g2o/lib/libg2o.so
stereo_kitti: CMakeFiles/stereo_kitti.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable stereo_kitti"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_kitti.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_kitti.dir/build: stereo_kitti

.PHONY : CMakeFiles/stereo_kitti.dir/build

CMakeFiles/stereo_kitti.dir/requires: CMakeFiles/stereo_kitti.dir/Examples/Stereo/stereo_kitti.cpp.o.requires

.PHONY : CMakeFiles/stereo_kitti.dir/requires

CMakeFiles/stereo_kitti.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_kitti.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_kitti.dir/clean

CMakeFiles/stereo_kitti.dir/depend:
	cd /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM/build /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM/build /home/ustc/ricoh_ws/src/panorama_slam/ORB_Panorama_SLAM/build/CMakeFiles/stereo_kitti.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_kitti.dir/depend

