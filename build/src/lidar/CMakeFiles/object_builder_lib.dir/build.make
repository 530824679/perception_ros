# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/linuxidc/perception_ros

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/linuxidc/perception_ros/build

# Include any dependencies generated for this target.
include src/lidar/CMakeFiles/object_builder_lib.dir/depend.make

# Include the progress variables for this target.
include src/lidar/CMakeFiles/object_builder_lib.dir/progress.make

# Include the compile flags for this target's objects.
include src/lidar/CMakeFiles/object_builder_lib.dir/flags.make

src/lidar/CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.o: src/lidar/CMakeFiles/object_builder_lib.dir/flags.make
src/lidar/CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.o: ../src/lidar/object_builder/bbox_fitting.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/linuxidc/perception_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lidar/CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.o"
	cd /home/linuxidc/perception_ros/build/src/lidar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.o -c /home/linuxidc/perception_ros/src/lidar/object_builder/bbox_fitting.cpp

src/lidar/CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.i"
	cd /home/linuxidc/perception_ros/build/src/lidar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/linuxidc/perception_ros/src/lidar/object_builder/bbox_fitting.cpp > CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.i

src/lidar/CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.s"
	cd /home/linuxidc/perception_ros/build/src/lidar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/linuxidc/perception_ros/src/lidar/object_builder/bbox_fitting.cpp -o CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.s

# Object files for target object_builder_lib
object_builder_lib_OBJECTS = \
"CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.o"

# External object files for target object_builder_lib
object_builder_lib_EXTERNAL_OBJECTS =

devel/lib/libobject_builder_lib.a: src/lidar/CMakeFiles/object_builder_lib.dir/object_builder/bbox_fitting.cpp.o
devel/lib/libobject_builder_lib.a: src/lidar/CMakeFiles/object_builder_lib.dir/build.make
devel/lib/libobject_builder_lib.a: src/lidar/CMakeFiles/object_builder_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/linuxidc/perception_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../devel/lib/libobject_builder_lib.a"
	cd /home/linuxidc/perception_ros/build/src/lidar && $(CMAKE_COMMAND) -P CMakeFiles/object_builder_lib.dir/cmake_clean_target.cmake
	cd /home/linuxidc/perception_ros/build/src/lidar && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/object_builder_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lidar/CMakeFiles/object_builder_lib.dir/build: devel/lib/libobject_builder_lib.a

.PHONY : src/lidar/CMakeFiles/object_builder_lib.dir/build

src/lidar/CMakeFiles/object_builder_lib.dir/clean:
	cd /home/linuxidc/perception_ros/build/src/lidar && $(CMAKE_COMMAND) -P CMakeFiles/object_builder_lib.dir/cmake_clean.cmake
.PHONY : src/lidar/CMakeFiles/object_builder_lib.dir/clean

src/lidar/CMakeFiles/object_builder_lib.dir/depend:
	cd /home/linuxidc/perception_ros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/linuxidc/perception_ros /home/linuxidc/perception_ros/src/lidar /home/linuxidc/perception_ros/build /home/linuxidc/perception_ros/build/src/lidar /home/linuxidc/perception_ros/build/src/lidar/CMakeFiles/object_builder_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lidar/CMakeFiles/object_builder_lib.dir/depend

