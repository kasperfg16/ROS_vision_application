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
CMAKE_SOURCE_DIR = /home/ubuntu/p5_project_group_364/src/Universal_Robots_Client_Library

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel

# Include any dependencies generated for this target.
include examples/CMakeFiles/primary_pipeline_example.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/primary_pipeline_example.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/primary_pipeline_example.dir/flags.make

examples/CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.o: examples/CMakeFiles/primary_pipeline_example.dir/flags.make
examples/CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.o: /home/ubuntu/p5_project_group_364/src/Universal_Robots_Client_Library/examples/primary_pipeline.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.o"
	cd /home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.o -c /home/ubuntu/p5_project_group_364/src/Universal_Robots_Client_Library/examples/primary_pipeline.cpp

examples/CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.i"
	cd /home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/p5_project_group_364/src/Universal_Robots_Client_Library/examples/primary_pipeline.cpp > CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.i

examples/CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.s"
	cd /home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/p5_project_group_364/src/Universal_Robots_Client_Library/examples/primary_pipeline.cpp -o CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.s

# Object files for target primary_pipeline_example
primary_pipeline_example_OBJECTS = \
"CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.o"

# External object files for target primary_pipeline_example
primary_pipeline_example_EXTERNAL_OBJECTS =

examples/primary_pipeline_example: examples/CMakeFiles/primary_pipeline_example.dir/primary_pipeline.cpp.o
examples/primary_pipeline_example: examples/CMakeFiles/primary_pipeline_example.dir/build.make
examples/primary_pipeline_example: liburcl.so
examples/primary_pipeline_example: examples/CMakeFiles/primary_pipeline_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable primary_pipeline_example"
	cd /home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/primary_pipeline_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/primary_pipeline_example.dir/build: examples/primary_pipeline_example

.PHONY : examples/CMakeFiles/primary_pipeline_example.dir/build

examples/CMakeFiles/primary_pipeline_example.dir/clean:
	cd /home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel/examples && $(CMAKE_COMMAND) -P CMakeFiles/primary_pipeline_example.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/primary_pipeline_example.dir/clean

examples/CMakeFiles/primary_pipeline_example.dir/depend:
	cd /home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/p5_project_group_364/src/Universal_Robots_Client_Library /home/ubuntu/p5_project_group_364/src/Universal_Robots_Client_Library/examples /home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel /home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel/examples /home/ubuntu/p5_project_group_364/build_isolated/ur_client_library/devel/examples/CMakeFiles/primary_pipeline_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/primary_pipeline_example.dir/depend

