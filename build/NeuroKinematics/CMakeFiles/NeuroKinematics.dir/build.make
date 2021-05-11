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
CMAKE_SOURCE_DIR = /home/kehan/neurobot_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kehan/neurobot_ws/build

# Include any dependencies generated for this target.
include NeuroKinematics/CMakeFiles/NeuroKinematics.dir/depend.make

# Include the progress variables for this target.
include NeuroKinematics/CMakeFiles/NeuroKinematics.dir/progress.make

# Include the compile flags for this target's objects.
include NeuroKinematics/CMakeFiles/NeuroKinematics.dir/flags.make

NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o: NeuroKinematics/CMakeFiles/NeuroKinematics.dir/flags.make
NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o: /home/kehan/neurobot_ws/src/NeuroKinematics/src/NeuroKinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kehan/neurobot_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o"
	cd /home/kehan/neurobot_ws/build/NeuroKinematics && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o -c /home/kehan/neurobot_ws/src/NeuroKinematics/src/NeuroKinematics.cpp

NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.i"
	cd /home/kehan/neurobot_ws/build/NeuroKinematics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kehan/neurobot_ws/src/NeuroKinematics/src/NeuroKinematics.cpp > CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.i

NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.s"
	cd /home/kehan/neurobot_ws/build/NeuroKinematics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kehan/neurobot_ws/src/NeuroKinematics/src/NeuroKinematics.cpp -o CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.s

NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o.requires:

.PHONY : NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o.requires

NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o.provides: NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o.requires
	$(MAKE) -f NeuroKinematics/CMakeFiles/NeuroKinematics.dir/build.make NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o.provides.build
.PHONY : NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o.provides

NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o.provides.build: NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o


# Object files for target NeuroKinematics
NeuroKinematics_OBJECTS = \
"CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o"

# External object files for target NeuroKinematics
NeuroKinematics_EXTERNAL_OBJECTS =

/home/kehan/neurobot_ws/devel/lib/libNeuroKinematics.so: NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o
/home/kehan/neurobot_ws/devel/lib/libNeuroKinematics.so: NeuroKinematics/CMakeFiles/NeuroKinematics.dir/build.make
/home/kehan/neurobot_ws/devel/lib/libNeuroKinematics.so: NeuroKinematics/CMakeFiles/NeuroKinematics.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kehan/neurobot_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/kehan/neurobot_ws/devel/lib/libNeuroKinematics.so"
	cd /home/kehan/neurobot_ws/build/NeuroKinematics && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/NeuroKinematics.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
NeuroKinematics/CMakeFiles/NeuroKinematics.dir/build: /home/kehan/neurobot_ws/devel/lib/libNeuroKinematics.so

.PHONY : NeuroKinematics/CMakeFiles/NeuroKinematics.dir/build

NeuroKinematics/CMakeFiles/NeuroKinematics.dir/requires: NeuroKinematics/CMakeFiles/NeuroKinematics.dir/src/NeuroKinematics.cpp.o.requires

.PHONY : NeuroKinematics/CMakeFiles/NeuroKinematics.dir/requires

NeuroKinematics/CMakeFiles/NeuroKinematics.dir/clean:
	cd /home/kehan/neurobot_ws/build/NeuroKinematics && $(CMAKE_COMMAND) -P CMakeFiles/NeuroKinematics.dir/cmake_clean.cmake
.PHONY : NeuroKinematics/CMakeFiles/NeuroKinematics.dir/clean

NeuroKinematics/CMakeFiles/NeuroKinematics.dir/depend:
	cd /home/kehan/neurobot_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kehan/neurobot_ws/src /home/kehan/neurobot_ws/src/NeuroKinematics /home/kehan/neurobot_ws/build /home/kehan/neurobot_ws/build/NeuroKinematics /home/kehan/neurobot_ws/build/NeuroKinematics/CMakeFiles/NeuroKinematics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : NeuroKinematics/CMakeFiles/NeuroKinematics.dir/depend

