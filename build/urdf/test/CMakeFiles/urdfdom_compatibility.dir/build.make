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
CMAKE_SOURCE_DIR = /home/siasun/urdf

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/siasun/urdf/build

# Include any dependencies generated for this target.
include urdf/test/CMakeFiles/urdfdom_compatibility.dir/depend.make

# Include the progress variables for this target.
include urdf/test/CMakeFiles/urdfdom_compatibility.dir/progress.make

# Include the compile flags for this target's objects.
include urdf/test/CMakeFiles/urdfdom_compatibility.dir/flags.make

urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o: urdf/test/CMakeFiles/urdfdom_compatibility.dir/flags.make
urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o: ../urdf/test/urdfdom_compatibility.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/siasun/urdf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o"
	cd /home/siasun/urdf/build/urdf/test && /opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o -c /home/siasun/urdf/urdf/test/urdfdom_compatibility.cpp

urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.i"
	cd /home/siasun/urdf/build/urdf/test && /opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/siasun/urdf/urdf/test/urdfdom_compatibility.cpp > CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.i

urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.s"
	cd /home/siasun/urdf/build/urdf/test && /opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/siasun/urdf/urdf/test/urdfdom_compatibility.cpp -o CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.s

urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o.requires:

.PHONY : urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o.requires

urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o.provides: urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o.requires
	$(MAKE) -f urdf/test/CMakeFiles/urdfdom_compatibility.dir/build.make urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o.provides.build
.PHONY : urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o.provides

urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o.provides.build: urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o


# Object files for target urdfdom_compatibility
urdfdom_compatibility_OBJECTS = \
"CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o"

# External object files for target urdfdom_compatibility
urdfdom_compatibility_EXTERNAL_OBJECTS =

urdf/test/urdfdom_compatibility: urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o
urdf/test/urdfdom_compatibility: urdf/test/CMakeFiles/urdfdom_compatibility.dir/build.make
urdf/test/urdfdom_compatibility: urdf/test/libgtest_main.a
urdf/test/urdfdom_compatibility: urdf/test/libgtest.a
urdf/test/urdfdom_compatibility: urdf/liburdf.so
urdf/test/urdfdom_compatibility: urdf/liburdfdom_model.so
urdf/test/urdfdom_compatibility: urdf/test/CMakeFiles/urdfdom_compatibility.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/siasun/urdf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable urdfdom_compatibility"
	cd /home/siasun/urdf/build/urdf/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/urdfdom_compatibility.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
urdf/test/CMakeFiles/urdfdom_compatibility.dir/build: urdf/test/urdfdom_compatibility

.PHONY : urdf/test/CMakeFiles/urdfdom_compatibility.dir/build

urdf/test/CMakeFiles/urdfdom_compatibility.dir/requires: urdf/test/CMakeFiles/urdfdom_compatibility.dir/urdfdom_compatibility.cpp.o.requires

.PHONY : urdf/test/CMakeFiles/urdfdom_compatibility.dir/requires

urdf/test/CMakeFiles/urdfdom_compatibility.dir/clean:
	cd /home/siasun/urdf/build/urdf/test && $(CMAKE_COMMAND) -P CMakeFiles/urdfdom_compatibility.dir/cmake_clean.cmake
.PHONY : urdf/test/CMakeFiles/urdfdom_compatibility.dir/clean

urdf/test/CMakeFiles/urdfdom_compatibility.dir/depend:
	cd /home/siasun/urdf/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/siasun/urdf /home/siasun/urdf/urdf/test /home/siasun/urdf/build /home/siasun/urdf/build/urdf/test /home/siasun/urdf/build/urdf/test/CMakeFiles/urdfdom_compatibility.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : urdf/test/CMakeFiles/urdfdom_compatibility.dir/depend

