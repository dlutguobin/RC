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
CMAKE_COMMAND = /opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/cmake

# The command to remove a file.
RM = /opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/siasun/urdf

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/siasun/urdf/cmake-build-release

# Include any dependencies generated for this target.
include kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/depend.make

# Include the progress variables for this target.
include kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/progress.make

# Include the compile flags for this target's objects.
include kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/flags.make

kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o: kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/flags.make
kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o: ../kinematics_plugin_loader/src/kinematics_plugin_loader.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/siasun/urdf/cmake-build-release/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o"
	cd /home/siasun/urdf/cmake-build-release/kinematics_plugin_loader && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o -c /home/siasun/urdf/kinematics_plugin_loader/src/kinematics_plugin_loader.cpp

kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.i"
	cd /home/siasun/urdf/cmake-build-release/kinematics_plugin_loader && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/siasun/urdf/kinematics_plugin_loader/src/kinematics_plugin_loader.cpp > CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.i

kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.s"
	cd /home/siasun/urdf/cmake-build-release/kinematics_plugin_loader && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/siasun/urdf/kinematics_plugin_loader/src/kinematics_plugin_loader.cpp -o CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.s

kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o.requires:
.PHONY : kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o.requires

kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o.provides: kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o.requires
	$(MAKE) -f kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/build.make kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o.provides.build
.PHONY : kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o.provides

kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o.provides.build: kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o

# Object files for target moveit_kinematics_plugin_loader
moveit_kinematics_plugin_loader_OBJECTS = \
"CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o"

# External object files for target moveit_kinematics_plugin_loader
moveit_kinematics_plugin_loader_EXTERNAL_OBJECTS =

kinematics_plugin_loader/libmoveit_kinematics_plugin_loader.a: kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o
kinematics_plugin_loader/libmoveit_kinematics_plugin_loader.a: kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/build.make
kinematics_plugin_loader/libmoveit_kinematics_plugin_loader.a: kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libmoveit_kinematics_plugin_loader.a"
	cd /home/siasun/urdf/cmake-build-release/kinematics_plugin_loader && $(CMAKE_COMMAND) -P CMakeFiles/moveit_kinematics_plugin_loader.dir/cmake_clean_target.cmake
	cd /home/siasun/urdf/cmake-build-release/kinematics_plugin_loader && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/moveit_kinematics_plugin_loader.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/build: kinematics_plugin_loader/libmoveit_kinematics_plugin_loader.a
.PHONY : kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/build

kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/requires: kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/src/kinematics_plugin_loader.cpp.o.requires
.PHONY : kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/requires

kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/clean:
	cd /home/siasun/urdf/cmake-build-release/kinematics_plugin_loader && $(CMAKE_COMMAND) -P CMakeFiles/moveit_kinematics_plugin_loader.dir/cmake_clean.cmake
.PHONY : kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/clean

kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/depend:
	cd /home/siasun/urdf/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/siasun/urdf /home/siasun/urdf/kinematics_plugin_loader /home/siasun/urdf/cmake-build-release /home/siasun/urdf/cmake-build-release/kinematics_plugin_loader /home/siasun/urdf/cmake-build-release/kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : kinematics_plugin_loader/CMakeFiles/moveit_kinematics_plugin_loader.dir/depend
