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
CMAKE_BINARY_DIR = /home/siasun/urdf/build

# Include any dependencies generated for this target.
include tf/CMakeFiles/tf_echo.dir/depend.make

# Include the progress variables for this target.
include tf/CMakeFiles/tf_echo.dir/progress.make

# Include the compile flags for this target's objects.
include tf/CMakeFiles/tf_echo.dir/flags.make

tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o: tf/CMakeFiles/tf_echo.dir/flags.make
tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o: ../tf/src/tf_echo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/siasun/urdf/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o"
	cd /home/siasun/urdf/build/tf && /opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o -c /home/siasun/urdf/tf/src/tf_echo.cpp

tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tf_echo.dir/src/tf_echo.cpp.i"
	cd /home/siasun/urdf/build/tf && /opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/siasun/urdf/tf/src/tf_echo.cpp > CMakeFiles/tf_echo.dir/src/tf_echo.cpp.i

tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tf_echo.dir/src/tf_echo.cpp.s"
	cd /home/siasun/urdf/build/tf && /opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/siasun/urdf/tf/src/tf_echo.cpp -o CMakeFiles/tf_echo.dir/src/tf_echo.cpp.s

tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o.requires:
.PHONY : tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o.requires

tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o.provides: tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o.requires
	$(MAKE) -f tf/CMakeFiles/tf_echo.dir/build.make tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o.provides.build
.PHONY : tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o.provides

tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o.provides.build: tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o

# Object files for target tf_echo
tf_echo_OBJECTS = \
"CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o"

# External object files for target tf_echo
tf_echo_EXTERNAL_OBJECTS =

tf/tf_echo: tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o
tf/tf_echo: tf/CMakeFiles/tf_echo.dir/build.make
tf/tf_echo: tf/libtf.a
tf/tf_echo: tf/CMakeFiles/tf_echo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable tf_echo"
	cd /home/siasun/urdf/build/tf && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tf_echo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tf/CMakeFiles/tf_echo.dir/build: tf/tf_echo
.PHONY : tf/CMakeFiles/tf_echo.dir/build

tf/CMakeFiles/tf_echo.dir/requires: tf/CMakeFiles/tf_echo.dir/src/tf_echo.cpp.o.requires
.PHONY : tf/CMakeFiles/tf_echo.dir/requires

tf/CMakeFiles/tf_echo.dir/clean:
	cd /home/siasun/urdf/build/tf && $(CMAKE_COMMAND) -P CMakeFiles/tf_echo.dir/cmake_clean.cmake
.PHONY : tf/CMakeFiles/tf_echo.dir/clean

tf/CMakeFiles/tf_echo.dir/depend:
	cd /home/siasun/urdf/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/siasun/urdf /home/siasun/urdf/tf /home/siasun/urdf/build /home/siasun/urdf/build/tf /home/siasun/urdf/build/tf/CMakeFiles/tf_echo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tf/CMakeFiles/tf_echo.dir/depend
