# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/siasun/urdf

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/siasun/urdf/build

# Include any dependencies generated for this target.
include CMakeFiles/urdfdom_model.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/urdfdom_model.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/urdfdom_model.dir/flags.make

CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o: CMakeFiles/urdfdom_model.dir/flags.make
CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o: ../src/urdf_parser/pose.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/siasun/urdf/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o -c /home/siasun/urdf/src/urdf_parser/pose.cpp

CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.i"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/siasun/urdf/src/urdf_parser/pose.cpp > CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.i

CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.s"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/siasun/urdf/src/urdf_parser/pose.cpp -o CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.s

CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o.requires:
.PHONY : CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o.requires

CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o.provides: CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o.requires
	$(MAKE) -f CMakeFiles/urdfdom_model.dir/build.make CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o.provides.build
.PHONY : CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o.provides

CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o.provides.build: CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o

CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o: CMakeFiles/urdfdom_model.dir/flags.make
CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o: ../src/urdf_parser/model.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/siasun/urdf/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o -c /home/siasun/urdf/src/urdf_parser/model.cpp

CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.i"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/siasun/urdf/src/urdf_parser/model.cpp > CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.i

CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.s"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/siasun/urdf/src/urdf_parser/model.cpp -o CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.s

CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o.requires:
.PHONY : CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o.requires

CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o.provides: CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o.requires
	$(MAKE) -f CMakeFiles/urdfdom_model.dir/build.make CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o.provides.build
.PHONY : CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o.provides

CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o.provides.build: CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o

CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o: CMakeFiles/urdfdom_model.dir/flags.make
CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o: ../src/urdf_parser/link.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/siasun/urdf/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o -c /home/siasun/urdf/src/urdf_parser/link.cpp

CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.i"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/siasun/urdf/src/urdf_parser/link.cpp > CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.i

CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.s"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/siasun/urdf/src/urdf_parser/link.cpp -o CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.s

CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o.requires:
.PHONY : CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o.requires

CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o.provides: CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o.requires
	$(MAKE) -f CMakeFiles/urdfdom_model.dir/build.make CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o.provides.build
.PHONY : CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o.provides

CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o.provides.build: CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o

CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o: CMakeFiles/urdfdom_model.dir/flags.make
CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o: ../src/urdf_parser/joint.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/siasun/urdf/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o -c /home/siasun/urdf/src/urdf_parser/joint.cpp

CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.i"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/siasun/urdf/src/urdf_parser/joint.cpp > CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.i

CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.s"
	/opt/windriver/wrlinux/5.0-intel-atom-baytrail/sysroots/x86_64-wrlinuxsdk-linux/usr/bin/atom-wrs-linux/atom-wrswrap-linux-gnu-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/siasun/urdf/src/urdf_parser/joint.cpp -o CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.s

CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o.requires:
.PHONY : CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o.requires

CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o.provides: CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o.requires
	$(MAKE) -f CMakeFiles/urdfdom_model.dir/build.make CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o.provides.build
.PHONY : CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o.provides

CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o.provides.build: CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o

# Object files for target urdfdom_model
urdfdom_model_OBJECTS = \
"CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o" \
"CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o" \
"CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o" \
"CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o"

# External object files for target urdfdom_model
urdfdom_model_EXTERNAL_OBJECTS =

liburdfdom_model.so: CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o
liburdfdom_model.so: CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o
liburdfdom_model.so: CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o
liburdfdom_model.so: CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o
liburdfdom_model.so: CMakeFiles/urdfdom_model.dir/build.make
liburdfdom_model.so: CMakeFiles/urdfdom_model.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library liburdfdom_model.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/urdfdom_model.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/urdfdom_model.dir/build: liburdfdom_model.so
.PHONY : CMakeFiles/urdfdom_model.dir/build

CMakeFiles/urdfdom_model.dir/requires: CMakeFiles/urdfdom_model.dir/src/urdf_parser/pose.cpp.o.requires
CMakeFiles/urdfdom_model.dir/requires: CMakeFiles/urdfdom_model.dir/src/urdf_parser/model.cpp.o.requires
CMakeFiles/urdfdom_model.dir/requires: CMakeFiles/urdfdom_model.dir/src/urdf_parser/link.cpp.o.requires
CMakeFiles/urdfdom_model.dir/requires: CMakeFiles/urdfdom_model.dir/src/urdf_parser/joint.cpp.o.requires
.PHONY : CMakeFiles/urdfdom_model.dir/requires

CMakeFiles/urdfdom_model.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/urdfdom_model.dir/cmake_clean.cmake
.PHONY : CMakeFiles/urdfdom_model.dir/clean

CMakeFiles/urdfdom_model.dir/depend:
	cd /home/siasun/urdf/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/siasun/urdf /home/siasun/urdf /home/siasun/urdf/build /home/siasun/urdf/build /home/siasun/urdf/build/CMakeFiles/urdfdom_model.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/urdfdom_model.dir/depend

