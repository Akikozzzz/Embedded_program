# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/team6/Project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/team6/Project/build

# Include any dependencies generated for this target.
include CMakeFiles/scd41_driver.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/scd41_driver.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/scd41_driver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/scd41_driver.dir/flags.make

CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.o: CMakeFiles/scd41_driver.dir/flags.make
CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.o: /home/team6/Project/SCD041_TEST_CALLBACK.cpp
CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.o: CMakeFiles/scd41_driver.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/team6/Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.o -MF CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.o.d -o CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.o -c /home/team6/Project/SCD041_TEST_CALLBACK.cpp

CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/team6/Project/SCD041_TEST_CALLBACK.cpp > CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.i

CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/team6/Project/SCD041_TEST_CALLBACK.cpp -o CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.s

# Object files for target scd41_driver
scd41_driver_OBJECTS = \
"CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.o"

# External object files for target scd41_driver
scd41_driver_EXTERNAL_OBJECTS =

scd41_driver: CMakeFiles/scd41_driver.dir/SCD041_TEST_CALLBACK.cpp.o
scd41_driver: CMakeFiles/scd41_driver.dir/build.make
scd41_driver: CMakeFiles/scd41_driver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/team6/Project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable scd41_driver"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/scd41_driver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/scd41_driver.dir/build: scd41_driver
.PHONY : CMakeFiles/scd41_driver.dir/build

CMakeFiles/scd41_driver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/scd41_driver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/scd41_driver.dir/clean

CMakeFiles/scd41_driver.dir/depend:
	cd /home/team6/Project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/team6/Project /home/team6/Project /home/team6/Project/build /home/team6/Project/build /home/team6/Project/build/CMakeFiles/scd41_driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/scd41_driver.dir/depend

