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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.25.2/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.25.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/nickhardy/esp/esp-idf/components/bootloader/subproject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader

# Utility rule file for uf2.

# Include any custom commands dependencies for this target.
include CMakeFiles/uf2.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/uf2.dir/progress.make

CMakeFiles/uf2: bootloader
	/Users/nickhardy/.espressif/python_env/idf5.1_py3.9_env/bin/python /Users/nickhardy/esp/esp-idf/tools/mkuf2.py write -o /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/uf2.bin --json /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/flasher_args.json --chip-id 0x1c5f21b0

uf2: CMakeFiles/uf2
uf2: CMakeFiles/uf2.dir/build.make
.PHONY : uf2

# Rule to build all files generated by this target.
CMakeFiles/uf2.dir/build: uf2
.PHONY : CMakeFiles/uf2.dir/build

CMakeFiles/uf2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/uf2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/uf2.dir/clean

CMakeFiles/uf2.dir/depend:
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/nickhardy/esp/esp-idf/components/bootloader/subproject /Users/nickhardy/esp/esp-idf/components/bootloader/subproject /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/CMakeFiles/uf2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/uf2.dir/depend

