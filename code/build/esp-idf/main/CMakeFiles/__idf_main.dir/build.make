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
CMAKE_SOURCE_DIR = /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build

# Include any dependencies generated for this target.
include esp-idf/main/CMakeFiles/__idf_main.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include esp-idf/main/CMakeFiles/__idf_main.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/main/CMakeFiles/__idf_main.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/main/CMakeFiles/__idf_main.dir/flags.make

esp-idf/main/CMakeFiles/__idf_main.dir/solarcode2.c.obj: esp-idf/main/CMakeFiles/__idf_main.dir/flags.make
esp-idf/main/CMakeFiles/__idf_main.dir/solarcode2.c.obj: /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/main/solarcode2.c
esp-idf/main/CMakeFiles/__idf_main.dir/solarcode2.c.obj: esp-idf/main/CMakeFiles/__idf_main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/main/CMakeFiles/__idf_main.dir/solarcode2.c.obj"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/esp-idf/main && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/main/CMakeFiles/__idf_main.dir/solarcode2.c.obj -MF CMakeFiles/__idf_main.dir/solarcode2.c.obj.d -o CMakeFiles/__idf_main.dir/solarcode2.c.obj -c /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/main/solarcode2.c

esp-idf/main/CMakeFiles/__idf_main.dir/solarcode2.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_main.dir/solarcode2.c.i"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/esp-idf/main && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/main/solarcode2.c > CMakeFiles/__idf_main.dir/solarcode2.c.i

esp-idf/main/CMakeFiles/__idf_main.dir/solarcode2.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_main.dir/solarcode2.c.s"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/esp-idf/main && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/main/solarcode2.c -o CMakeFiles/__idf_main.dir/solarcode2.c.s

# Object files for target __idf_main
__idf_main_OBJECTS = \
"CMakeFiles/__idf_main.dir/solarcode2.c.obj"

# External object files for target __idf_main
__idf_main_EXTERNAL_OBJECTS =

esp-idf/main/libmain.a: esp-idf/main/CMakeFiles/__idf_main.dir/solarcode2.c.obj
esp-idf/main/libmain.a: esp-idf/main/CMakeFiles/__idf_main.dir/build.make
esp-idf/main/libmain.a: esp-idf/main/CMakeFiles/__idf_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libmain.a"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/esp-idf/main && $(CMAKE_COMMAND) -P CMakeFiles/__idf_main.dir/cmake_clean_target.cmake
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/esp-idf/main && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/main/CMakeFiles/__idf_main.dir/build: esp-idf/main/libmain.a
.PHONY : esp-idf/main/CMakeFiles/__idf_main.dir/build

esp-idf/main/CMakeFiles/__idf_main.dir/clean:
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/esp-idf/main && $(CMAKE_COMMAND) -P CMakeFiles/__idf_main.dir/cmake_clean.cmake
.PHONY : esp-idf/main/CMakeFiles/__idf_main.dir/clean

esp-idf/main/CMakeFiles/__idf_main.dir/depend:
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/main /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/esp-idf/main /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/esp-idf/main/CMakeFiles/__idf_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/main/CMakeFiles/__idf_main.dir/depend

