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

# Include any dependencies generated for this target.
include esp-idf/hal/CMakeFiles/__idf_hal.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include esp-idf/hal/CMakeFiles/__idf_hal.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/hal/CMakeFiles/__idf_hal.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/hal/CMakeFiles/__idf_hal.dir/flags.make

esp-idf/hal/CMakeFiles/__idf_hal.dir/mpu_hal.c.obj: esp-idf/hal/CMakeFiles/__idf_hal.dir/flags.make
esp-idf/hal/CMakeFiles/__idf_hal.dir/mpu_hal.c.obj: /Users/nickhardy/esp/esp-idf/components/hal/mpu_hal.c
esp-idf/hal/CMakeFiles/__idf_hal.dir/mpu_hal.c.obj: esp-idf/hal/CMakeFiles/__idf_hal.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/hal/CMakeFiles/__idf_hal.dir/mpu_hal.c.obj"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/hal/CMakeFiles/__idf_hal.dir/mpu_hal.c.obj -MF CMakeFiles/__idf_hal.dir/mpu_hal.c.obj.d -o CMakeFiles/__idf_hal.dir/mpu_hal.c.obj -c /Users/nickhardy/esp/esp-idf/components/hal/mpu_hal.c

esp-idf/hal/CMakeFiles/__idf_hal.dir/mpu_hal.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_hal.dir/mpu_hal.c.i"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/nickhardy/esp/esp-idf/components/hal/mpu_hal.c > CMakeFiles/__idf_hal.dir/mpu_hal.c.i

esp-idf/hal/CMakeFiles/__idf_hal.dir/mpu_hal.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_hal.dir/mpu_hal.c.s"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/nickhardy/esp/esp-idf/components/hal/mpu_hal.c -o CMakeFiles/__idf_hal.dir/mpu_hal.c.s

esp-idf/hal/CMakeFiles/__idf_hal.dir/efuse_hal.c.obj: esp-idf/hal/CMakeFiles/__idf_hal.dir/flags.make
esp-idf/hal/CMakeFiles/__idf_hal.dir/efuse_hal.c.obj: /Users/nickhardy/esp/esp-idf/components/hal/efuse_hal.c
esp-idf/hal/CMakeFiles/__idf_hal.dir/efuse_hal.c.obj: esp-idf/hal/CMakeFiles/__idf_hal.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/hal/CMakeFiles/__idf_hal.dir/efuse_hal.c.obj"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/hal/CMakeFiles/__idf_hal.dir/efuse_hal.c.obj -MF CMakeFiles/__idf_hal.dir/efuse_hal.c.obj.d -o CMakeFiles/__idf_hal.dir/efuse_hal.c.obj -c /Users/nickhardy/esp/esp-idf/components/hal/efuse_hal.c

esp-idf/hal/CMakeFiles/__idf_hal.dir/efuse_hal.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_hal.dir/efuse_hal.c.i"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/nickhardy/esp/esp-idf/components/hal/efuse_hal.c > CMakeFiles/__idf_hal.dir/efuse_hal.c.i

esp-idf/hal/CMakeFiles/__idf_hal.dir/efuse_hal.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_hal.dir/efuse_hal.c.s"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/nickhardy/esp/esp-idf/components/hal/efuse_hal.c -o CMakeFiles/__idf_hal.dir/efuse_hal.c.s

esp-idf/hal/CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.obj: esp-idf/hal/CMakeFiles/__idf_hal.dir/flags.make
esp-idf/hal/CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.obj: /Users/nickhardy/esp/esp-idf/components/hal/esp32/efuse_hal.c
esp-idf/hal/CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.obj: esp-idf/hal/CMakeFiles/__idf_hal.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/hal/CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.obj"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/hal/CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.obj -MF CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.obj.d -o CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.obj -c /Users/nickhardy/esp/esp-idf/components/hal/esp32/efuse_hal.c

esp-idf/hal/CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.i"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/nickhardy/esp/esp-idf/components/hal/esp32/efuse_hal.c > CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.i

esp-idf/hal/CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.s"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/nickhardy/esp/esp-idf/components/hal/esp32/efuse_hal.c -o CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.s

esp-idf/hal/CMakeFiles/__idf_hal.dir/mmu_hal.c.obj: esp-idf/hal/CMakeFiles/__idf_hal.dir/flags.make
esp-idf/hal/CMakeFiles/__idf_hal.dir/mmu_hal.c.obj: /Users/nickhardy/esp/esp-idf/components/hal/mmu_hal.c
esp-idf/hal/CMakeFiles/__idf_hal.dir/mmu_hal.c.obj: esp-idf/hal/CMakeFiles/__idf_hal.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object esp-idf/hal/CMakeFiles/__idf_hal.dir/mmu_hal.c.obj"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/hal/CMakeFiles/__idf_hal.dir/mmu_hal.c.obj -MF CMakeFiles/__idf_hal.dir/mmu_hal.c.obj.d -o CMakeFiles/__idf_hal.dir/mmu_hal.c.obj -c /Users/nickhardy/esp/esp-idf/components/hal/mmu_hal.c

esp-idf/hal/CMakeFiles/__idf_hal.dir/mmu_hal.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_hal.dir/mmu_hal.c.i"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/nickhardy/esp/esp-idf/components/hal/mmu_hal.c > CMakeFiles/__idf_hal.dir/mmu_hal.c.i

esp-idf/hal/CMakeFiles/__idf_hal.dir/mmu_hal.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_hal.dir/mmu_hal.c.s"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/nickhardy/esp/esp-idf/components/hal/mmu_hal.c -o CMakeFiles/__idf_hal.dir/mmu_hal.c.s

esp-idf/hal/CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.obj: esp-idf/hal/CMakeFiles/__idf_hal.dir/flags.make
esp-idf/hal/CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.obj: /Users/nickhardy/esp/esp-idf/components/hal/wdt_hal_iram.c
esp-idf/hal/CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.obj: esp-idf/hal/CMakeFiles/__idf_hal.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object esp-idf/hal/CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.obj"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/hal/CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.obj -MF CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.obj.d -o CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.obj -c /Users/nickhardy/esp/esp-idf/components/hal/wdt_hal_iram.c

esp-idf/hal/CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.i"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/nickhardy/esp/esp-idf/components/hal/wdt_hal_iram.c > CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.i

esp-idf/hal/CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.s"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/nickhardy/esp/esp-idf/components/hal/wdt_hal_iram.c -o CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.s

# Object files for target __idf_hal
__idf_hal_OBJECTS = \
"CMakeFiles/__idf_hal.dir/mpu_hal.c.obj" \
"CMakeFiles/__idf_hal.dir/efuse_hal.c.obj" \
"CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.obj" \
"CMakeFiles/__idf_hal.dir/mmu_hal.c.obj" \
"CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.obj"

# External object files for target __idf_hal
__idf_hal_EXTERNAL_OBJECTS =

esp-idf/hal/libhal.a: esp-idf/hal/CMakeFiles/__idf_hal.dir/mpu_hal.c.obj
esp-idf/hal/libhal.a: esp-idf/hal/CMakeFiles/__idf_hal.dir/efuse_hal.c.obj
esp-idf/hal/libhal.a: esp-idf/hal/CMakeFiles/__idf_hal.dir/esp32/efuse_hal.c.obj
esp-idf/hal/libhal.a: esp-idf/hal/CMakeFiles/__idf_hal.dir/mmu_hal.c.obj
esp-idf/hal/libhal.a: esp-idf/hal/CMakeFiles/__idf_hal.dir/wdt_hal_iram.c.obj
esp-idf/hal/libhal.a: esp-idf/hal/CMakeFiles/__idf_hal.dir/build.make
esp-idf/hal/libhal.a: esp-idf/hal/CMakeFiles/__idf_hal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C static library libhal.a"
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && $(CMAKE_COMMAND) -P CMakeFiles/__idf_hal.dir/cmake_clean_target.cmake
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_hal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/hal/CMakeFiles/__idf_hal.dir/build: esp-idf/hal/libhal.a
.PHONY : esp-idf/hal/CMakeFiles/__idf_hal.dir/build

esp-idf/hal/CMakeFiles/__idf_hal.dir/clean:
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal && $(CMAKE_COMMAND) -P CMakeFiles/__idf_hal.dir/cmake_clean.cmake
.PHONY : esp-idf/hal/CMakeFiles/__idf_hal.dir/clean

esp-idf/hal/CMakeFiles/__idf_hal.dir/depend:
	cd /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/nickhardy/esp/esp-idf/components/bootloader/subproject /Users/nickhardy/esp/esp-idf/components/hal /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal /Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader/esp-idf/hal/CMakeFiles/__idf_hal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/hal/CMakeFiles/__idf_hal.dir/depend

