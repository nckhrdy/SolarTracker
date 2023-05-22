# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/nickhardy/esp/esp-idf/components/bootloader/subproject"
  "/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader"
  "/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader-prefix"
  "/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader-prefix/tmp"
  "/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader-prefix/src"
  "/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
