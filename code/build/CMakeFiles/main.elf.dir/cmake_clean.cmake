file(REMOVE_RECURSE
  "bootloader/bootloader.bin"
  "bootloader/bootloader.elf"
  "bootloader/bootloader.map"
  "config/sdkconfig.cmake"
  "config/sdkconfig.h"
  "flash_project_args"
  "main.bin"
  "main.map"
  "project_elf_src_esp32.c"
  "CMakeFiles/main.elf.dir/project_elf_src_esp32.c.obj"
  "CMakeFiles/main.elf.dir/project_elf_src_esp32.c.obj.d"
  "main.elf"
  "main.elf.pdb"
  "project_elf_src_esp32.c"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/main.elf.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
