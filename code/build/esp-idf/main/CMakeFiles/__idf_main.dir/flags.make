# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# compile C with /Users/nickhardy/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc
C_DEFINES = -DESP_PLATFORM -DIDF_VER=\"v5.1-dev-3025-g49551cc48c\" -DMBEDTLS_CONFIG_FILE=\"mbedtls/esp_config.h\" -DUNITY_INCLUDE_CONFIG_H -D_GNU_SOURCE -D_POSIX_READER_WRITER_LOCKS

C_INCLUDES = -I/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code/build/config -I/Users/nickhardy/esp/esp-idf/components/newlib/platform_include -I/Users/nickhardy/esp/esp-idf/components/freertos/FreeRTOS-Kernel/include -I/Users/nickhardy/esp/esp-idf/components/freertos/FreeRTOS-Kernel/portable/xtensa/include -I/Users/nickhardy/esp/esp-idf/components/freertos/esp_additions/include/freertos -I/Users/nickhardy/esp/esp-idf/components/freertos/esp_additions/include -I/Users/nickhardy/esp/esp-idf/components/esp_hw_support/include -I/Users/nickhardy/esp/esp-idf/components/esp_hw_support/include/soc -I/Users/nickhardy/esp/esp-idf/components/esp_hw_support/include/soc/esp32 -I/Users/nickhardy/esp/esp-idf/components/esp_hw_support/port/esp32/. -I/Users/nickhardy/esp/esp-idf/components/esp_hw_support/port/esp32/private_include -I/Users/nickhardy/esp/esp-idf/components/heap/include -I/Users/nickhardy/esp/esp-idf/components/log/include -I/Users/nickhardy/esp/esp-idf/components/soc/include -I/Users/nickhardy/esp/esp-idf/components/soc/esp32 -I/Users/nickhardy/esp/esp-idf/components/soc/esp32/include -I/Users/nickhardy/esp/esp-idf/components/hal/esp32/include -I/Users/nickhardy/esp/esp-idf/components/hal/include -I/Users/nickhardy/esp/esp-idf/components/hal/platform_port/include -I/Users/nickhardy/esp/esp-idf/components/esp_rom/include -I/Users/nickhardy/esp/esp-idf/components/esp_rom/include/esp32 -I/Users/nickhardy/esp/esp-idf/components/esp_rom/esp32 -I/Users/nickhardy/esp/esp-idf/components/esp_common/include -I/Users/nickhardy/esp/esp-idf/components/esp_system/include -I/Users/nickhardy/esp/esp-idf/components/esp_system/port/soc -I/Users/nickhardy/esp/esp-idf/components/esp_system/port/include/private -I/Users/nickhardy/esp/esp-idf/components/xtensa/include -I/Users/nickhardy/esp/esp-idf/components/xtensa/esp32/include -I/Users/nickhardy/esp/esp-idf/components/lwip/include -I/Users/nickhardy/esp/esp-idf/components/lwip/include/apps -I/Users/nickhardy/esp/esp-idf/components/lwip/include/apps/sntp -I/Users/nickhardy/esp/esp-idf/components/lwip/lwip/src/include -I/Users/nickhardy/esp/esp-idf/components/lwip/port/esp32/include -I/Users/nickhardy/esp/esp-idf/components/lwip/port/esp32/include/arch -I/Users/nickhardy/esp/esp-idf/components/esp_ringbuf/include -I/Users/nickhardy/esp/esp-idf/components/efuse/include -I/Users/nickhardy/esp/esp-idf/components/efuse/esp32/include -I/Users/nickhardy/esp/esp-idf/components/driver/include -I/Users/nickhardy/esp/esp-idf/components/driver/deprecated -I/Users/nickhardy/esp/esp-idf/components/driver/esp32/include -I/Users/nickhardy/esp/esp-idf/components/esp_pm/include -I/Users/nickhardy/esp/esp-idf/components/mbedtls/port/include -I/Users/nickhardy/esp/esp-idf/components/mbedtls/mbedtls/include -I/Users/nickhardy/esp/esp-idf/components/mbedtls/mbedtls/library -I/Users/nickhardy/esp/esp-idf/components/mbedtls/esp_crt_bundle/include -I/Users/nickhardy/esp/esp-idf/components/esp_app_format/include -I/Users/nickhardy/esp/esp-idf/components/bootloader_support/include -I/Users/nickhardy/esp/esp-idf/components/bootloader_support/bootloader_flash/include -I/Users/nickhardy/esp/esp-idf/components/esp_partition/include -I/Users/nickhardy/esp/esp-idf/components/app_update/include -I/Users/nickhardy/esp/esp-idf/components/spi_flash/include -I/Users/nickhardy/esp/esp-idf/components/pthread/include -I/Users/nickhardy/esp/esp-idf/components/esp_timer/include -I/Users/nickhardy/esp/esp-idf/components/app_trace/include -I/Users/nickhardy/esp/esp-idf/components/esp_event/include -I/Users/nickhardy/esp/esp-idf/components/nvs_flash/include -I/Users/nickhardy/esp/esp-idf/components/esp_phy/include -I/Users/nickhardy/esp/esp-idf/components/esp_phy/esp32/include -I/Users/nickhardy/esp/esp-idf/components/vfs/include -I/Users/nickhardy/esp/esp-idf/components/esp_netif/include -I/Users/nickhardy/esp/esp-idf/components/wpa_supplicant/include -I/Users/nickhardy/esp/esp-idf/components/wpa_supplicant/port/include -I/Users/nickhardy/esp/esp-idf/components/wpa_supplicant/esp_supplicant/include -I/Users/nickhardy/esp/esp-idf/components/esp_wifi/include -I/Users/nickhardy/esp/esp-idf/components/unity/include -I/Users/nickhardy/esp/esp-idf/components/unity/unity/src -I/Users/nickhardy/esp/esp-idf/components/cmock/CMock/src -I/Users/nickhardy/esp/esp-idf/components/console -I/Users/nickhardy/esp/esp-idf/components/http_parser -I/Users/nickhardy/esp/esp-idf/components/esp-tls -I/Users/nickhardy/esp/esp-idf/components/esp-tls/esp-tls-crypto -I/Users/nickhardy/esp/esp-idf/components/esp_adc/include -I/Users/nickhardy/esp/esp-idf/components/esp_adc/interface -I/Users/nickhardy/esp/esp-idf/components/esp_adc/esp32/include -I/Users/nickhardy/esp/esp-idf/components/esp_adc/deprecated/include -I/Users/nickhardy/esp/esp-idf/components/esp_eth/include -I/Users/nickhardy/esp/esp-idf/components/esp_gdbstub/include -I/Users/nickhardy/esp/esp-idf/components/esp_gdbstub/xtensa -I/Users/nickhardy/esp/esp-idf/components/esp_gdbstub/esp32 -I/Users/nickhardy/esp/esp-idf/components/esp_hid/include -I/Users/nickhardy/esp/esp-idf/components/tcp_transport/include -I/Users/nickhardy/esp/esp-idf/components/esp_http_client/include -I/Users/nickhardy/esp/esp-idf/components/esp_http_server/include -I/Users/nickhardy/esp/esp-idf/components/esp_https_ota/include -I/Users/nickhardy/esp/esp-idf/components/esp_lcd/include -I/Users/nickhardy/esp/esp-idf/components/esp_lcd/interface -I/Users/nickhardy/esp/esp-idf/components/protobuf-c/protobuf-c -I/Users/nickhardy/esp/esp-idf/components/protocomm/include/common -I/Users/nickhardy/esp/esp-idf/components/protocomm/include/security -I/Users/nickhardy/esp/esp-idf/components/protocomm/include/transports -I/Users/nickhardy/esp/esp-idf/components/esp_local_ctrl/include -I/Users/nickhardy/esp/esp-idf/components/esp_psram/include -I/Users/nickhardy/esp/esp-idf/components/espcoredump/include -I/Users/nickhardy/esp/esp-idf/components/espcoredump/include/port/xtensa -I/Users/nickhardy/esp/esp-idf/components/wear_levelling/include -I/Users/nickhardy/esp/esp-idf/components/sdmmc/include -I/Users/nickhardy/esp/esp-idf/components/fatfs/diskio -I/Users/nickhardy/esp/esp-idf/components/fatfs/vfs -I/Users/nickhardy/esp/esp-idf/components/fatfs/src -I/Users/nickhardy/esp/esp-idf/components/idf_test/include -I/Users/nickhardy/esp/esp-idf/components/idf_test/include/esp32 -I/Users/nickhardy/esp/esp-idf/components/ieee802154/include -I/Users/nickhardy/esp/esp-idf/components/json/cJSON -I/Users/nickhardy/esp/esp-idf/components/mqtt/esp-mqtt/include -I/Users/nickhardy/esp/esp-idf/components/perfmon/include -I/Users/nickhardy/esp/esp-idf/components/spiffs/include -I/Users/nickhardy/esp/esp-idf/components/ulp/ulp_common/include -I/Users/nickhardy/esp/esp-idf/components/ulp/ulp_common/include/esp32 -I/Users/nickhardy/esp/esp-idf/components/wifi_provisioning/include

C_FLAGS = -mlongcalls -Wno-frame-address  -fdiagnostics-color=always -ffunction-sections -fdata-sections -Wall -Werror=all -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=unused-but-set-variable -Wno-error=deprecated-declarations -Wextra -Wno-unused-parameter -Wno-sign-compare -Wno-enum-conversion -gdwarf-4 -ggdb -Og -fmacro-prefix-map=/Users/nickhardy/Documents/GitHub/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-1/code=. -fmacro-prefix-map=/Users/nickhardy/esp/esp-idf=/IDF -fstrict-volatile-bitfields -fno-jump-tables -fno-tree-switch-conversion -DconfigENABLE_FREERTOS_DEBUG_OCDAWARE=1 -std=gnu17 -Wno-old-style-declaration

