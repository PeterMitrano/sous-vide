set(CMAKE_SYSTEM_NAME Generic)

set(ESP_ROOT /home/peter/.arduino15/packages/esp8266)
set(TOOLCHAIN_ROOT ${ESP_ROOT}/tools/xtensa-lx106-elf-gcc/1.20.0-26-gb404fb9-2/bin)
set(TRIPLE "xtensa-lx106-elf")
set(CMAKE_SYSTEM_PROCESSOR xtensia)
set(CMAKE_CROSSCOMPILING 1)
set(BASE "${TOOLCHAIN_ROOT}/${TRIPLE}")

set(CMAKE_C_COMPILER "${BASE}-gcc" CACHE PATH "gcc" FORCE)
set(CMAKE_CXX_COMPILER "${BASE}-g++" CACHE PATH "g++" FORCE)
set(CMAKE_AR "${BASE}-ar" CACHE PATH "archive" FORCE)
set(CMAKE_LINKER "${BASE}-ld" CACHE PATH "linker" FORCE)
set(CMAKE_RANLIB "${BASE}-ranlib" CACHE PATH "ranlib" FORCE)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_C_FLAGS "-w -Os -g -std=gnu99 -Wpointer-arith -Wno-implicit-function-declaration -Wundef -pipe -D__ets__ -DICACHE_FLASH -fno-inline-functions -ffunction-sections -nostdlib -mlongcalls -mtext-section-literals -falign-functions=4 -fdata-sections" CACHE STRING "C compiler flags" FORCE)
set(CMAKE_CXX_FLAGS "-w -Os -g -D__ets__ -DICACHE_FLASH -mlongcalls -mtext-section-literals -fno-exceptions -fno-rtti -falign-functions=4 -std=c++11 -MMD -ffunction-sections -fdata-sections" CACHE STRING "CXX compiler flags" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "-nostdlib -Wl,--no-check-sections -Wl,-static -Wl,--gc-sections -L${ESP_ROOT}/hardware/esp8266/2.3.0/tools/sdk/ld -Teagle.flash.2m.ld -u call_user_start -Wl,-wrap,system_restart_local -Wl,-wrap,register_chipv6_phy" CACHE STRING "linker flags" FORCE)

set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> -o <TARGET> -Wl,--start-group <OBJECTS> <LINK_LIBRARIES> -Wl,--end-group" CACHE STRING "C linker invocation")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_COMPILER> <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> -o <TARGET> -Wl,--start-group <OBJECTS> <LINK_LIBRARIES> -Wl,--end-group" CACHE STRING "CXX linker invocation")

