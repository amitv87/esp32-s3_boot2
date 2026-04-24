set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR xtensa)

if(NOT DEFINED ENV{ESP32S3_TOOLCHAIN_PREFIX})
  set(ENV{ESP32S3_TOOLCHAIN_PREFIX}
      "/Volumes/boggy/toolchain/xtensa-esp-elf-15.2.0_20251204/bin/xtensa-esp32s3-elf-")
endif()
set(TOOLCHAIN_PREFIX "$ENV{ESP32S3_TOOLCHAIN_PREFIX}")

set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_OBJDUMP      "${TOOLCHAIN_PREFIX}objdump")
set(CMAKE_SIZE         "${TOOLCHAIN_PREFIX}size")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CPU_FLAGS "-mlongcalls -fno-builtin-memcpy -fno-builtin-memset -fno-builtin-bzero")
set(CMAKE_C_FLAGS_INIT   "${CPU_FLAGS} -ffreestanding -fno-builtin")
set(CMAKE_ASM_FLAGS_INIT "${CPU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-nostartfiles -nostdlib -Wl,--gc-sections")
