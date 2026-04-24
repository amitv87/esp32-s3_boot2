cmake_minimum_required(VERSION 3.20)
project(esp32s3_boot2 C ASM)

# Guard: fail early if someone configured without our toolchain file.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Generic")
  message(FATAL_ERROR
    "Toolchain file not set. Configure with:\n"
    "  cmake -B <builddir> -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-esp32s3.cmake -S .")
endif()

# ---- Build-time options ----
set(BOOT2_IO    "USB_JTAG"  CACHE STRING "IO channel: UART | USB_JTAG")
set(BOOT2_PSRAM "OFF"   CACHE STRING "PSRAM support: OFF | OCTAL")
option(BOOT2_BUILD_TESTS "Build test apps under test/" OFF)
set_property(CACHE BOOT2_IO    PROPERTY STRINGS "UART" "USB_JTAG")
set_property(CACHE BOOT2_PSRAM PROPERTY STRINGS "OFF" "OCTAL")

# ---- Sources ----
set(BOOT2_SOURCES
    boot2/start.S
    boot2/main.c
    boot2/loader.c
    boot2/wdt.c
)

if(BOOT2_IO STREQUAL "UART")
    list(APPEND BOOT2_SOURCES boot2/io_uart.c)
elseif(BOOT2_IO STREQUAL "USB_JTAG")
    list(APPEND BOOT2_SOURCES boot2/io_usb_jtag.c)
else()
    message(FATAL_ERROR "BOOT2_IO must be UART or USB_JTAG, got: ${BOOT2_IO}")
endif()

if(BOOT2_PSRAM STREQUAL "OCTAL")
    list(APPEND BOOT2_SOURCES boot2/psram.c)
endif()

add_executable(boot2 ${BOOT2_SOURCES})

# ---- Compile / link options ----
target_compile_options(boot2 PRIVATE -Os -g3 -Wall -Wextra -Wno-unused-parameter)

target_include_directories(boot2 PRIVATE
    boot2
    port
    vendor/soc/esp32s3/register/soc
    vendor/esp_common
)

target_compile_definitions(boot2 PRIVATE
    $<$<STREQUAL:${BOOT2_IO},UART>:BOOT2_IO_UART=1>
    $<$<STREQUAL:${BOOT2_IO},USB_JTAG>:BOOT2_IO_USB_JTAG=1>
    $<$<STREQUAL:${BOOT2_PSRAM},OCTAL>:BOOT2_PSRAM_OCTAL=1>
)

# Linker script + ROM symbol scripts (added in Phase 3 and Phase 2 respectively).
# For Phase 1, we leave the link line bare so we can verify the toolchain picks
# up the right binaries and builds a relocatable.
set(BOOT2_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker/boot2.ld)
set(BOOT2_ROM_SCRIPTS
    ${CMAKE_SOURCE_DIR}/vendor/esp_rom/esp32s3/ld/esp32s3.rom.ld
    ${CMAKE_SOURCE_DIR}/vendor/esp_rom/esp32s3/ld/esp32s3.rom.api.ld
)

if(EXISTS ${BOOT2_LINKER_SCRIPT})
    target_link_options(boot2 PRIVATE
        -T ${BOOT2_LINKER_SCRIPT}
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/boot2.map
    )
    set_property(TARGET boot2 APPEND PROPERTY LINK_DEPENDS ${BOOT2_LINKER_SCRIPT})

    foreach(rom_ld IN LISTS BOOT2_ROM_SCRIPTS)
        if(EXISTS ${rom_ld})
            target_link_options(boot2 PRIVATE -T ${rom_ld})
            set_property(TARGET boot2 APPEND PROPERTY LINK_DEPENDS ${rom_ld})
        endif()
    endforeach()
endif()

set_target_properties(boot2 PROPERTIES
    OUTPUT_NAME "boot2"
    SUFFIX ".elf"
)

# Post-build image packer. Using OUTPUT-style add_custom_command so it
# re-runs whenever either the ELF or the packer script changes.
set(BOOT2_BIN ${CMAKE_CURRENT_BINARY_DIR}/boot2.bin)
set(PACK_SCRIPT ${CMAKE_SOURCE_DIR}/scripts/pack_image.py)

if(EXISTS ${PACK_SCRIPT})
    add_custom_command(
        OUTPUT ${BOOT2_BIN}
        COMMAND python3 ${PACK_SCRIPT} $<TARGET_FILE:boot2> ${BOOT2_BIN}
        DEPENDS boot2 ${PACK_SCRIPT}
        COMMENT "[pack] boot2.elf -> boot2.bin"
        VERBATIM
    )
    add_custom_target(boot2_bin ALL DEPENDS ${BOOT2_BIN})
endif()

# ---- Optional test apps ----
if(BOOT2_BUILD_TESTS)
    add_subdirectory(test/hello-world)
endif()
