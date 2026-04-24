#
# Hello-world test app for boot2.
#
# Expects to inherit the Xtensa toolchain from the top-level CMake
# invocation. Output: hello_world.elf + hello_world.bin (raw byte
# stream sendable over the boot2 UART/USB channel).
#
# Gated by BOOT2_BUILD_TESTS=ON (default OFF) so this only builds
# when the user asks.
#

add_executable(hello_world main.c)

target_include_directories(hello_world PRIVATE
    ${CMAKE_SOURCE_DIR}/port          # for app_header.h
)

target_compile_options(hello_world PRIVATE
    -Os -g3 -Wall -Wextra -ffunction-sections -fdata-sections
)

set(APP_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/app.ld)
target_link_options(hello_world PRIVATE
    -T ${APP_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/hello_world.map
    -Wl,--gc-sections
)
set_property(TARGET hello_world APPEND PROPERTY LINK_DEPENDS ${APP_LINKER_SCRIPT})

set_target_properties(hello_world PROPERTIES
    OUTPUT_NAME "hello_world"
    SUFFIX ".elf"
)

# Raw binary — exactly what boot2 expects on the wire. Use objcopy -O
# binary, not the ESP image packer (this is a custom format, not an
# ESP image).
set(HELLO_BIN ${CMAKE_CURRENT_BINARY_DIR}/hello_world.bin)
add_custom_command(
    OUTPUT ${HELLO_BIN}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:hello_world> ${HELLO_BIN}
    DEPENDS hello_world
    COMMENT "[app] hello_world.elf -> hello_world.bin"
    VERBATIM
)
add_custom_target(hello_world_bin ALL DEPENDS ${HELLO_BIN})
