add_executable(hello_world_psram main.c)

target_include_directories(hello_world_psram PRIVATE
    ${CMAKE_SOURCE_DIR}/port
)

target_compile_options(hello_world_psram PRIVATE
    -Os -g3 -Wall -Wextra -ffunction-sections -fdata-sections
)

set(APP_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/app.ld)
target_link_options(hello_world_psram PRIVATE
    -T ${APP_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/hello_world_psram.map
    -Wl,--gc-sections
)
# Libraries must come after objects in the link command; target_link_libraries
# guarantees that — target_link_options would place them before the .obj files.
target_link_libraries(hello_world_psram PRIVATE gcc c m)
set_property(TARGET hello_world_psram APPEND PROPERTY LINK_DEPENDS ${APP_LINKER_SCRIPT})

set_target_properties(hello_world_psram PROPERTIES
    OUTPUT_NAME "hello_world_psram"
    SUFFIX ".elf"
)

set(HELLO_PSRAM_BIN ${CMAKE_CURRENT_BINARY_DIR}/hello_world_psram.bin)
add_custom_command(
    OUTPUT ${HELLO_PSRAM_BIN}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:hello_world_psram> ${HELLO_PSRAM_BIN}
    DEPENDS hello_world_psram
    COMMENT "[app] hello_world_psram.elf -> hello_world_psram.bin"
    VERBATIM
)
add_custom_target(hello_world_psram_bin ALL DEPENDS ${HELLO_PSRAM_BIN})
