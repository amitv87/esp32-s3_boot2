add_executable(touch_lcd
    main.c
    printf.c
    lcd_axs15231b.c
    touch_axs15231b.c
)

target_include_directories(touch_lcd PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/port
    # IDF SoC headers (register definitions)
    ${CMAKE_SOURCE_DIR}/vendor/idf/soc/esp32s3/include
    ${CMAKE_SOURCE_DIR}/vendor/idf/soc/esp32s3/register
    ${CMAKE_SOURCE_DIR}/vendor/idf/soc/include
    # esp_assert.h, esp_bit_defs.h, esp_attr.h stubs
    ${CMAKE_SOURCE_DIR}/boot2/idf_shims
    # Common vendor headers
    ${CMAKE_SOURCE_DIR}/vendor/esp_common
)

target_compile_options(touch_lcd PRIVATE
    -Os -g3 -Wall -Wextra
    -Wno-unused-parameter
    -ffunction-sections -fdata-sections
)

set(APP_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/app.ld)
target_link_options(touch_lcd PRIVATE
    -T ${APP_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/touch_lcd.map
    -Wl,--gc-sections
)
target_link_libraries(touch_lcd PRIVATE gcc c m)
set_property(TARGET touch_lcd APPEND PROPERTY LINK_DEPENDS ${APP_LINKER_SCRIPT})

set_target_properties(touch_lcd PROPERTIES
    OUTPUT_NAME "touch_lcd"
    SUFFIX ".elf"
)

set(TOUCH_LCD_BIN ${CMAKE_CURRENT_BINARY_DIR}/touch_lcd.bin)
set(PACK_APP_SCRIPT ${CMAKE_SOURCE_DIR}/scripts/pack_app.py)
add_custom_command(
    OUTPUT  ${TOUCH_LCD_BIN}
    COMMAND python3 ${PACK_APP_SCRIPT} $<TARGET_FILE:touch_lcd> ${TOUCH_LCD_BIN}
    DEPENDS touch_lcd ${PACK_APP_SCRIPT}
    COMMENT "[pack_app] touch_lcd.elf -> touch_lcd.bin"
    VERBATIM
)
add_custom_target(touch_lcd_bin ALL DEPENDS ${TOUCH_LCD_BIN})
