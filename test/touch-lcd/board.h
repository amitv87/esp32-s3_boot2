#pragma once

#include <stdio.h>

/* Waveshare ESP32-S3-Touch-LCD-3.49 pin assignments */

/* QSPI LCD (AXS15231B) */
#define LCD_PIN_CS      9   /* GPIO9  - manual CS (GPIO output) */
#define LCD_PIN_CLK     10  /* GPIO10 - SPI3 CLK */
#define LCD_PIN_DATA0   11  /* GPIO11 - SPI3 D0 (MOSI) */
#define LCD_PIN_DATA1   12  /* GPIO12 - SPI3 D1 (MISO/quad) */
#define LCD_PIN_DATA2   13  /* GPIO13 - SPI3 D2 (HD/quad) */
#define LCD_PIN_DATA3   14  /* GPIO14 - SPI3 D3 (WP/quad) */
#define LCD_PIN_RST     21  /* GPIO21 - LCD RESET */
#define LCD_PIN_BL      8   /* GPIO8  - LCD backlight */

/* Touch (AXS15231B I2C, bit-banged) */
#define TOUCH_PIN_SDA   17  /* GPIO17 - TP_SDA */
#define TOUCH_PIN_SCL   18  /* GPIO18 - TP_SCL */
#define TOUCH_I2C_ADDR  0x3B

/* Display geometry (native, portrait-ish: 172 wide x 640 tall) */
#define LCD_WIDTH_NATIVE  172
#define LCD_HEIGHT_NATIVE 640

/* Landscape rotation via software: effective 640x172 */
#define LCD_WIDTH   640
#define LCD_HEIGHT  172
