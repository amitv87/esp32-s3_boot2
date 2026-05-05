#pragma once
#include <stdint.h>
#include "board.h"

/* Pass ctx->micros_now for accurate hardware-timed delays.
 * Without it, busy-loop calibration is CPU-frequency dependent. */
typedef uint64_t (*lcd_micros_fn_t)(void);

void lcd_init(lcd_micros_fn_t micros_now);
/* Blit a full LCD_WIDTH_NATIVE × LCD_HEIGHT_NATIVE framebuffer (RGB565, big-endian) */
void lcd_blit_frame(const uint16_t *fb);
/* Fill entire display with one color */
void lcd_fill(uint16_t color);
/* Read panel ID registers (DAh/DBh/DCh) over QSPI and print results.
 * Datasheet defaults: DAh→0x40, DBh→0x00, DCh→0x03.  Use to verify
 * QSPI wire format is correct in both directions. */
void lcd_read_id(void);
