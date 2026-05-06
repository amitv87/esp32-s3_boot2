#pragma once
#include <stdint.h>
#include "board.h"
#include "gui_types.h"

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

/* Partial-region blit.  `buffer` points at column x, row y in a contiguous
 * surface of `stride` pixels per row (RGB565 byte-swapped).
 *
 * AXS15231B QSPI mode has CASET (X-window) but no RASET — RAMWR resets the
 * Y-cursor to 0.  This call sends rows 0..y+h-1 of x..x+w-1, sourcing rows
 * above the AABB from the surface (which already matches the panel state),
 * so writing them is a visual no-op. */
void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
              uint16_t stride, const uint16_t *buffer);

/* Compute the AABB of all dirty rects in `surface` and lcd_blit it. */
void lcd_blit_dma2d(gui_surface_t *surface);
