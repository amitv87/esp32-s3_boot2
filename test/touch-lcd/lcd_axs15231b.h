#pragma once
#include <stdint.h>
#include "board.h"

void lcd_init(void);
/* Blit a full LCD_WIDTH_NATIVE × LCD_HEIGHT_NATIVE framebuffer (RGB565, big-endian) */
void lcd_blit_frame(const uint16_t *fb);
/* Fill entire display with one color */
void lcd_fill(uint16_t color);
