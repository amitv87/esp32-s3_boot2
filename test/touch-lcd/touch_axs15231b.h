#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t x;       /* 0..639 (touch panel X, landscape) */
    uint16_t y;       /* 0..171 (touch panel Y, landscape) */
    bool     pressed;
} touch_point_t;

void touch_init(void);
bool touch_read(touch_point_t *pt);
