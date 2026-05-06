#include "touch_axs15231b.h"
#include "board.h"
#include "hal_i2c.h"

/* Mirrors esp_lcd_axs15231b.c — the AXS15231B touch report is 6 bytes per
 * point preceded by 2 status bytes (gesture + count).  AXS_MAX_TOUCH_NUMBER
 * sets both the requested length in the read command (bytes [6:7] of cmd)
 * and the size of the response buffer. */
#define AXS_MAX_TOUCH_NUMBER 1

typedef struct {
    uint8_t gesture;    /* AXS_TOUCH_GESTURE_POS:0 */
    uint8_t num;        /* AXS_TOUCH_POINT_NUM:1   */
    uint8_t x_h : 4;    /* AXS_TOUCH_X_H_POS:2 (low nibble = high 4 bits of X) */
    uint8_t     : 2;
    uint8_t event : 2;  /* AXS_TOUCH_EVENT_POS:2 (bits 6-7) */
    uint8_t x_l;        /* AXS_TOUCH_X_L_POS:3 */
    uint8_t y_h : 4;    /* AXS_TOUCH_Y_H_POS:4 (low nibble = high 4 bits of Y) */
    uint8_t     : 4;
    uint8_t y_l;        /* AXS_TOUCH_Y_L_POS:5 */
} __attribute__((packed)) touch_record_struct_t;

static bb_i2c_t touch_bus;

void touch_init(void)
{
    bb_i2c_init(&touch_bus, TOUCH_PIN_SDA, TOUCH_PIN_SCL);
}

bool touch_read(touch_point_t *pt)
{
    static const uint8_t read_cmd[11] = {
        0xB5, 0xAB, 0xA5, 0x5A,
        0x00, 0x00,
        (AXS_MAX_TOUCH_NUMBER * 6 + 2) >> 8,
        (AXS_MAX_TOUCH_NUMBER * 6 + 2) & 0xFF,
        0x00, 0x00, 0x00,
    };
    uint8_t data[AXS_MAX_TOUCH_NUMBER * 6 + 2] = {0};

    if (!bb_i2c_write_read(&touch_bus, TOUCH_I2C_ADDR,
                           read_cmd, sizeof(read_cmd),
                           data, sizeof(data))) {
        pt->pressed = false;
        return false;
    }

    const touch_record_struct_t *p = (const touch_record_struct_t *)data;
    if (p->num && p->num <= AXS_MAX_TOUCH_NUMBER) {
        pt->x = ((uint16_t)p->x_h << 8) | p->x_l;
        pt->y = ((uint16_t)p->y_h << 8) | p->y_l;
        pt->pressed = true;
    } else {
        pt->pressed = false;
    }
    return true;
}
