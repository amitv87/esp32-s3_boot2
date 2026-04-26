#include "touch_axs15231b.h"
#include "board.h"
#include "hal_i2c.h"

static bb_i2c_t touch_bus;

void touch_init(void)
{
    bb_i2c_init(&touch_bus, TOUCH_PIN_SDA, TOUCH_PIN_SCL);
}

bool touch_read(touch_point_t *pt)
{
    static const uint8_t cmd[11] = {
        0xB5, 0xAB, 0xA5, 0x5A,
        0x00, 0x00, 0x00, 0x0E,
        0x00, 0x00, 0x00
    };
    uint8_t data[14] = {0};

    if (!bb_i2c_write_read(&touch_bus, TOUCH_I2C_ADDR, cmd, 11, data, 14)) {
        pt->pressed = false;
        return false;
    }

    if (data[1] > 0 && data[1] < 5) {
        pt->x = (uint16_t)(((data[2] & 0x0F) << 8) | data[3]);
        pt->y = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);
        pt->pressed = true;
    } else {
        pt->pressed = false;
    }
    return true;
}
