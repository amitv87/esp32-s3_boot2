#pragma once
#include <stdint.h>
/* GPIO reservation tracking — harmless no-op for us. */
static inline void esp_gpio_reserve(uint64_t gpios) { (void)gpios; }
