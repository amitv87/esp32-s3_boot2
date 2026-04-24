#pragma once
#define ESP_STATIC_ASSERT(c, m) _Static_assert(c, m)
#include <assert.h>
#undef assert
#define assert(x) ((void)(x))
