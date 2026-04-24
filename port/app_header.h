#include <stdint.h>

typedef struct{
  int (*printf)(const char* fmt, ...);
  void (*put_bytes)(uint8_t* buff, size_t length);
  uint64_t (*micros_now)();
} app_context_t;

typedef struct{
  uint32_t magic;
  uint32_t size;
  uint32_t dest_addr;
  void (*entry)(app_context_t* context);
  uint8_t bytes[0];
} app_image_header_t;
