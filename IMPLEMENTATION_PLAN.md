# ESP32-S3 Boot2 — Standalone Minimal Second-Stage Bootloader Plan

## Context

The user has an existing BL61x second-stage bootloader at `/Volumes/boggy/repos/bl61x_boot2` that listens on UART/USB, receives an application image prefixed with a custom `app_image_header_t` (magic `0xDEADBEEF`), copies the payload into RAM at `dest_addr`, and jumps to `entry(context)`. Goal: build the equivalent for ESP32-S3 at `/Volumes/boggy/repos/esp32-s3_boot2`, independent of the ESP-IDF build system.

The ESP-IDF production bootloader at `/Volumes/awsm/rep/esp-idf/components/bootloader/subproject/main/bootloader_start.c` drags in partition tables, OTA, secure boot, flash encryption, deep-sleep wake paths, and image parsing — none of which are needed here. We need only:
- ROM bootloader → our boot2 (via standard ESP image header, `0xE9` magic)
- boot2 brings up the minimum runtime (clock/console are already good enough from ROM)
- Poll a configurable IO channel for a `0xDEADBEEF`-prefixed blob
- Copy payload to `dest_addr` (internal SRAM or PSRAM), invalidate ICache, call `entry(&ctx)`

Shared image header: `/Volumes/boggy/repos/bl61x_boot2/port/app_header.h` (copied into this project — **identical** layout, identical `app_context_t` callbacks).

Per user decisions:
- IO channels supported: **UART0** and **USB-Serial-JTAG** (ROM polling functions; no USB-OTG CDC)
- Channel is a **compile-time choice** (`BOOT2_IO=UART | USB_JTAG`), not runtime-arbitrated
- App image may target **internal SRAM or PSRAM** (PSRAM requires init + MMU map)
- **Use ROM functions** for UART and USB; no custom drivers
- Toolchain: `/Volumes/boggy/toolchain/xtensa-esp-elf-15.2.0_20251204` (GCC 15.2.0), prefix `xtensa-esp32s3-elf-`. This matches IDF's GCC approach in `tools/cmake/toolchain-esp32s3.cmake` — the prefix-specific binary has the ESP32-S3 LX7 dynconfig baked in, so no `-mcpu` flag is required.

---

## 1. Target architecture

ESP32-S3 is a dual-core **Xtensa LX7** (not RISC-V — this is a significant divergence from the BL61x project). Startup is Xtensa assembly; cache/MMU layout is Xtensa-specific.

- **Toolchain**: `/Volumes/boggy/toolchain/xtensa-esp-elf-15.2.0_20251204/bin/xtensa-esp32s3-elf-gcc`. CMake toolchain file honors `ESP32S3_TOOLCHAIN_PREFIX` env var with the above as default.
- **CPU at entry**: 40 MHz XTAL (ROM default). We'll bump to 80/160 MHz only if a higher baud is needed; otherwise leave as-is (polling UART/USB at ROM defaults works fine).
- **ROM default UART0**: 115200 8N1. `esp_rom_uart_rx_one_char` / `esp_rom_uart_tx_one_char` work immediately — no init.
- **USB-Serial-JTAG**: ROM functions `usb_uart_device_rx_one_char`, `usb_uart_device_tx_one_char`, `usb_uart_device_tx_flush`. Peripheral is live if the host has enumerated the device; no `Uart_Init_USB` required (that's for USB-OTG CDC, which we're not using).

---

## 2. ROM bootloader contract — what we must honor

ROM reads flash starting at offset `0x0` on ESP32-S3, expecting:

**`esp_image_header_t`** (24 bytes, from `/Volumes/awsm/rep/esp-idf/components/bootloader_support/include/esp_app_format.h`):

| off | size | field | value we use |
|---|---|---|---|
| 0 | 1 | magic | `0xE9` |
| 1 | 1 | segment_count | set by post-build packer |
| 2 | 1 | spi_mode | `0x02` (DIO) — safe default |
| 3 | 1 | spi_speed/size | `0x20` (40 MHz / 4 MB) — safe default |
| 4 | 4 | entry_addr | address of `call_start_cpu0` |
| 8 | 1 | wp_pin | `0xEE` (disabled) |
| 9 | 3 | spi_pin_drv | `{0,0,0}` |
| 12 | 2 | chip_id | `0x0009` (ESP32-S3) |
| 14 | 1 | min_chip_rev | `0` |
| 15..22 | | revs/reserved | zeros |
| 23 | 1 | hash_appended | `0` |

Each segment follows as `esp_image_segment_header_t { uint32_t load_addr; uint32_t data_len; }` + payload, padded to 4 bytes. After last segment: 1 byte XOR checksum (seed `0xEF`).

ROM copies segments to `load_addr` (must be IRAM `0x4037xxxx` or DRAM `0x3FCxxxxx`) and jumps to `entry_addr`. **boot2 is NOT XIP from flash** in this simple scheme — it's a RAM image. We'll put code and data in IRAM/DRAM regions that match the ESP-IDF bootloader layout so it coexists with the ROM's reserved shared buffers.

We must produce this image via a post-build packer (see §10).

---

## 3. Project structure

```
esp32-s3_boot2/
├── CMakeLists.txt                        # Top-level standalone CMake
├── IMPLEMENTATION_PLAN.md                # This file
├── README.md
│
├── cmake/
│   ├── toolchain-esp32s3.cmake           # Xtensa toolchain file
│   └── image_pack.cmake                  # post-build: pack ELF → .bin with 0xE9 header
│
├── boot2/
│   ├── main.c                            # Entry: bring-up + poll loop (adapted from bl61x main.c)
│   ├── io_uart.c                         # UART0 wrappers around ROM functions
│   ├── io_usb_jtag.c                     # USB-Serial-JTAG wrappers around ROM functions
│   ├── io.h                              # Unified io_getchar / io_putchar / io_rxavailable / io_printf
│   ├── loader.c                          # 0xDEADBEEF header parser, payload receive, cache flush, jump
│   ├── psram.c                           # Optional PSRAM init (compile-gated)
│   ├── start.S                           # Xtensa reset vector + early entry stub
│   └── rom_symbols.h                     # Prototypes for ROM functions we call
│
├── port/
│   ├── app_header.h                      # Verbatim copy of bl61x_boot2/port/app_header.h
│   └── memory_map.h                      # IRAM/DRAM/PSRAM region constants
│
├── linker/
│   └── boot2.ld                          # Adapted from bootloader.ld.in; ENTRY(call_start_cpu0)
│
├── vendor/                               # Minimal ESP-IDF files copied verbatim
│   ├── esp_rom/esp32s3/ld/
│   │   ├── esp32s3.rom.ld                # ROM function addresses
│   │   └── esp32s3.rom.api.ld            # esp_rom_* aliases
│   ├── soc/esp32s3/register/soc/
│   │   ├── uart_reg.h
│   │   ├── uart_struct.h
│   │   ├── usb_serial_jtag_reg.h
│   │   ├── usb_serial_jtag_struct.h
│   │   └── soc.h                         # DR_REG_* bases
│   └── esp_common/
│       └── attr.h                        # IRAM_ATTR / DRAM_ATTR only if needed
│
└── scripts/
    └── pack_image.py                     # Python: ELF → ESP image binary (magic 0xE9 + segments + checksum)
```

We deliberately **avoid** pulling in: `bootloader_support` (too tangled with IDF config), `esp_psram` system layer (we'll adapt only the octal-impl routine), `hal`, `log`, `spi_flash`. The project is tiny by design.

---

## 4. Build system

### 4.1 Toolchain file (`cmake/toolchain-esp32s3.cmake`)

Modelled after `/Volumes/awsm/rep/esp-idf/tools/cmake/toolchain-esp32s3.cmake` (which is literally `set(_CMAKE_TOOLCHAIN_PREFIX xtensa-esp32s3-elf-)` + shared `toolchain.cmake`). We inline the shared parts so the file stands alone.

```cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR xtensa)

if(NOT DEFINED ENV{ESP32S3_TOOLCHAIN_PREFIX})
  set(ENV{ESP32S3_TOOLCHAIN_PREFIX}
      "/Volumes/boggy/toolchain/xtensa-esp-elf-15.2.0_20251204/bin/xtensa-esp32s3-elf-")
endif()
set(TOOLCHAIN_PREFIX "$ENV{ESP32S3_TOOLCHAIN_PREFIX}")

set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_OBJDUMP      "${TOOLCHAIN_PREFIX}objdump")
set(CMAKE_SIZE         "${TOOLCHAIN_PREFIX}size")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Xtensa flags IDF uses in components/soc/project_include.cmake:
#   -mlongcalls               relaxes call ranges so the linker can reach ROM addresses
#   -fno-builtin-memcpy/memset/bzero   lets us reuse ROM's implementations without builtin-inline conflicts
# We additionally use -ffreestanding / -nostdlib since we have no libc startup.
set(CPU_FLAGS "-mlongcalls -fno-builtin-memcpy -fno-builtin-memset -fno-builtin-bzero")
set(CMAKE_C_FLAGS_INIT   "${CPU_FLAGS} -ffreestanding -fno-builtin")
set(CMAKE_ASM_FLAGS_INIT "${CPU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-nostartfiles -nostdlib -Wl,--gc-sections")
```

### 4.2 Top-level `CMakeLists.txt` highlights

- No `find_package`, no `idf_component_register`.
- `add_executable(boot2 boot2/*.c boot2/start.S)`; sources listed explicitly.
- Include dirs: `boot2/`, `port/`, `vendor/soc/esp32s3/register/soc`, `vendor/esp_common`.
- Linker inputs: `linker/boot2.ld`, `vendor/esp_rom/esp32s3/ld/esp32s3.rom.ld`, `vendor/esp_rom/esp32s3/ld/esp32s3.rom.api.ld`.
- Optimization: `-Os -g3`.
- Cache variables exposed:
  - `BOOT2_IO=UART | USB_JTAG` (default `UART`)
  - `BOOT2_PSRAM=OFF | OCTAL` (default `OFF`)
- Translate these into `-DBOOT2_IO_UART=1` / `-DBOOT2_IO_USB_JTAG=1` / `-DBOOT2_PSRAM_OCTAL=1` compile defs; CMake conditionally adds `io_uart.c` vs `io_usb_jtag.c` and `psram.c`.
- Post-build: `objcopy -O binary boot2.elf boot2.elf.bin` → `python scripts/pack_image.py boot2.elf.bin boot2.bin` (generates ESP-compatible image with `0xE9` header, segment headers, checksum).

### 4.3 Configure & build

```bash
cmake -B /Volumes/awsm/build/esp32-s3_boot2_8de19eb5d4f360be8d0d6268c16f7ce9 \
      -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-esp32s3.cmake \
      -DBOOT2_IO=UART \
      -DBOOT2_PSRAM=OFF \
      -S /Volumes/boggy/repos/esp32-s3_boot2
cmake --build /Volumes/awsm/build/esp32-s3_boot2_8de19eb5d4f360be8d0d6268c16f7ce9
```

Output artifacts in the build dir: `boot2.elf`, `boot2.bin` (flashable), `boot2.map`.

---

## 5. Linker script (`linker/boot2.ld`)

Adapted from `/Volumes/awsm/rep/esp-idf/components/bootloader/subproject/main/ld/esp32s3/bootloader.ld.in`, but simplified: we have no libbootloader_support, libhal, etc., so the massive KEEP(*libfoo.a:*) list collapses to a single code section.

Key constants (identical to IDF bootloader to stay coexistent with ROM shared buffers):

```
iram_dram_offset       = 0x6f0000
bootloader_usable_dram_end = 0x3fce9700
bootloader_dram_seg_len    = 0x5000
bootloader_iram_loader_seg_len = 0x7000    # kept for layout parity
bootloader_iram_seg_len    = 0x3000
```

Sections:
- `.entry.text` (at `0x40378000`-ish) — `start.S` reset/entry stub
- `.iram.text` — all code (`*(.text .text.* .literal .literal.*)`)
- `.dram0.bss` (NOLOAD) — zero-init
- `.dram0.data` — initialized data
- `.dram0.rodata` — read-only

`ENTRY(call_start_cpu0)` — referenced by the image header's `entry_addr`.

Include the ROM linker scripts as input:
```
INPUT(vendor/esp_rom/esp32s3/ld/esp32s3.rom.ld)
INPUT(vendor/esp_rom/esp32s3/ld/esp32s3.rom.api.ld)
```

---

## 6. Startup (`boot2/start.S`)

Minimal Xtensa stub — ROM has already:
- Set up initial stack in DRAM
- Placed our image segments at their load addresses
- Configured WindowBase / WindowStart for window-based ABI
- Jumped to `call_start_cpu0`

We just need:
1. Zero `.bss` from `_bss_start` to `_bss_end`.
2. Optionally re-set stack pointer to our own stack top (top of `dram_seg`).
3. Call C entry `boot2_main()`.

```asm
.section .entry.text, "ax"
.global call_start_cpu0
call_start_cpu0:
    /* Zero BSS */
    movi    a2, _bss_start
    movi    a3, _bss_end
    movi    a4, 0
1:  s32i    a4, a2, 0
    addi    a2, a2, 4
    bltu    a2, a3, 1b

    /* Jump to C main (never returns unless app exits) */
    call4   boot2_main
    j       .   /* spin if boot2_main ever returns */
```

(Exact Xtensa opcodes / window-ABI macros refined during implementation; this is the shape, not the final text.)

---

## 7. Core logic (`boot2/main.c`, `boot2/loader.c`)

Directly mirrors the BL61x `main.c`, minus Bouffalo-specific HAL and LED code. Only one poll loop is compiled in (per `BOOT2_IO`).

```c
// boot2/main.c
#include "app_header.h"
#include "io.h"
#include "loader.h"

void boot2_main(void) {
    io_init();                            // no-op for UART (ROM already init); triggers USB-JTAG flush
    io_printf("boot2 %s %s\r\n", __DATE__, __TIME__);

#if BOOT2_PSRAM_OCTAL
    psram_init();                         // only if compiled in
#endif

    loader_poll();                        // never returns in normal flow
    while (1) { }
}
```

```c
// boot2/loader.c  — the important bit
#define APP_MAGIC 0xDEADBEEFu

void loader_poll(void) {
    app_context_t ctx = {
        .printf = io_printf,
        .put_bytes = io_put_bytes,
        .micros_now = rom_micros_now,    // thin wrapper around esp_rom_get_cpu_ticks_per_us + systimer read
    };

    uint8_t hdr_buf[sizeof(app_image_header_t)];
    size_t bytes_read = 0;
    app_image_header_t *app_hdr = NULL;

    for (;;) {
        if (bytes_read < sizeof(hdr_buf)) {
            if (io_rxavailable()) {
                hdr_buf[bytes_read++] = io_getchar();
            }
            if (bytes_read == sizeof(hdr_buf)) {
                app_image_header_t *tmp = (app_image_header_t *)hdr_buf;
                if (tmp->magic == APP_MAGIC) {
                    app_hdr = (app_image_header_t *)tmp->dest_addr;
                    *app_hdr = *tmp;
                } else {
                    io_printf("bad magic=0x%x size=%u dest=0x%x\r\n",
                              tmp->magic, tmp->size, tmp->dest_addr);
                    bytes_read = 0;
                }
            }
        } else if (app_hdr && bytes_read < app_hdr->size) {
            if (io_rxavailable()) {
                app_hdr->bytes[bytes_read++ - sizeof(*app_hdr)] = io_getchar();
            }
            if (bytes_read == app_hdr->size) {
                io_printf("jump 0x%x\r\n", (unsigned)app_hdr->entry);
                Cache_Invalidate_ICache_All();   // ROM function
                app_hdr->entry(&ctx);
                io_printf("app exited\r\n");
                app_hdr = NULL; bytes_read = 0;
            }
        }
    }
}
```

Byte-at-a-time IO matches the BL61x reference verbatim. If throughput becomes painful (app images can be 100s of KB, at 115200 that's ~20s) we can revisit with a bulk-read variant — but not in v1.

---

## 8. IO layer (`boot2/io_uart.c`, `boot2/io_usb_jtag.c`)

Both files expose the same four functions:

```c
void   io_init(void);
bool   io_rxavailable(void);
uint8_t io_getchar(void);             // blocking
void   io_put_bytes(const uint8_t *buf, size_t len);
int    io_printf(const char *fmt, ...);  // uses ets_printf / local vsnprintf
```

### UART0 variant
Uses ROM `uart_tx_one_char` / `uart_rx_one_char` / `ets_printf`. ROM `uart_rx_one_char` is consume-on-read, so we keep a 1-byte peek cache to implement `io_rxavailable()` without losing data.

### USB-Serial-JTAG variant
Uses ROM `usb_uart_device_rx_one_char` / `usb_uart_device_tx_one_char` / `usb_uart_device_tx_flush`. Same 1-byte peek pattern. `io_put_bytes` calls `usb_uart_device_tx_flush()` at the end to push the endpoint packet. `ets_printf` goes through a shim via `esp_rom_install_channel_putc` so logs end up on USB too.

---

## 9. PSRAM (optional, `boot2/psram.c`)

Only compiled when `BOOT2_PSRAM=OCTAL`. Adapt the octal-PSRAM init from `/Volumes/awsm/rep/esp-idf/components/esp_psram/esp32s3/esp_psram_impl_octal.c`:

1. Configure MSPI pins for octal PSRAM (ROM-helper-driven).
2. Call ROM `esp_rom_opiflash_wait_idle` etc. to probe chip ID.
3. Map PSRAM into DBUS via `Cache_Dbus_MMU_Set` (ROM function).
4. Enable cache for that region.

For v1, keeping `BOOT2_PSRAM=OFF` is fine — the app itself can do PSRAM init if boot2 loads it to internal SRAM first.

---

## 10. Image packing (`scripts/pack_image.py`)

Standalone Python 3 script (no esptool dependency) — takes an ELF, emits the ESP-IDF-compatible image:

1. Parse ELF program headers to enumerate LOAD segments.
2. Write header with magic `0xE9`, `segment_count`, entry = ELF `e_entry`.
3. For each LOAD segment: write `{load_addr, size}` + padded payload.
4. Compute XOR checksum (seed `0xEF`) over all segment bytes; append checksum byte at 16-byte alignment.
5. Pad final image to multiple of 16 bytes.

Invoked by CMake via `image_pack.cmake`. Output: `boot2.bin`.

---

## 11. Files to copy verbatim from ESP-IDF

Into `vendor/`:

| From | To |
|---|---|
| `components/esp_rom/esp32s3/ld/esp32s3.rom.ld` | `vendor/esp_rom/esp32s3/ld/` |
| `components/esp_rom/esp32s3/ld/esp32s3.rom.api.ld` | same |
| `components/soc/esp32s3/register/soc/uart_reg.h` | `vendor/soc/esp32s3/register/soc/` |
| `components/soc/esp32s3/register/soc/uart_struct.h` | same |
| `components/soc/esp32s3/register/soc/usb_serial_jtag_reg.h` | same |
| `components/soc/esp32s3/register/soc/usb_serial_jtag_struct.h` | same |
| `components/soc/esp32s3/register/soc/soc.h` | same |

Into `port/`:
| From | To |
|---|---|
| `/Volumes/boggy/repos/bl61x_boot2/port/app_header.h` | `port/app_header.h` (unchanged) |

---

## 12. Phased execution roadmap

### Phase 1 — Scaffold & toolchain
1. Create directory structure.
2. Write `cmake/toolchain-esp32s3.cmake`.
3. Write skeleton `CMakeLists.txt` that just builds an empty `start.S` + `main.c` stub.
4. Validate: `cmake -B <builddir> ...` configures without error; `cmake --build` produces an ELF with `call_start_cpu0` symbol.

### Phase 2 — Vendor copy
1. `cp` the files listed in §11 into `vendor/`.
2. `cp` `app_header.h` into `port/`.
3. Validate: header includes resolve.

### Phase 3 — Linker + startup
1. Write `linker/boot2.ld` (adapted from `bootloader.ld.in`).
2. Write `boot2/start.S` (BSS zero + call `boot2_main`).
3. Add ROM linker scripts to link line.
4. Validate: ELF links; `objdump -d` shows `call_start_cpu0` at `0x40378000`-ish; no undefined symbols.

### Phase 4 — IO layer (UART first)
1. Write `boot2/rom_symbols.h` (prototypes with addresses from the `.rom.ld`).
2. Write `boot2/io_uart.c` + `boot2/io.h`.
3. Write minimal `boot2/main.c` that just prints `"boot2 hello\r\n"` and spins.
4. Write `scripts/pack_image.py`; wire into CMake post-build.
5. **Hardware validation**: flash `boot2.bin` to offset `0x0`, watch UART0 at 115200 for the banner.

### Phase 5 — USB-Serial-JTAG variant
1. Write `boot2/io_usb_jtag.c` (ROM `usb_uart_device_*` calls).
2. Reconfigure with `-DBOOT2_IO=USB_JTAG`; verify banner appears on USB CDC.

### Phase 6 — Loader logic
1. Write `boot2/loader.c` with the `0xDEADBEEF` poll loop.
2. Build a tiny test app using `app_image_header_t` that prints via `ctx->printf` and returns.
3. Send it over UART; confirm boot2 parses header, copies payload, jumps, and returns cleanly.
4. Repeat for USB-JTAG variant.

### Phase 7 — PSRAM (optional, defer)
1. Add `boot2/psram.c` adapted from `esp_psram_impl_octal.c`.
2. Gate behind `BOOT2_PSRAM=OCTAL`.
3. Test with an app whose `dest_addr` is in the mapped PSRAM range.

---

## 13. Critical files — quick reference

| File | Role |
|---|---|
| `/Volumes/boggy/repos/bl61x_boot2/boot2/main.c` | Logic reference — mirror its poll loop exactly |
| `/Volumes/boggy/repos/bl61x_boot2/port/app_header.h` | Copy verbatim into `port/` |
| `/Volumes/boggy/repos/bl61x_boot2/IMPLEMENTATION_PLAN.md` | Workflow reference (copy-first principle) |
| `/Volumes/awsm/rep/esp-idf/tools/cmake/toolchain-esp32s3.cmake` | IDF GCC toolchain — the pattern we mirror |
| `/Volumes/awsm/rep/esp-idf/tools/cmake/toolchain.cmake` | Shared IDF toolchain init (C/ASM compiler setup) |
| `/Volumes/awsm/rep/esp-idf/components/soc/project_include.cmake` | Source of Xtensa compile flags (`-mlongcalls`, etc.) |
| `/Volumes/awsm/rep/esp-idf/components/bootloader/subproject/main/bootloader_start.c` | **Do not copy** — reference only |
| `/Volumes/awsm/rep/esp-idf/components/bootloader/subproject/main/ld/esp32s3/bootloader.ld.in` | Template for our `linker/boot2.ld` |
| `/Volumes/awsm/rep/esp-idf/components/bootloader_support/include/esp_app_format.h` | `esp_image_header_t` layout (we replicate, don't include) |
| `/Volumes/awsm/rep/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.ld` | ROM function addresses — **copy verbatim** |
| `/Volumes/awsm/rep/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.api.ld` | `esp_rom_*` alias symbols — copy verbatim |
| `/Volumes/awsm/rep/esp-idf/components/esp_rom/esp32s3/include/esp32s3/rom/uart.h` | Prototype reference for UART ROM funcs |
| `/Volumes/awsm/rep/esp-idf/components/esp_psram/esp32s3/esp_psram_impl_octal.c` | PSRAM reference for phase 7 |

---

## 14. Verification

### Build verification
```bash
cmake -B /Volumes/awsm/build/esp32-s3_boot2_8de19eb5d4f360be8d0d6268c16f7ce9 \
      -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-esp32s3.cmake \
      -S /Volumes/boggy/repos/esp32-s3_boot2
cmake --build /Volumes/awsm/build/esp32-s3_boot2_8de19eb5d4f360be8d0d6268c16f7ce9
```

Expected artifacts in build dir:
- `boot2.elf` — links cleanly, no undefined symbols
- `boot2.bin` — starts with `0xE9`, size < 16 KB
- `boot2.map` — IRAM usage < 12 KB, DRAM usage < 5 KB

### Flash + runtime verification
```bash
esptool.py --chip esp32s3 --port /dev/cu.usbmodem* write_flash 0x0 boot2.bin
# Watch UART0 @ 115200 (or USB-CDC depending on BOOT2_IO):
#   "boot2 <date> <time>" banner
```

End-to-end with a test app:
1. Build a minimal test app: just a function that calls `ctx->printf("hello from app\r\n")` and returns, prefixed with `app_image_header_t { .magic=0xDEADBEEF, .size=..., .dest_addr=0x3FC90000, .entry=... }`.
2. `cat test_app.bin > /dev/cu.usbmodem*` (or equivalent sender).
3. Expected log: banner → `jump 0x3FC90000` → `hello from app` → `app exited`.

### Smoke tests
- **Magic mismatch**: send 4 garbage bytes; boot2 prints `bad magic=...` and resets buffer (no restart needed).
- **Partial image**: disconnect mid-upload; re-sending from start resumes cleanly.
- **Compile matrix**: both `BOOT2_IO=UART` and `BOOT2_IO=USB_JTAG` variants build.

---

## 15. Risks & mitigations

| Risk | Mitigation |
|---|---|
| Xtensa window-ABI mistakes in `start.S` | Keep start.S to the absolute minimum (BSS zero + `call4`); rely on ROM's pre-set stack |
| ROM function signature drift across chip revisions | Prototypes in `rom_symbols.h`; symbols resolved via the copied linker script |
| Image layout assertion failure (`bootloader_iram_loader_seg_start != 0x403CB700`) | Match IDF's layout constants exactly |
| USB-JTAG host not enumerated at boot2 start → early prints lost | Accept first byte may drop; banner re-prints after first RX |
| PSRAM init fragility when copied out of IDF | Defer to phase 7 |
| `esptool`-free image packing bugs | Cross-validate `boot2.bin` against `esptool.py image_info` during phase 4 |
