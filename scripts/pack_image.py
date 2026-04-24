#!/usr/bin/env python3
"""
Pack an ELF file into an ESP32-S3-flashable image with the ROM-expected
esp_image_header_t layout.

Layout:
    esp_image_header_t   (24 bytes, magic 0xE9)
    for each segment:
        esp_image_segment_header_t (load_addr, data_len)  8 bytes
        payload (padded to 4 bytes)
    padding so checksum lands at (N*16 - 1)
    checksum byte (XOR seed 0xEF over all segment bytes)

Reads LOAD program headers from the ELF. No esptool dependency.

Usage: pack_image.py <input.elf> <output.bin>
"""

import struct
import sys
from pathlib import Path


# ---- ELF parsing (LSB, 32-bit; sufficient for Xtensa) ----
EI_NIDENT = 16
ELF_HEADER_FMT = "<16sHHIIIIIHHHHHH"
PROGRAM_HEADER_FMT = "<IIIIIIII"
PT_LOAD = 1


def read_elf_load_segments(elf_bytes: bytes):
    if elf_bytes[:4] != b"\x7fELF":
        raise ValueError("not an ELF file")
    fields = struct.unpack(ELF_HEADER_FMT, elf_bytes[: struct.calcsize(ELF_HEADER_FMT)])
    # fields: (e_ident, e_type, e_machine, e_version, e_entry, e_phoff,
    #          e_shoff, e_flags, e_ehsize, e_phentsize, e_phnum,
    #          e_shentsize, e_shnum, e_shstrndx)
    e_entry     = fields[4]
    e_phoff     = fields[5]
    e_phentsize = fields[9]
    e_phnum     = fields[10]

    segments = []
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        (p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz,
         p_flags, _p_align) = struct.unpack(
            PROGRAM_HEADER_FMT, elf_bytes[off: off + struct.calcsize(PROGRAM_HEADER_FMT)])
        if p_type != PT_LOAD or p_filesz == 0:
            continue
        data = elf_bytes[p_offset: p_offset + p_filesz]
        segments.append((p_vaddr, data))
    return e_entry, segments


# ---- ESP image layout ----
ESP_IMAGE_MAGIC = 0xE9
ESP_CHIP_ID_ESP32S3 = 0x0009
ESP_CHECKSUM_SEED = 0xEF

IMAGE_HEADER_FMT = "<BBBBIBBBBHBHHI B"
# magic (B), segment_count (B), spi_mode (B), spi_speed/size (B),
# entry_addr (I), wp_pin (B), spi_pin_drv[3] (BBB),
# chip_id (H), min_chip_rev (B), min_chip_rev_full (H), max_chip_rev_full (H),
# reserved (I), hash_appended (B)  -- total 24 bytes


def pack_image_header(segment_count: int, entry_addr: int) -> bytes:
    return struct.pack(
        IMAGE_HEADER_FMT,
        ESP_IMAGE_MAGIC,         # magic
        segment_count,           # segment_count
        0x00,                    # spi_mode = QIO (matches vendor firmware)
        0x4F,                    # spi_speed/size = 80MHz / 16MB
                                 # (upper nibble 0x4 = 16MB, lower nibble 0xF = 80MHz DIV_1)
        entry_addr,              # entry_addr
        0xEE,                    # wp_pin = disabled
        0, 0, 0,                 # spi_pin_drv[3]
        ESP_CHIP_ID_ESP32S3,     # chip_id
        0,                       # min_chip_rev
        0,                       # min_chip_rev_full
        0xFFFF,                  # max_chip_rev_full (no upper bound)
        0,                       # reserved
        0,                       # hash_appended = no
    )


def pack_image(elf_path: Path, out_path: Path) -> None:
    elf_bytes = elf_path.read_bytes()
    entry_addr, segments = read_elf_load_segments(elf_bytes)
    if not segments:
        raise RuntimeError("no PT_LOAD segments in ELF")

    # Build segments: each one is (load_addr_u32, size_u32, data-padded-to-4).
    segment_blobs = []
    for vaddr, data in segments:
        pad = (-len(data)) & 3
        data_padded = data + (b"\x00" * pad)
        segment_blobs.append(
            struct.pack("<II", vaddr, len(data_padded)) + data_padded
        )

    body = b"".join(segment_blobs)
    header = pack_image_header(len(segments), entry_addr)

    # Checksum is XOR seed 0xEF over the PAYLOAD bytes of every segment
    # (not headers). It is placed at the end, padded so the checksum byte
    # itself sits at (N*16 - 1).
    checksum = ESP_CHECKSUM_SEED
    for _, data in segments:
        for b in data:
            checksum ^= b
        # Payload is padded for size; pad bytes are zeros so XOR is no-op.

    image = header + body
    # Pad so final image ends on 16-byte boundary (after checksum byte).
    pad_len = 15 - (len(image) % 16)
    image += b"\x00" * pad_len
    image += bytes([checksum & 0xFF])

    out_path.write_bytes(image)

    print(
        f"[pack] {out_path.name}: entry=0x{entry_addr:08x}, "
        f"{len(segments)} segs, {len(image)} bytes, cksum=0x{checksum:02x}"
    )


def main(argv):
    if len(argv) != 3:
        print(f"usage: {argv[0]} <input.elf> <output.bin>", file=sys.stderr)
        return 2
    pack_image(Path(argv[1]), Path(argv[2]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
