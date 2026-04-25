#!/usr/bin/env python3
"""
Pack a PSRAM app ELF into a flat binary for boot2 to load.

Uses p_paddr (LMA) — not p_vaddr (VMA) — as each segment's destination
address. Required for apps built with a VMA/LMA-split linker script where
code has IBUS VMAs (0x42......) but DBUS LMAs (0x3C......). All PT_LOAD
segments must have contiguous LMAs (any gap is zero-filled). The output is
a flat binary spanning exactly the LMA range; the loader writes it to DBUS
starting at the address embedded in the app_image_header_t.

Usage: pack_app.py <input.elf> <output.bin>
"""

import struct
import sys
from pathlib import Path

ELF_HEADER_FMT   = "<16sHHIIIIIHHHHHH"
PROG_HEADER_FMT  = "<IIIIIIII"
PT_LOAD = 1


def read_load_segments(elf: bytes):
    if elf[:4] != b"\x7fELF":
        raise ValueError("not an ELF file")
    fields = struct.unpack(ELF_HEADER_FMT, elf[:struct.calcsize(ELF_HEADER_FMT)])
    e_entry, e_phoff, e_phentsize, e_phnum = fields[4], fields[5], fields[9], fields[10]

    segs = []
    ph_size = struct.calcsize(PROG_HEADER_FMT)
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_flags, p_align = \
            struct.unpack(PROG_HEADER_FMT, elf[off:off + ph_size])
        if p_type != PT_LOAD or p_filesz == 0:
            continue
        segs.append((p_paddr, p_vaddr, elf[p_offset:p_offset + p_filesz]))

    return e_entry, segs


def pack_app(elf_path: Path, out_path: Path) -> None:
    elf = elf_path.read_bytes()
    e_entry, segs = read_load_segments(elf)
    if not segs:
        raise RuntimeError("no PT_LOAD segments in ELF")

    segs.sort(key=lambda s: s[0])   # sort by LMA

    base = segs[0][0]
    end  = max(lma + len(data) for lma, _, data in segs)
    buf  = bytearray(end - base)    # zero-fills any gaps

    for lma, vma, data in segs:
        buf[lma - base : lma - base + len(data)] = data
        print(f"  seg  LMA=0x{lma:08x}  VMA=0x{vma:08x}  {len(data)} bytes")

    # Pad to 4-byte alignment (mirrors ". = ALIGN(4)" at end of .data;
    # p_filesz omits trailing section padding so the flat binary may be
    # shorter than the linker counter without this step).
    pad = (-len(buf)) & 3
    buf += bytearray(pad)

    # Verify the size field in app_image_header_t (offset 4) matches.
    # The linker computes it from its section counter; it should equal the
    # padded binary length. Warn on mismatch rather than silently patching.
    header_size = struct.unpack_from("<I", buf, 4)[0]
    if header_size != len(buf):
        print(f"  WARNING: header.size=0x{header_size:x} != binary size=0x{len(buf):x}",
              file=sys.stderr)

    out_path.write_bytes(bytes(buf))
    print(f"[pack_app] {out_path.name}: "
          f"base=0x{base:08x}  entry=0x{e_entry:08x}  "
          f"{len(buf)} bytes total")


def main(argv):
    if len(argv) != 3:
        print(f"usage: {argv[0]} <input.elf> <output.bin>", file=sys.stderr)
        return 2
    pack_app(Path(argv[1]), Path(argv[2]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
