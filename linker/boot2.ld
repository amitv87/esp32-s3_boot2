/*
 * ESP32-S3 boot2 linker script.
 *
 * Adapted from esp-idf bootloader.ld.in — we keep the IRAM/DRAM region
 * constants identical so we coexist with the ROM's reserved shared buffers
 * at 0x3fcd7e00..0x3fce9704, and so the 0x403CB700 SRAM_IRAM_END boundary
 * that IDF apps rely on is preserved.
 *
 * The section collapses the IDF's long KEEP(*libfoo.a:*) list into a single
 * catch-all for our small codebase.
 */

iram_dram_offset               = 0x6f0000;
bootloader_usable_dram_end     = 0x3fce9700;
bootloader_stack_overhead      = 0x2000;
bootloader_dram_seg_len        = 0x5000;
bootloader_iram_loader_seg_len = 0x7000;
bootloader_iram_seg_len        = 0x3000;

bootloader_dram_seg_end         = bootloader_usable_dram_end - bootloader_stack_overhead;
bootloader_dram_seg_start       = bootloader_dram_seg_end - bootloader_dram_seg_len;
bootloader_iram_loader_seg_start = bootloader_dram_seg_start - bootloader_iram_loader_seg_len + iram_dram_offset;
bootloader_iram_seg_start       = bootloader_iram_loader_seg_start - bootloader_iram_seg_len;

MEMORY
{
  iram_seg        (RWX) : org = bootloader_iram_seg_start,        len = bootloader_iram_seg_len
  iram_loader_seg (RWX) : org = bootloader_iram_loader_seg_start, len = bootloader_iram_loader_seg_len
  dram_seg        (RW)  : org = bootloader_dram_seg_start,        len = bootloader_dram_seg_len
}

ASSERT(bootloader_iram_loader_seg_start == 0x403CB700,
       "bootloader_iram_loader_seg_start inconsistent with SRAM_IRAM_END");

ENTRY(call_start_cpu0);

/* Explicit PHDRS so ld only emits PT_LOAD headers for the segments we
   actually want in the output binary. Without this, ld can emit a
   degenerate PT_LOAD for the DRAM memory region even when all its
   sections are empty, which confuses our image packer (it ends up
   wrapping the ELF header as segment payload). */
PHDRS
{
    iram_entry_seg PT_LOAD;
    iram_text_seg  PT_LOAD;
    dram_seg_phdr  PT_LOAD;
}

SECTIONS
{
  /* Entry stub lands at the very start of iram_seg so call_start_cpu0 has a
     predictable address for esp_image_header.entry_addr. */
  .iram.entry :
  {
    . = ALIGN(16);
    /* Xtensa l32r loads literals with negative PC-relative offset, so
       literals must precede their use in the output section. */
    *(.entry.literal)
    *(.init.literal)
    KEEP(*(.entry.text))
    *(.init)
  } > iram_seg :iram_entry_seg

  .iram.text :
  {
    . = ALIGN(16);
    _loader_text_start = ABSOLUTE(.);
    _text_start = ABSOLUTE(.);
    *(.literal .text .literal.* .text.*)
    *(.iram1 .iram1.*)
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)
    /* CPU prefetches up to 16 bytes past the last instruction; pad. */
    . += 16;
    _text_end = ABSOLUTE(.);
    _loader_text_end = ABSOLUTE(.);
  } > iram_loader_seg :iram_text_seg

  /* BSS is NOLOAD and has no phdr — it will never appear as a PT_LOAD. */
  .dram0.bss (NOLOAD) :
  {
    . = ALIGN(8);
    _dram_start = ABSOLUTE(.);
    _bss_start = ABSOLUTE(.);
    *(.dynsbss)
    *(.sbss .sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2 .sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(8);
    _bss_end = ABSOLUTE(.);
  } > dram_seg

  .dram0.data :
  {
    _data_start = ABSOLUTE(.);
    *(.dram1 .dram1.*)
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    *(.data1)
    *(.sdata .sdata.*)
    *(.gnu.linkonce.s.*)
    *(.gnu.linkonce.s2.*)
    *(.jcr)
    _data_end = ABSOLUTE(.);
  } > dram_seg :dram_seg_phdr

  .dram0.rodata :
  {
    _rodata_start = ABSOLUTE(.);
    *(.rodata .rodata.*)
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    *(.sdata2 .sdata2.*)
    *(.gnu.linkonce.lit4.*)
    *(.eh_frame_hdr)
    *(.eh_frame)
    . = ALIGN(4);
    _rodata_end = ABSOLUTE(.);
    _dram_end = ABSOLUTE(.);
  } > dram_seg :dram_seg_phdr

  /* Xtensa-specific informational sections — keep out of the image. */
  .xt.prop 0   : { KEEP(*(.xt.prop .gnu.linkonce.prop.*)) }
  .xt.lit  0   : { KEEP(*(.xt.lit  .gnu.linkonce.p.*))    }
  .xtensa.info 0 : { *(.xtensa.info) }

  /DISCARD/ :
  {
    *(.note.GNU-stack)
    *(.comment)
    *(.ARM.attributes)
  }
}
