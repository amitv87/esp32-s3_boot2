/*
 * Linker script for the hello-world SRAM1 app.
 *
 * VMA/LMA split: code has IBUS VMAs so indirect calls through function
 * pointers work; data has DBUS VMAs for load/store.  Both windows map
 * the same physical SRAM1 bytes with a fixed offset of 0x6F0000:
 *
 *   SRAM1 DBUS base : 0x3FC90000  (where the loader writes bytes)
 *   SRAM1 IBUS base : 0x40380000  (where the CPU fetches instructions)
 *   IBUS - DBUS     : 0x006F0000
 *
 * Critical invariant: every code section must satisfy
 *   VMA - LMA == 0x006F0000
 * so that the IBUS VMA and the DBUS LMA address the same physical byte.
 * Achieved by placing all code (including the header) > sram_ibus AT> sram_dbus,
 * which keeps both region counters in lockstep from byte zero.
 *
 * boot2 reads the header from DBUS (the LMA); the entry field holds an
 * IBUS address; dbus_to_ibus() passes it through unchanged.
 *
 * Companion change: pack_app.py uses p_paddr (LMA) not p_vaddr as the
 * segment destination, producing a compact DBUS-addressed binary.
 */

MEMORY
{
    sram_ibus (RX) : ORIGIN = 0x40380000, LENGTH = 0x8000   /* SRAM1 instruction window */
    sram_dbus (RW) : ORIGIN = 0x3FC90000, LENGTH = 0x8000   /* SRAM1 data window        */
}

ENTRY(__app_hdr)

SECTIONS
{
    /* ---- Header + code: VMA = IBUS, LMA = DBUS ----------------------- */
    .app_header : ALIGN(4)
    {
        KEEP(*(.app_header))
    } > sram_ibus AT> sram_dbus

    .literal : { *(.literal .literal.*) } > sram_ibus AT> sram_dbus
    .text    : { *(.text    .text.*)    } > sram_ibus AT> sram_dbus

    /* ---- Data: VMA = LMA = DBUS -------------------------------------- */
    .rodata  : { *(.rodata  .rodata.*)  } > sram_dbus
    .sdata2  : { *(.sdata2  .sdata2.*)  } > sram_dbus
    .data    :
    {
        *(.data  .data.*)
        *(.sdata .sdata.*)
        . = ALIGN(4);
    } > sram_dbus

    __app_image_size_sym = ABSOLUTE(. - ORIGIN(sram_dbus));

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss  .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > sram_dbus

    ASSERT(_bss_end == _bss_start,
           "hello-world app must have empty .bss (no zero-init globals)")

    /DISCARD/ :
    {
        *(.comment)
        *(.note .note.*)
        *(.eh_frame .eh_frame_hdr)
        *(.xt.prop .xt.lit)
        *(.xtensa.info)
    }
}
