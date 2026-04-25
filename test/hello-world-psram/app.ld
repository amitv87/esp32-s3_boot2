/*
 * Linker script for hello-world-psram.
 *
 * VMA/LMA split so indirect calls through function pointers work:
 *   .literal / .text  →  VMA = IBUS (0x42......), LMA = DBUS (0x3C......)
 *   .rodata / .data   →  VMA = LMA = DBUS (0x3C......)
 *
 * IBUS and DBUS are aliases of the same physical PSRAM pages. The loader
 * writes everything to contiguous DBUS addresses starting at 0x3C000000
 * (using LMA, not VMA). The cache exposes those same bytes for instruction
 * fetch at the IBUS alias. Function pointers therefore hold 0x42......
 * addresses and indirect calls (CALLX8) fetch correctly from IBUS.
 *
 * No sizes are specified upfront — the linker fills ibus with code and
 * dbus with data organically.
 *
 * Companion changes required:
 *   - binary packer:   use p_paddr (LMA) not p_vaddr as segment destination
 *   - boot2/loader.c:  entry in app_image_header_t is now an IBUS address;
 *                      skip dbus_to_ibus() conversion
 */

MEMORY
{
    ibus (RX) : ORIGIN = 0x42000000, LENGTH = 0x800000   /* PSRAM instruction window */
    dbus (RW) : ORIGIN = 0x3C000000, LENGTH = 0x800000   /* PSRAM data window        */
}

ENTRY(__app_hdr)

SECTIONS
{
    /* ---- App header + code: VMA = IBUS, LMA = DBUS ------------------- */
    /* All code sections (including the header) go > ibus AT> dbus.
     *
     * The critical invariant: VMA - LMA must equal exactly
     * IBUS_BASE - DBUS_BASE = 0x06000000 for every byte in the code
     * region, so that IBUS VMA 0x42...... and DBUS LMA 0x3C...... refer
     * to the same physical PSRAM page.  If the header were placed in
     * the dbus region first, the dbus LMA counter would advance by 16
     * before the ibus VMA counter starts, breaking the invariant and
     * causing L32R to fetch app-header bytes instead of literals.
     *
     * boot2 reads the header from DBUS (app_hdr->dest_addr = 0x3C000000),
     * which is the LMA — the bytes the loader wrote there.  entry holds
     * an IBUS address; dbus_to_ibus() passes it through unchanged. */
    .app_header : ALIGN(4)
    {
        KEEP(*(.app_header))
    } > ibus AT> dbus

    .literal : { *(.literal .literal.*) } > ibus AT> dbus
    .text    : { *(.text    .text.*)    } > ibus AT> dbus

    /* ---- Data: VMA = LMA = DBUS -------------------------------------- */
    /* Placed right after the code's LMA in the dbus region, no gap. */
    .rodata  : { *(.rodata  .rodata.*)  } > dbus
    .sdata2  : { *(.sdata2  .sdata2.*)  } > dbus
    .data    :
    {
        *(.data   .data.*)
        *(.sdata  .sdata.*)
        . = ALIGN(4);
    } > dbus

    /* Image size = all bytes the loader must receive, measured in DBUS
       space from ORIGIN(dbus) to end of last loaded section. */
    __app_image_size_sym = ABSOLUTE(. - ORIGIN(dbus));

    /* ---- BSS (NOLOAD — not in image, zeroed by app startup code) ----- */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss  .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > dbus

    /DISCARD/ :
    {
        *(.comment)
        *(.note .note.*)
        *(.eh_frame .eh_frame_hdr)
        *(.xt.prop .xt.lit)
        *(.xtensa.info)
    }
}
