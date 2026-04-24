/*
 * Linker script for the hello-world RAM app.
 *
 * Link address must match APP_DEST_ADDR in main.c. Boot2 streams the
 * whole output binary into that address byte-for-byte and then calls
 * the `entry` field of the header, so absolute addresses in .text must
 * resolve correctly at that load location.
 */

MEMORY
{
    /* DBUS alias of SRAM1. See comment on APP_DEST_ADDR in main.c for
       why code+rodata lives here instead of at the IBUS alias. */
    sram (RWX) : ORIGIN = 0x3FC90000, LENGTH = 0x8000   /* 32 KB */
}

/* No ENTRY() — boot2 uses the header's `entry` field, not the ELF
   entry point. ld warns if neither is set, so point it at a symbol
   we know exists. */
ENTRY(__app_hdr)

SECTIONS
{
    .image : ALIGN(4)
    {
        __app_image_start = .;

        /* Header struct must be first — boot2 reads offset 0..15 as
           the app_image_header_t and uses .dest_addr/.size/.entry. */
        KEEP(*(.app_header))

        /* Xtensa literal pools must precede their use in the output
           section. */
        *(.literal .literal.*)
        *(.text    .text.*)
        *(.rodata  .rodata.*)
        *(.data    .data.*)
        *(.sdata   .sdata.*)
        *(.sdata2  .sdata2.*)

        . = ALIGN(4);
        __app_image_end = .;
    } > sram

    /* Export image size as an "address" — main.c casts its value into
       the header's .size field. */
    __app_image_size_sym = ABSOLUTE(__app_image_end - __app_image_start);

    /* BSS allowed but must be small and we never zero it — so just
       keep it empty by construction. Any zero-init data via a global
       would end up here and silently break. */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > sram

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
