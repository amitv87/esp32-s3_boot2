/*
 * Linker script for hello-world-psram. Links at 0x3C000000, the DBUS
 * alias of PSRAM that boot2 has already MMU-mapped during startup.
 * Boot2 translates DBUS->IBUS before calling entry (see dbus_to_ibus
 * in boot2/loader.c), so code execution goes through IBUS 0x42000000.
 */

MEMORY
{
    psram (RWX) : ORIGIN = 0x3C000000, LENGTH = 0x100000   /* 1 MB window */
}

ENTRY(__app_hdr)

SECTIONS
{
    .image : ALIGN(4)
    {
        __app_image_start = .;

        KEEP(*(.app_header))

        *(.literal .literal.*)
        *(.text    .text.*)
        *(.rodata  .rodata.*)
        *(.data    .data.*)
        *(.sdata   .sdata.*)
        *(.sdata2  .sdata2.*)

        . = ALIGN(4);
        __app_image_end = .;
    } > psram

    __app_image_size_sym = ABSOLUTE(__app_image_end - __app_image_start);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > psram

    ASSERT(_bss_end == _bss_start,
           "hello-world-psram must have empty .bss (no zero-init globals)")

    /DISCARD/ :
    {
        *(.comment)
        *(.note .note.*)
        *(.eh_frame .eh_frame_hdr)
        *(.xt.prop .xt.lit)
        *(.xtensa.info)
    }
}
