/*
 * ESP32-S3 boot2 early entry.
 * ROM has already set SP, WindowBase/WindowStart, and copied our segments
 * to their load addresses. Zero .bss, then call into C.
 */

    .section .entry.text, "ax"
    .global call_start_cpu0
    .type   call_start_cpu0, @function
    .align  4
call_start_cpu0:
    /* Zero BSS: a2 = cursor, a3 = end, a4 = 0 */
    movi    a2, _bss_start
    movi    a3, _bss_end
    movi    a4, 0
    beq     a2, a3, 2f
1:  s32i    a4, a2, 0
    addi    a2, a2, 4
    bltu    a2, a3, 1b
2:
    /* Jump into C. boot2_main is __attribute__((noreturn)); if it ever
       returns, spin. */
    call4   boot2_main
    j       .
    .size   call_start_cpu0, . - call_start_cpu0
