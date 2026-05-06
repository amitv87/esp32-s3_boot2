/* Minimal newlib-nano reentrancy stub for bare-metal single-threaded use.
 *
 * libm_nano's wrapper functions (wf_sqrt, w_pow, w_acos, ...) set errno on
 * domain/range errors via __errno() → __getreent().  Since gui_lite calls
 * sqrtf/sinf/cosf and main.c calls sqrtf, the linker pulls in those
 * wrappers and demands __getreent.  Provide one global _reent struct — we
 * never read errno, so any valid pointer suffices. */
#include <reent.h>

static struct _reent s_reent = _REENT_INIT(s_reent);

struct _reent *__getreent(void)
{
    return &s_reent;
}
