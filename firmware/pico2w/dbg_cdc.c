/*
 * dbg_cdc - printf over the debug build's USB serial port (LHP_DEBUG_CDC=1).
 *
 * printf may run on core 0's main loop, in BTstack's background IRQ or on
 * core 1, but TinyUSB may only be touched from the tud_task() context. So
 * printf only copies into a ring buffer (under a spin lock), and
 * dbg_cdc_task() in the main loop moves it to the CDC port.
 */
#if LHP_DEBUG_CDC
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio/driver.h"
#include "hardware/sync.h"
#include "tusb.h"
#include "dbg_cdc.h"

#define RING 8192         /* 8 KB: RAM is tight with 7.1 */
static char s_ring[RING];
static volatile uint32_t s_w, s_r, s_dropped;
static spin_lock_t *s_lock;

static void out_chars(const char *buf, int len)
{
    uint32_t irq = spin_lock_blocking(s_lock);
    for (int i = 0; i < len; i++) {
        if (s_w - s_r >= RING) { s_dropped++; continue; }
        s_ring[s_w % RING] = buf[i];
        s_w++;
    }
    spin_unlock(s_lock, irq);
}

static stdio_driver_t s_drv = {
    .out_chars = out_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = true,
#endif
};

void dbg_cdc_init(void)
{
    s_lock = spin_lock_init(spin_lock_claim_unused(true));
    stdio_set_driver_enabled(&s_drv, true);
}

void dbg_cdc_task(void)
{
    /* Don't wait for DTR: some terminals never set it. If the port isn't open,
     * TinyUSB's own buffer overwrites old text, so the newest lines win. */
    if (!tud_mounted()) return;
    for (;;) {
        uint32_t r = s_r, w = s_w;
        if (r == w) break;
        uint32_t n = w - r, room = tud_cdc_write_available();
        uint32_t contig = RING - (r % RING);
        if (n > contig) n = contig;
        if (n > room) n = room;
        if (!n) break;
        tud_cdc_write(&s_ring[r % RING], n);
        s_r = r + n;
    }
    tud_cdc_write_flush();
}

uint32_t dbg_cdc_dropped(void) { return s_dropped; }

/* printf-style log straight into the ring, bypassing stdio's mutex, so it is
 * safe from both the core 0 main loop and core 0 interrupt handlers. */
#include <stdarg.h>
#include <stdio.h>
void dbg_logf(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n >= (int)sizeof buf) n = sizeof buf - 1;
    /* match stdio's CRLF translation */
    for (int i = 0; i < n; i++) {
        if (buf[i] == '\n') out_chars("\r\n", 2);
        else out_chars(&buf[i], 1);
    }
}
#endif /* LHP_DEBUG_CDC */
