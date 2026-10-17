/* Sink stalls (e.g. Bluetooth retransmissions) must not overflow the TinyUSB FIFO:
 * models the host sending 48 frames/ms into a 16-packet FIFO, the guarded USB
 * pump, the DSP, and a sink that stops draining for STALL_MS at a time. Output
 * must stay bit-identical to the engine (no dropped/duplicated frames). */
#include "../firmware/common/dh_pipeline.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define FIFO_CAP (16 * 49)
static lhp_engine E1, E2; static dhp_t P; static uint8_t blob[1 << 16];
int main(int argc, char **argv) {
    (void)argc; char fn[512]; snprintf(fn, sizeof fn, "%s/lhp_filter_dh1_small_48000.bin", argv[1]);
    FILE *f = fopen(fn, "rb"); size_t bl = fread(blob, 1, sizeof blob, f); fclose(f);
    dhp_filter_t F = {48000, 1, blob, (uint32_t)bl};
    dhp_init(&P, &E1, &F, 1, 48000, 1);
    lhp_init(&E2); int8_t m[6] = DHP_MAP_USB51; lhp_set_map(&E2, m, 6); lhp_load_filter(&E2, blob, bl, 1.0f);
    const int N = 48000 * 6; int16_t *src = malloc(N * 12), *ref = malloc(N * 4), *out = calloc(N + 8192, 4);
    srand(7); for (int i = 0; i < N * 6; i++) src[i] = (int16_t)(rand() % 12000 - 6000);
    for (int b = 0; b + 1024 <= N; b += 1024) lhp_block_s16(&E2, src + b * 6, ref + b * 2);
    int16_t *fifo = malloc(FIFO_CAP * 12); int fl = 0, fmax = 0, lost = 0;
    uint32_t sent = 0, outp = 0; int started = 0; int16_t tmp[64 * 2];
    for (int STALL_MS = 40, ms = 0; ms < 5800; ms++) {
        /* host: 48 frames/ms into the FIFO (drops if full = audible glitch) */
        for (int i = 0; i < 48 && sent < (uint32_t)N; i++, sent++) {
            if (fl < FIFO_CAP) memcpy(fifo + fl++ * 6, src + sent * 6, 12); else lost++;
        }
        if (fl > fmax) fmax = fl;
        /* USB pump (same policy as firmware) */
        uint32_t want = dhp_usb_frames_wanted_guarded(&P, fl, FIFO_CAP * 3 / 4, FIFO_CAP / 2);
        if (want > (uint32_t)fl) want = fl;
        uint32_t w = dhp_usb_write(&P, fifo, want); memmove(fifo, fifo + w * 6, (fl - w) * 12); fl -= w;
        dhp_dsp_poll(&P); dhp_dsp_poll(&P);
        /* sink: drains 48/ms, but stalls for STALL_MS every 700 ms, then catches up */
        static int debt = 0;
        int stalled = (ms % 700) < STALL_MS && ms > 500;
        if (!started && dhp_out_available(&P) >= 1024) started = 1;
        if (started) {
            debt += 48;
            if (!stalled) while (debt >= 48 && dhp_out_available(&P) >= 48) {
                dhp_read_stereo(&P, tmp, 48); memcpy(out + outp * 2, tmp, 48 * 4); outp += 48; debt -= 48; }
        }
    }
    int maxd = 0; uint32_t cmp = outp < (uint32_t)N - 2048 ? outp : (uint32_t)N - 2048;
    for (uint32_t i = 0; i < cmp * 2; i++) { int d = abs(out[i] - ref[i]); if (d > maxd) maxd = d; }
    printf("40 ms sink stalls every 700 ms: FIFO peak %d/%d frames, frames lost at USB %d, "
           "in_overruns %u, max |out - engine| %d over %u frames\n", fmax, FIFO_CAP, lost, P.in_overruns, maxd, cmp);
    return (lost == 0 && maxd == 0) ? 0 : 1;
}
