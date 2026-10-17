/* Rooms -> low-latency stereo -> rooms, with a modelled USB FIFO and a steady
 * 48 frames/ms sink. Reports buffered latency per phase, checks that the
 * low-latency output is an exact (gain 0.5) downmix of the input with a fixed
 * delay, that nothing is lost at USB, and that rooms mode resumes cleanly. */
#include "../firmware/common/dh_pipeline.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define FIFO_CAP (16 * 49)
static lhp_engine E; static dhp_t P; static uint8_t blob[1 << 16];
int main(int argc, char **argv) {
    (void)argc; char fn[512]; snprintf(fn, sizeof fn, "%s/lhp_filter_dh1_small_48000.bin", argv[1]);
    FILE *f = fopen(fn, "rb"); size_t bl = fread(blob, 1, sizeof blob, f); fclose(f);
    dhp_filter_t F = {48000, 1, blob, (uint32_t)bl};
    dhp_init(&P, &E, &F, 1, 48000, 1);
    const int N = 48000 * 7; int16_t *src = malloc(N * 12); int16_t *out = calloc(N + 48000, 4);
    srand(9); for (int i = 0; i < N * 6; i++) src[i] = (int16_t)(rand() % 8000 - 4000);
    int16_t *fifo = malloc(FIFO_CAP * 12); int fl = 0, lost = 0; uint32_t sent = 0, outp = 0;
    double fill_sum[3] = {0}; int fill_n[3] = {0}; uint32_t ur_phase[3] = {0}, ur0 = 0;
    int16_t tmp[48 * 2]; int started = 0;
    for (int ms = 0; ms < 6900; ms++) {
        int phase = ms < 2300 ? 0 : ms < 4600 ? 1 : 2;
        if (ms == 2300) { dhp_set_low_latency(&P, true); ur_phase[0] = P.out_underruns - ur0; ur0 = P.out_underruns; }
        if (ms == 4600) { dhp_set_low_latency(&P, false); ur_phase[1] = P.out_underruns - ur0; ur0 = P.out_underruns; }
        /* host: nominal 48 frames/ms, nudged by UAC2 feedback that tries to keep
         * the FIFO half full (TinyUSB FIFO_COUNT method, max +-1 frame/ms) */
        static double acc = 0;
        double rate = 48.0 + ((FIFO_CAP / 2) - fl) / (double)(FIFO_CAP / 2);
        if (rate > 49) rate = 49; if (rate < 47) rate = 47;
        acc += rate; int nsend = (int)acc; acc -= nsend;
        for (int i = 0; i < nsend && sent < (uint32_t)N; i++, sent++) {
            if (fl < FIFO_CAP) memcpy(fifo + fl++ * 6, src + sent * 6, 12); else lost++; }
        uint32_t want = dhp_usb_frames_wanted_guarded(&P, fl, FIFO_CAP * 3 / 4, FIFO_CAP / 2);
        if (want > (uint32_t)fl) want = fl;
        uint32_t w = dhp_usb_write(&P, fifo, want); memmove(fifo, fifo + w * 6, (fl - w) * 12); fl -= w;
        for (int k = 0; k < 4; k++) dhp_dsp_poll(&P);
        if (!started && dhp_out_available(&P) >= 1024) started = 1;
        if (started) { dhp_read_stereo(&P, tmp, 48); memcpy(out + outp * 2, tmp, 48 * 4); outp += 48; }
        int settle = (ms % 2300) > 1500;      /* skip the transitions */
        if (settle) { fill_sum[phase] += fl + dhp_out_available(&P) + (P.in_w - P.in_r); fill_n[phase]++; }
    }
    ur_phase[2] = P.out_underruns - ur0;
    const char *nm[3] = {"rooms", "low-latency", "rooms again"};
    for (int i = 0; i < 3; i++)
        printf("%-12s buffered %.1f ms (USB FIFO + rings), underruns %u\n", nm[i], fill_sum[i] / fill_n[i] / 48.0, ur_phase[i]);
    /* LL check: find the lag where out == downmix(src) during the settled LL phase */
    int best_lag = 0, found = 0;
    uint32_t o0 = 48 * 4300;               /* output frame index inside LL phase */
    for (int lag = -48 * 400; lag < 48 * 400 && !found; lag++) {
        int ok = 1;
        for (int j = 0; j < 2000 && ok; j++) {
            int s = (int)o0 + j - lag; if (s < 0) { ok = 0; break; }
            const int16_t *fr = src + s * 6;
            float c = 0.7071f * ((float)fr[2] + fr[3]);
            int L = (int)lrintf(0.5f * (fr[0] + c + 0.7071f * fr[4])), R = (int)lrintf(0.5f * (fr[1] + c + 0.7071f * fr[5]));
            if (abs(out[(o0 + j) * 2] - L) > 1 || abs(out[(o0 + j) * 2 + 1] - R) > 1) ok = 0;
        }
        if (ok) { best_lag = lag; found = 1; }
    }
    printf("low-latency output = exact downmix, sample offset %d frames (the switch dropped the queued audio); USB frames lost %d\n",
           best_lag, lost);
    printf("blocks/chunks processed %u, in_overruns %u\n", P.blocks, P.in_overruns);
    return (found && lost == 0 && ur_phase[1] < 3 && ur_phase[2] < 3) ? 0 : 1;
}
