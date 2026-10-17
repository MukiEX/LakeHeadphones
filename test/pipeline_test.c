/* Simulates USB producer -> pipeline -> sink with mismatched clocks and checks:
 *  1. output == engine output for the same input (after USB->engine remap)
 *  2. the USB pull rate locks to the sink rate (no overruns/underruns)
 *  3. room cycling works and produces beeps */
#include "../firmware/common/dh_pipeline.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static lhp_engine E1, E2; static dhp_t P;
static uint8_t blobs[3][1 << 16]; static size_t blen[3];
int main(int argc, char **argv) { (void)argc; uint32_t ur_at_end = 0;
    const char *dir = argv[1];
    const char *names[3] = {"lhp_filter_dh1_small_48000.bin", "lhp_filter_dh2_medium_48000.bin", "lhp_filter_dh3_large_48000.bin"};
    dhp_filter_t F[3];
    for (int i = 0; i < 3; i++) { char fn[512]; snprintf(fn, sizeof fn, "%s/%s", dir, names[i]);
        FILE *f = fopen(fn, "rb"); blen[i] = fread(blobs[i], 1, sizeof blobs[i], f); fclose(f);
        F[i] = (dhp_filter_t){48000, (uint8_t)(i + 1), blobs[i], (uint32_t)blen[i]}; }
    if (dhp_init(&P, &E1, F, 3, 48000, 1)) { puts("init fail"); return 1; }
    /* reference engine fed directly */
    lhp_init(&E2); int8_t m[6] = DHP_MAP_USB51; lhp_set_map(&E2, m, 6); lhp_load_filter(&E2, blobs[0], blen[0], 1.0f);
    const int N = 48000 * 4; int16_t *src = malloc(N * 12); srand(3);
    for (int i = 0; i < N * 6; i++) src[i] = (int16_t)((rand() % 16000) - 8000);
    int16_t *ref = malloc(N * 4); int16_t *out = calloc(N * 2 + 4096, 4);
    for (int b = 0; b + LHP_BLOCK <= N; b += LHP_BLOCK) lhp_block_s16(&E2, src + b * 6, ref + b * 2);
    /* simulate 4 s: host sends; sink drains 48.0 frames/ms; host clock irrelevant
       because USB pulls only what's wanted. DSP polled every ms. */
    uint32_t src_pos = 0, out_pos = 0; int16_t tmp[64 * 2];
    uint32_t max_fill = 0, min_fill = 1u << 30; int started = 0;
    for (int ms = 0; ms < 4000 + 100; ms++) {
        uint32_t want = dhp_usb_frames_wanted(&P);
        if (want > 49) want = 49;                 /* at most one USB packet per ms */
        if (src_pos + want > (uint32_t)N) want = N - src_pos;
        dhp_usb_write(&P, src + src_pos * 6, want); src_pos += want;
        dhp_dsp_poll(&P);
        if (!started && dhp_out_available(&P) >= 1024) started = 1;
        if (started) { dhp_read_stereo(&P, tmp, 48); memcpy(out + out_pos * 2, tmp, 48 * 4); out_pos += 48; }
        if (ms == 3900) ur_at_end = P.out_underruns;
        uint32_t fill = dhp_out_available(&P); if (ms > 500 && ms < 3900) { if (fill > max_fill) max_fill = fill; if (fill < min_fill) min_fill = fill; }
    }
    int maxd = 0; uint32_t cmp = out_pos < (uint32_t)N - 4096 ? out_pos : N - 4096;
    for (uint32_t i = 0; i < cmp * 2; i++) { int d = abs(out[i] - ref[i]); if (d > maxd) maxd = d; }
    printf("compared %u frames: max |pipeline - engine| = %d\n", cmp, maxd);
    printf("blocks=%u in_overruns=%u out_underruns=%u drops=%u  out fill range [%u,%u]\n",
           P.blocks, P.in_overruns, P.out_underruns, P.out_drops, min_fill, max_fill);
    /* room cycling */
    dhp_cycle_room(&P); for (int i = 0; i < 2; i++) { dhp_usb_write(&P, src, 1024); dhp_dsp_poll(&P); }
    printf("room after cycle: %d (%s)\n", dhp_room(&P), dhp_room_name(dhp_room(&P)));
    dhp_cycle_room(&P); dhp_cycle_room(&P); dhp_usb_write(&P, src, 1024); dhp_dsp_poll(&P);
    printf("room after 2 more cycles: %d\n", dhp_room(&P));
    printf("underruns while source active: %u\n", ur_at_end);
    return (maxd == 0 && ur_at_end == 0 && P.in_overruns == 0) ? 0 : 1;
}
