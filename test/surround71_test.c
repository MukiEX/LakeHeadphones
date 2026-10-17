/* surround71_test: 7.1 back pair.
 *  1. factory placement: a back channel sounds exactly like the same signal
 *     on the 5.1 surround (it is mixed into the surround input)
 *  2. back pair moved: a back channel sounds exactly like the 5.1 surround
 *     with that placement
 *  3. side + back together = sum of the two (linearity, rounding only)
 *  4. the pipeline in 7.1 mode matches the engine
 *  5. azimuth <-> placement mapping
 * usage: surround71_test <bins dir> */
#include "lakehp.h"
#include "dh_pipeline.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define NB 24
static lhp_engine E;
static uint8_t blob[1 << 16];
static size_t blen;
static int16_t in8[NB * LHP_BLOCK * 8], in6[NB * LHP_BLOCK * 6];
static int16_t o1[NB * LHP_BLOCK * 2], o2[NB * LHP_BLOCK * 2], o3[NB * LHP_BLOCK * 2];

static uint32_t rng = 1;
static int16_t noise(void) { rng = rng * 1664525u + 1013904223u; return (int16_t)((int32_t)(rng >> 16) - 32768) / 6; }

static void run(int nch, const lhp_surround_t *sur, const lhp_surround_t *back, const int16_t *in, int16_t *out)
{
    static const int8_t m51[6] = DHP_MAP_USB51, m71[6] = DHP_MAP_USB71;
    lhp_init(&E);
    if (nch == 8) { lhp_set_map(&E, m71, 8); lhp_set_map_back(&E, 4, 5); }
    else lhp_set_map(&E, m51, 6);
    lhp_set_surround(&E, sur);
    lhp_set_back(&E, back);
    if (lhp_load_filter(&E, blob, blen, 1.0f)) { printf("load failed\n"); exit(1); }
    for (int b = 0; b < NB; b++) lhp_block_s16(&E, &in[b * LHP_BLOCK * nch], &out[b * LHP_BLOCK * 2]);
}

static int maxdiff(const int16_t *a, const int16_t *b)
{
    int m = 0;
    for (int i = 0; i < NB * LHP_BLOCK * 2; i++) { int d = abs(a[i] - b[i]); if (d > m) m = d; }
    return m;
}
static double rms(const int16_t *a) { double s = 0; for (int i = 0; i < NB * LHP_BLOCK * 2; i++) s += (double)a[i] * a[i]; return sqrt(s / (NB * LHP_BLOCK * 2)); }

int main(int argc, char **argv)
{
    char path[512];
    snprintf(path, sizeof path, "%s/lhp_filter_dh2_medium_48000.bin", argc > 1 ? argv[1] : "../bins");
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 1; }
    blen = fread(blob, 1, sizeof blob, f); fclose(f);
    const lhp_surround_t F = LHP_SURROUND_FACTORY, R = LHP_SURROUND_REAR;
    const lhp_surround_t FAR = lhp_surround_for_azimuth(150);
    int fail = 0;

    /* signal A on the back-left channel only */
    memset(in8, 0, sizeof in8); memset(in6, 0, sizeof in6);
    for (int i = 0; i < NB * LHP_BLOCK; i++) { int16_t v = noise(); in8[i * 8 + 4] = v; in6[i * 6 + 4] = v; }

    run(8, &F, &F, in8, o1); run(6, &F, &F, in6, o2);
    int d = maxdiff(o1, o2);
    printf("1. factory: 7.1 back-left vs 5.1 surround-left: max diff %d (want 0), level %.0f\n", d, rms(o1));
    fail |= d != 0 || rms(o1) < 100;

    run(8, &F, &FAR, in8, o1); run(6, &FAR, &F, in6, o2);
    d = maxdiff(o1, o2);
    printf("2. back at 150: 7.1 back-left vs 5.1 surround-left at 150: max diff %d (want 0)\n", d);
    fail |= d != 0;
    run(6, &F, &F, in6, o3);
    printf("   ... and it differs from the factory position: max diff %d\n", maxdiff(o1, o3));
    fail |= maxdiff(o1, o3) < 50;

    /* 3. B on side-left (ch 6) + A on back-left (ch 4), sides at 136, backs at 150 */
    static int16_t inB6[NB * LHP_BLOCK * 6];
    memset(inB6, 0, sizeof inB6);
    for (int i = 0; i < NB * LHP_BLOCK; i++) { int16_t v = noise(); in8[i * 8 + 6] = v; inB6[i * 6 + 4] = v; }
    run(8, &R, &FAR, in8, o1);
    run(6, &FAR, &F, in6, o2);                       /* A as a surround at 150 */
    run(6, &R, &F, inB6, o3);                        /* B as a surround at 136 */
    int m = 0;
    for (int i = 0; i < NB * LHP_BLOCK * 2; i++) { int dd = abs(o1[i] - (o2[i] + o3[i])); if (dd > m) m = dd; }
    printf("3. side 136 + back 150 together vs sum of separate runs: max diff %d (rounding, want <= 2)\n", m);
    fail |= m > 2;

    /* 4. pipeline in 7.1 mode == engine */
    static dhp_t P; static lhp_engine E2;
    static dhp_filter_t flt[1];
    flt[0] = (dhp_filter_t){ 48000, 2, blob, (uint32_t)blen };
    dhp_init(&P, &E2, flt, 1, 48000, 2);
    dhp_set_placement_now(&P, &R, &FAR);
    if (dhp_set_channels(&P, 8)) { printf("set_channels failed\n"); return 1; }
    static int16_t po[NB * LHP_BLOCK * 2];
    uint32_t got = 0;
    for (int i = 0; i < NB * LHP_BLOCK; i += 96) {
        dhp_usb_write(&P, &in8[i * 8], 96);
        while (dhp_dsp_poll(&P)) { }
        got += dhp_read_stereo(&P, &po[got * 2], dhp_out_available(&P));
    }
    run(8, &R, &FAR, in8, o1);
    m = 0;
    for (uint32_t i = 0; i < got * 2; i++) { int dd = abs(po[i] - o1[i]); if (dd > m) m = dd; }
    printf("4. pipeline 7.1 vs engine: %u frames, max diff %d (want <= 1)\n", got, m);
    fail |= m > 1 || got < (NB - 2) * LHP_BLOCK;

    /* 5. mapping */
    printf("5. azimuth -> placement -> azimuth:\n");
    const float az[] = { 90, 100, 110, 118, 136, 150, 165 };
    for (int i = 0; i < 7; i++) {
        lhp_surround_t s = lhp_surround_for_azimuth(az[i]);
        float back = lhp_surround_azimuth(&s);
        printf("   %5.0f deg: delay %5.1f  far HF %+5.1f dB  rear cues %+5.2f  -> %5.1f deg\n",
               az[i], s.near_delay, s.far_hf_db, s.back_eq, back);
        fail |= fabsf(back - az[i]) > 1.0f;
    }
    printf("   rear preset (6, 4, 1): %.1f deg\n", lhp_surround_azimuth(&R));
    printf(fail ? "FAIL\n" : "PASS\n");
    return fail;
}
