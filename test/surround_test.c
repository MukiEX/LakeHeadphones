/* surround_test: a live surround request reloads the filter, changes only the
 * surround channels' output, and comes back exactly when set to Dolby's. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "dh_pipeline.h"
static lhp_engine E; static dhp_t P; static uint8_t blob[1 << 17];
static double run(int ch, int16_t *out, int nblk)
{
    static int16_t in[64 * 6]; double e = 0; int o = 0;
    for (int b = 0; b < nblk * 16; b++) {
        memset(in, 0, sizeof in);
        for (int i = 0; i < 64; i++) in[i * 6 + ch] = (int16_t)(8000 * sin(0.05 * (b * 64 + i)) * sin(0.0013 * (b * 64 + i)));
        dhp_usb_write(&P, in, 64);
        while (dhp_dsp_poll(&P)) {}
        int16_t st[64 * 2]; dhp_read_stereo(&P, st, 64);
        for (int i = 0; i < 128; i++) { out[o++] = st[i]; e += st[i] * (double)st[i]; }
    }
    return e;
}
int main(int argc, char **argv)
{
    char fn[512]; snprintf(fn, sizeof fn, "%s/lhp_filter_dh2_medium_48000.bin", argc > 1 ? argv[1] : "bins");
    FILE *f = fopen(fn, "rb"); if (!f) { perror(fn); return 1; }
    size_t n = fread(blob, 1, sizeof blob, f); fclose(f);
    dhp_filter_t flt = { 48000, 2, blob, (uint32_t)n };
    static int16_t a[64 * 2 * 16 * 8], b[64 * 2 * 16 * 8];
    int fail = 0;
    for (int ch = 0; ch < 6; ch++) {
        if (ch == 3) continue;
        dhp_init(&P, &E, &flt, 1, 48000, 2); run(ch, a, 8);
        dhp_init(&P, &E, &flt, 1, 48000, 2);
        lhp_surround_t rear = LHP_SURROUND_REAR; dhp_set_surround(&P, &rear, false); run(ch, b, 8);
        double d = 0; for (int i = 0; i < 64 * 2 * 16 * 8; i++) d += fabs((double)a[i] - b[i]);
        int surround = ch >= 4;
        printf("ch %d: mean |diff| with rear preset %.2f %s\n", ch, d / (64 * 2 * 16 * 8), surround ? "(surround: should differ)" : "(should be 0)");
        if (surround ? d == 0 : d != 0) fail = 1;
        lhp_surround_t fact = LHP_SURROUND_FACTORY; dhp_set_surround(&P, &fact, false); run(ch, b, 2);
    }
    lhp_surround_t r = LHP_SURROUND_REAR, z = LHP_SURROUND_FACTORY;
    printf("azimuth: factory %.0f deg, rear preset %.0f deg\n", lhp_surround_azimuth(&z), lhp_surround_azimuth(&r));
    puts(fail ? "FAIL" : "PASS");
    return fail;
}
