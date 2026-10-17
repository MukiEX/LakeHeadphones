/* host_run: process interleaved s16 through lakehp.
   usage: host_run filter.bin acmodCfg in.s16 out.s16 [gain] */
#include "../engine/lakehp.h"
#include <stdio.h>
#include <stdlib.h>
static lhp_engine E;
int main(int argc, char **argv) {
    if (argc < 5) { fprintf(stderr, "usage: %s filter.bin cfg in.s16 out.s16\n", argv[0]); return 1; }
    FILE *f = fopen(argv[1], "rb"); static uint8_t blob[1 << 20];
    size_t n = fread(blob, 1, sizeof blob, f); fclose(f);
    lhp_init(&E);
    if (lhp_set_acmod(&E, atoi(argv[2]))) { fprintf(stderr, "bad cfg\n"); return 1; }
    int r = lhp_load_filter(&E, blob, n, argc > 5 ? (float)atof(argv[5]) : 1.0f);
    fprintf(stderr, "load=%d name=%s rate=%u room=%d flags=%02x coef_total=%u hist_total=%u state=%zu bytes\n",
            r, E.info.name, E.info.sample_rate, E.info.room, E.info.flags, E.coef_total, E.hist_total, sizeof E);
    FILE *in = fopen(argv[3], "rb"), *out = fopen(argv[4], "wb");
    static int16_t ib[LHP_BLOCK * LHP_MAXCH], ob[LHP_BLOCK * 2];
    size_t got; int nch = E.nch;
    while ((got = fread(ib, 2 * nch, LHP_BLOCK, in)) > 0) {
        for (size_t i = got * nch; i < (size_t)LHP_BLOCK * nch; i++) ib[i] = 0;
        lhp_block_s16(&E, ib, ob);
        fwrite(ob, 4, LHP_BLOCK, out);
        if (got < LHP_BLOCK) break;
    }
    fclose(in); fclose(out); return 0;
}
