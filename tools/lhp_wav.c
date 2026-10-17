/*
 * lhp_wav - preview the virtualizer on a PC.
 *   lhp_wav [--rear amount] <filter.bin> <in.wav> <out.wav>
 *   --rear 1.0 moves the surround speakers from ~115 to ~140 degrees.
 * in.wav: 16-bit PCM, 1/2/6 channels (6 = WAVE 5.1 order FL FR FC LFE BL/SL BR/SR).
 * out.wav: 16-bit stereo binaural. Use a filter matching the WAV's sample rate
 * (extract with: extract_filters.py DolbyHph.dll --bins dir/ --rates all).
 *
 *   cc -O2 -o lhp_wav tools/lhp_wav.c engine/lakehp.c -Iengine -lm
 */
#include "lakehp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static lhp_engine E;

static uint32_t rd32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static void wr32(FILE *f, uint32_t v) { fputc(v, f); fputc(v >> 8, f); fputc(v >> 16, f); fputc(v >> 24, f); }
static void wr16(FILE *f, uint16_t v) { fputc(v, f); fputc(v >> 8, f); }

int main(int argc, char **argv)
{
    float rear = 0.0f;
    if (argc == 6 && !strcmp(argv[1], "--rear")) { rear = (float)atof(argv[2]); argv += 2; argc -= 2; }
    if (argc != 4) { fprintf(stderr, "usage: %s [--rear amount] filter.bin in.wav out.wav\n", argv[0]); return 1; }
    FILE *ff = fopen(argv[1], "rb");
    if (!ff) { perror(argv[1]); return 1; }
    static uint8_t blob[1 << 17];
    size_t blen = fread(blob, 1, sizeof blob, ff); fclose(ff);

    FILE *in = fopen(argv[2], "rb");
    if (!in) { perror(argv[2]); return 1; }
    uint8_t hdr[12], ck[8];
    if (fread(hdr, 1, 12, in) != 12 || memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) { fprintf(stderr, "not a WAV\n"); return 1; }
    int nch = 0, bits = 0; uint32_t rate = 0, data_len = 0;
    while (fread(ck, 1, 8, in) == 8) {
        uint32_t len = rd32(ck + 4);
        if (!memcmp(ck, "fmt ", 4)) {
            uint8_t fmt[40] = {0};
            if (fread(fmt, 1, len < 40 ? len : 40, in) == 0) return 1;
            if (len > 40) fseek(in, len - 40, SEEK_CUR);
            nch = rd16(fmt + 2); rate = rd32(fmt + 4); bits = rd16(fmt + 14);
        } else if (!memcmp(ck, "data", 4)) { data_len = len; break; }
        else fseek(in, len + (len & 1), SEEK_CUR);
    }
    if (bits != 16 || !data_len || (nch != 1 && nch != 2 && nch != 6)) { fprintf(stderr, "need 16-bit PCM, 1/2/6 channels\n"); return 1; }

    lhp_init(&E);
    if (nch == 6) { const int8_t map[6] = {0, 2, 1, 4, 5, 3}; lhp_set_map(&E, map, 6); }
    else lhp_set_acmod(&E, nch == 1 ? 1 : 2);
    lhp_set_surround_rear(&E, rear);
    int r = lhp_load_filter(&E, blob, blen, 1.0f);
    if (r) { fprintf(stderr, "filter load failed (%d)\n", r); return 1; }
    if (E.info.sample_rate != rate) fprintf(stderr, "warning: WAV is %u Hz but filter is %u Hz\n", rate, E.info.sample_rate);
    fprintf(stderr, "%s, %d ch @ %u Hz\n", E.info.name, nch, rate);

    FILE *out = fopen(argv[3], "wb");
    if (!out) { perror(argv[3]); return 1; }
    fwrite("RIFF\0\0\0\0WAVEfmt ", 1, 16, out);
    wr32(out, 16); wr16(out, 1); wr16(out, 2); wr32(out, rate); wr32(out, rate * 4); wr16(out, 4); wr16(out, 16);
    fwrite("data\0\0\0\0", 1, 8, out);

    static int16_t ib[LHP_BLOCK * 6], ob[LHP_BLOCK * 2];
    uint32_t frames = data_len / (2 * nch), done = 0, written = 0;
    /* run one extra block of silence so the 1024-frame latency is flushed */
    while (done < frames + LHP_BLOCK) {
        size_t got = done < frames ? fread(ib, 2 * nch, LHP_BLOCK, in) : 0;
        memset(ib + got * nch, 0, (LHP_BLOCK - got) * 2 * nch);
        lhp_block_s16(&E, ib, ob);
        fwrite(ob, 4, LHP_BLOCK, out);
        written += LHP_BLOCK; done += LHP_BLOCK;
    }
    uint32_t dlen = written * 4;
    fseek(out, 4, SEEK_SET); wr32(out, 36 + dlen);
    fseek(out, 40, SEEK_SET); wr32(out, dlen);
    fclose(out); fclose(in);
    return 0;
}
