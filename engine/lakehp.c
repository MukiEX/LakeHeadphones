/*
 * lakehp.c - portable C reimplementation of the Lake/Dolby Headphone engine.
 * See lakehp.h for the signal-flow overview. Function-by-function notes
 * reference the original DolbyHph.dll (1.20.0276, x87 code path) addresses.
 */
#include "lakehp.h"
#include <string.h>
#include <math.h>

#ifndef LHP_PI
#define LHP_PI 3.14159265358979323846
#endif

#ifdef LHP_COEF_HALF
#define CF(x) ((float)(x))
#define TOCF(x) ((lhp_coef_t)(x))
#else
#define CF(x) (x)
#define TOCF(x) (x)
#endif

/* Time-critical functions can be placed in fast RAM on MCUs. */
#ifndef LHP_HOT
#if defined(LHP_PICO_RAMFUNC)                 /* pico-sdk: .time_critical -> SRAM */
#define LHP_HOT __attribute__((section(".time_critical.lhp"), noinline))
#else
#define LHP_HOT
#endif
#endif

/* ------------------------------------------------------------------------ */
/* Tables  (DLL: 0x1000336c rot/win, 0x10004dcc/0x10004ea4 FFT twiddles)     */
/* ------------------------------------------------------------------------ */
static void build_tables(lhp_engine *e)
{
    for (int i = 0; i < LHP_BLOCK; i++) {
        double th = (double)i * LHP_PI * 0.5 / 1024.0;
        float c = (float)(32768.0 * cos(th));
        float s = (float)(32768.0 * sin(th));
        e->rot[i].re = c;
        e->rot[i].im = s;
        e->win[i] = (uint16_t)(int32_t)s;          /* truncation (fistp, RC=chop) */
    }
    e->win[LHP_BLOCK] = 0;
    for (int i = 0; i < LHP_BLOCK / 2; i++) {
        double th = (double)(i + i) * LHP_PI / (double)LHP_BLOCK;
        e->fft_fwd[i].re = (float)cos(th);
        e->fft_fwd[i].im = (float)-sin(th);
        e->fft_inv[i].re = (float)cos(th);
        e->fft_inv[i].im = (float)sin(th);
    }
}

/* ------------------------------------------------------------------------ */
/* FFT: radix-2 DIT, bit-reversed input permutation, natural-order output,  */
/* no scaling.  Same loop structure as DLL 0x10004f7c / 0x1000511c.          */
/* ------------------------------------------------------------------------ */
LHP_HOT void lhp_fft(lhp_cpx *d, const lhp_cpx *tw)
{
    const unsigned n = LHP_BLOCK;
    /* bit reversal */
    for (unsigned i = 0, j = 0; i < n; i++) {
        if (j > i) { lhp_cpx t = d[j]; d[j] = d[i]; d[i] = t; }
        unsigned m = n >> 1;
        while (m & j) m >>= 1;
        j ^= (m - 1) ^ (n - 1);
        /* equivalent to: j = bitrev(i+1) */
    }
    for (unsigned h = 1, s = n >> 1; s; h <<= 1, s >>= 1) {
        {   /* j = 0: twiddle is exactly 1 (+-0j), so skip the multiplies;
             * bit-identical to the general butterfly below (~20% of them) */
            lhp_cpx *p0 = d, *p1 = d + h;
            for (unsigned g = 0; g < s; g++) {
                const float ar = p0->re, ai = p0->im, tr = p1->re, ti = p1->im;
                p1->re = ar - tr; p1->im = ai - ti;
                p0->re = ar + tr; p0->im = ai + ti;
                p0 += 2 * h; p1 += 2 * h;
            }
        }
        for (unsigned j = 1; j < h; j++) {
            const float wr = tw[j * s].re, wi = tw[j * s].im;
            lhp_cpx *p0 = d + j, *p1 = d + j + h;
            for (unsigned g = 0; g < s; g++) {
                float tr = wr * p1->re - wi * p1->im;
                float ti = wi * p1->re + wr * p1->im;
                float ar = p0->re, ai = p0->im;
                p1->re = ar - tr; p1->im = ai - ti;
                p0->re = ar + tr; p0->im = ai + ti;
                p0 += 2 * h; p1 += 2 * h;
            }
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Channel maps (DLL table @0x100a1f80): AC-3 acmod + 8*lfeon               */
/* entries: nch, then input-channel index for L,C,R,Ls,Rs,LFE (-1 none)     */
/* ------------------------------------------------------------------------ */
static const int8_t k_maps[16][7] = {
    {2,  0,-1, 1,-1,-1,-1}, {1, -1, 0,-1,-1,-1,-1}, {2,  0,-1, 1,-1,-1,-1},
    {3,  0, 1, 2,-1,-1,-1}, {3,  0,-1, 1, 2, 2,-1}, {4,  0, 1, 2, 3, 3,-1},
    {4,  0,-1, 1, 2, 3,-1}, {5,  0, 1, 2, 3, 4,-1}, {3,  0,-1, 1,-1,-1, 2},
    {2, -1, 0,-1,-1,-1, 1}, {3,  0,-1, 1,-1,-1, 2}, {4,  0, 1, 2,-1,-1, 3},
    {4,  0,-1, 1, 2, 2, 3}, {5,  0, 1, 2, 3, 3, 4}, {5,  0,-1, 1, 2, 3, 4},
    {6,  0, 1, 2, 3, 4, 5},
};

int lhp_set_map(lhp_engine *e, const int8_t map[LHP_NPOS], int nch)
{
    if (nch < 1 || nch > LHP_MAXCH) return -1;
    for (int p = 0; p < LHP_NPOS; p++) {
        if (map[p] >= nch) return -1;
        e->map[p] = map[p];
    }
    e->nch = nch;
    e->map_back[0] = e->map_back[1] = -1;
    memset(e->inbuf, 0, sizeof e->inbuf);       /* DLL clears unused channels */
    return 0;
}

int lhp_set_map_back(lhp_engine *e, int lb, int rb)
{
    if (lb >= e->nch || rb >= e->nch || (lb < 0) != (rb < 0)) return -1;
    e->map_back[0] = (int8_t)(lb < 0 ? -1 : lb);
    e->map_back[1] = (int8_t)(rb < 0 ? -1 : rb);
    return 0;
}

int lhp_set_acmod(lhp_engine *e, int cfg)
{
    if (cfg < 0 || cfg > 15) return -1;
    return lhp_set_map(e, &k_maps[cfg][1], k_maps[cfg][0]);
}

void lhp_reset(lhp_engine *e)   /* DLL LakeFlush */
{
    memset(e->hist, 0, sizeof e->hist);
    memset(e->mix, 0, sizeof e->mix);
    memset(e->inbuf, 0, sizeof e->inbuf);
    e->fill = 0;
}

void lhp_init(lhp_engine *e)
{
    memset(e, 0, sizeof *e);
    build_tables(e);
    e->gain = 1.0f;
    lhp_set_acmod(e, 15);
}

/* ------------------------------------------------------------------------ */
/* Filter blob decoding (DLL 0x10001354 header, 0x10001814 samples)          */
/* ------------------------------------------------------------------------ */
typedef struct {
    const uint8_t *p;
    uint32_t rem;      /* bytes allowed to read                           */
    int bits;          /* unread bits left in cur                         */
    int cur;           /* current byte (sign-extended, as in the DLL)     */
    int maxbits, scale_exp, hb, mask, off;
} lhp_br;

static int br_s8(lhp_br *b) { if (!b->rem) return 0; b->rem--; return (int8_t)*b->p++; }
static int br_u8(lhp_br *b) { if (!b->rem) return 0; b->rem--; return *b->p++; }
static int br_u16(lhp_br *b) { int lo = br_u8(b); int hi = br_s8(b); return (hi << 8) | lo; }
static uint32_t br_u32(lhp_br *b)
{
    uint32_t v = (uint32_t)br_u8(b);
    v |= (uint32_t)br_u8(b) << 8;
    v |= (uint32_t)br_u8(b) << 16;
    v |= (uint32_t)br_s8(b) << 24;
    return v;
}
static float br_q14(lhp_br *b) { return (float)(int16_t)br_u16(b) / 16384.0f; }

/* LSB-first bit reader returning a sign-extended n-bit value (0x10001b18). */
static int32_t br_bits(lhp_br *b, int n)
{
    static const uint8_t top[9] = {0x00,0x80,0xc0,0xe0,0xf0,0xf8,0xfc,0xfe,0xff};
    const int total = n;
    uint32_t r = 0;
    while (n >= b->bits) {
        uint32_t t = ((uint32_t)b->cur & top[b->bits]) << 24;
        r = (b->bits >= 32 ? 0 : (r >> b->bits)) | t;
        n -= b->bits;
        b->bits = 8;
        b->cur = br_s8(b);
    }
    if (n > 0) {
        uint32_t t = (((uint32_t)b->cur << (b->bits - n)) & top[n]) << 24;
        r = (r >> n) | t;
        b->bits -= n;
    }
    if (total <= 0) return 0;
    return (int32_t)r >> (32 - total);
}

static uint32_t rate_from_code(int c)
{
    switch (c) {
    case -1: return 0xffffffffu;
    case 0: return 48000; case 1: return 44100; case 2: return 32000;
    case 3: return 22050; case 4: return 16000; case 5: return 11025;
    case 6: return 8000; case 16: return 192000; case 17: return 96000;
    default: return 0;
    }
}

static int parse_header(lhp_br *b, const uint8_t *blob, size_t len, lhp_filter_info *h)
{
    memset(h, 0, sizeof *h);
    memset(b, 0, sizeof *b);
    b->p = blob; b->rem = len < 5 ? (uint32_t)len : 5;
    h->version = br_s8(b);
    if (h->version == 1) { b->off = 1; b->hb = 4; }
    else if (h->version == 2) { b->hb = 5; }
    else return -1;
    b->mask = (1 << b->hb) - 1;
    h->total_len = br_u32(b);
    if (h->total_len > len || h->total_len < 5) return -2;
    b->rem = h->total_len - 5;
    h->header_len = (uint16_t)br_u16(b);
    h->hdr3 = br_s8(b);
    h->hdr4 = br_s8(b);
    h->room = br_s8(b);
    h->sample_rate = rate_from_code(br_s8(b));
    if (!h->sample_rate) return -3;
    h->flags = br_s8(b) & 0xff;
    h->max_bits = br_s8(b);
    b->maxbits = h->max_bits;
    h->scale_exp = (h->version >= 2) ? br_s8(b) : h->max_bits;
    b->scale_exp = h->scale_exp;
    for (int i = 0; i < 5; i++) h->pos_gain[i] = br_q14(b);
    for (int i = 0; i < 12; i++) h->tail_cut[i] = (uint16_t)br_u16(b);
    for (int i = 0; i < 10; i++) h->misc[i] = br_q14(b);
    h->hdr37 = br_s8(b);
    int nlen = br_s8(b) + 1;
    for (int i = 0; i < nlen; i++) {
        int c = br_s8(b);
        if (i < (int)sizeof h->name - 1) h->name[i] = (char)c;
    }
    if (b->rem != h->total_len - h->header_len) return -4;
    return 0;
}

int lhp_parse_header(const uint8_t *blob, size_t len, lhp_filter_info *out)
{
    lhp_br b;
    return parse_header(&b, blob, len, out);
}

/* one 1024-sample impulse-response partition (64 groups of 16) */
static void decode_part(lhp_br *b, float *dst)
{
    const float scale = 1.0f / (float)(1 << (b->scale_exp - 1));
    for (int g = 0; g < LHP_BLOCK; g += 16) {
        int e = (br_bits(b, b->hb) & b->mask) + b->off;
        int nb = e, sh = 0;
        if (nb > b->maxbits) { nb = b->maxbits; sh = e - b->maxbits; }
        for (int i = 0; i < 16; i++) {
            int32_t v = br_bits(b, nb) << sh;
            dst[g + i] = (float)v * scale;
        }
    }
}

/* time-domain partition -> lapped spectrum (0x10003618 + FFT) */
static void part_to_spectrum(lhp_engine *e, const float *t, lhp_cpx *X)
{
    /* DLL scales by 2^-55. We store coefficients 2^40 larger (exact power-of-two
     * shift, keeps them inside FP16 range) and undo it in the output stage. */
    const float k55 = 3.0517578125e-05f;         /* 2^-55 * 2^40 = 2^-15 */
    for (int k = 0; k < LHP_BLOCK; k++) {
        float f = t[k] * k55 * e->gain;
        X[k].re = f * e->rot[k].re;
        X[k].im = f * e->rot[k].im;
    }
    lhp_fft(X, e->fft_fwd);
}

/* ------------------------------------------------------------------------ */
/* Surround placement (an addition, not in the DLL)
 *
 * The factory surround HRTFs measure as ITD 0.50 ms and ILD ~15 dB, a lateral angle
 * of about 65 degrees: the ITU 110-120 degree surround position, which on
 * headphones often collapses to "beside you". A source further behind, about
 * 140 degrees, has the same lateral cues as one at 40 degrees, plus
 * rear spectral cues. So for each surround position's direct (time-domain) response:
 *   - delay the near ear by 6 samples (0.125 ms): ITD 0.50 -> ~0.375 ms
 *   - raise the far ear above ~2 kHz by 4 dB: ILD ~15 -> ~11 dB
 *   - both ears: the "back" directional bands (Blauert) +2.5 dB at 1 kHz and
 *     +2 dB at 11 kHz, and -3 dB at 4.5 kHz, where the outer ear boosts sound from the front
 * All scaled by `amount`. The room tail is shared by all positions and stays. */
typedef struct { float b0, b1, b2, a1, a2; } biquad_t;

static biquad_t bq_peak(float fc, float q, float db)
{
    const float A = powf(10.0f, db / 40.0f), w = 2.0f * LHP_PI * fc / 48000.0f;
    const float al = sinf(w) / (2.0f * q), cw = cosf(w), a0 = 1.0f + al / A;
    return (biquad_t){ (1.0f + al * A) / a0, -2.0f * cw / a0, (1.0f - al * A) / a0, -2.0f * cw / a0, (1.0f - al / A) / a0 };
}

static biquad_t bq_highshelf(float fc, float db)
{
    const float A = powf(10.0f, db / 40.0f), w = 2.0f * LHP_PI * fc / 48000.0f;
    const float cw = cosf(w), al = sinf(w) / 2.0f * sqrtf(2.0f), sA = 2.0f * sqrtf(A) * al;
    const float a0 = (A + 1) - (A - 1) * cw + sA;
    return (biquad_t){ A * ((A + 1) + (A - 1) * cw + sA) / a0, -2 * A * ((A - 1) + (A + 1) * cw) / a0,
                       A * ((A + 1) + (A - 1) * cw - sA) / a0, 2 * ((A - 1) - (A + 1) * cw) / a0,
                       ((A + 1) - (A - 1) * cw - sA) / a0 };
}

static void bq_run(float *x, int n, biquad_t f)
{
    float z1 = 0, z2 = 0;
    for (int i = 0; i < n; i++) {
        const float in = x[i], y = f.b0 * in + z1;
        z1 = f.b1 * in - f.a1 * y + z2;
        z2 = f.b2 * in - f.a2 * y;
        x[i] = y;
    }
}

static void delay_frac(float *t, float d)          /* fractional delay via linear interp */
{
    const int di = (int)d;
    const float fr = d - (float)di;
    for (int i = LHP_BLOCK - 1; i >= 0; i--) {
        const int s0 = i - di, s1 = s0 - 1;
        const float v0 = s0 >= 0 ? t[s0] : 0.0f, v1 = s1 >= 0 ? t[s1] : 0.0f;
        t[i] = v0 * (1.0f - fr) + v1 * fr;
    }
}

static void surround_place(float *t, const lhp_surround_t *sp, int near_ear)
{
    if (sp->near_delay == 0.0f && sp->far_hf_db == 0.0f && sp->back_eq == 0.0f) return;
    float d = sp->near_delay;
    if (d > 24.0f) d = 24.0f;
    if (d < -12.0f) d = -12.0f;
    if (near_ear) {
        if (d > 0.0f) delay_frac(t, d);               /* shorter ITD: further back */
    } else {
        if (d < 0.0f) delay_frac(t, -d);              /* longer ITD: further forward */
        if (sp->far_hf_db != 0.0f) bq_run(t, LHP_BLOCK, bq_highshelf(2000.0f, sp->far_hf_db));
    }
    if (sp->back_eq != 0.0f) {
        bq_run(t, LHP_BLOCK, bq_peak(1000.0f, 1.2f, 2.5f * sp->back_eq));
        bq_run(t, LHP_BLOCK, bq_peak(4500.0f, 0.9f, -3.0f * sp->back_eq));
        bq_run(t, LHP_BLOCK, bq_peak(11000.0f, 1.5f, 2.0f * sp->back_eq));
    }
}

void lhp_set_surround(lhp_engine *e, const lhp_surround_t *s) { e->sur = *s; }
void lhp_set_back(lhp_engine *e, const lhp_surround_t *s) { e->back = *s; }
void lhp_set_surround_rear(lhp_engine *e, float amount)
{
    if (amount < 0) amount = 0;
    e->sur = (lhp_surround_t){ 6.0f * amount, 4.0f * amount, amount };
}
int lhp_surround_equal(const lhp_surround_t *a, const lhp_surround_t *b)
{
    return a->near_delay == b->near_delay && a->far_hf_db == b->far_hf_db && a->back_eq == b->back_eq;
}

/* Woodworth: ITD = a/c * (theta + sin theta), theta = lateral angle. The
 * factory surround measures ITD 0.50 ms = lateral ~62 deg, behind: 118 deg. */
#define HEAD_AC   (0.0875 / 343.0)                     /* head radius / c */
#define ITD_FACT  0.000500

float lhp_surround_azimuth(const lhp_surround_t *s)
{
    double itd = ITD_FACT - (double)s->near_delay / 48000.0;
    const double imax = HEAD_AC * (LHP_PI / 2 + 1.0);
    if (itd < 0) itd = 0;
    if (itd > imax) itd = imax;
    double lo = 0, hi = LHP_PI / 2;                    /* solve a*(th + sin th) = itd */
    for (int i = 0; i < 40; i++) { double m = 0.5 * (lo + hi); if (HEAD_AC * (m + sin(m)) < itd) lo = m; else hi = m; }
    return (float)(180.0 - lo * 180.0 / LHP_PI);
}

lhp_surround_t lhp_surround_for_azimuth(float deg)
{
    if (deg < 90.0f) deg = 90.0f;
    if (deg > 165.0f) deg = 165.0f;
    if (fabsf(deg - 118.0f) < 0.5f) return (lhp_surround_t){ 0.0f, 0.0f, 0.0f };
    const double th = (180.0 - deg) * LHP_PI / 180.0;  /* lateral angle */
    const double itd = HEAD_AC * (th + sin(th));
    lhp_surround_t s;
    s.near_delay = (float)((ITD_FACT - itd) * 48000.0);
    s.far_hf_db = (deg - 118.0f) * (4.0f / 18.0f);     /* rear preset: +4 dB at 136 */
    s.back_eq = deg >= 118.0f ? (deg - 118.0f) / 18.0f : (deg - 118.0f) / 28.0f;
    if (s.far_hf_db < -6.0f) s.far_hf_db = -6.0f;
    if (s.far_hf_db > 12.0f) s.far_hf_db = 12.0f;
    if (s.back_eq < -1.0f) s.back_eq = -1.0f;
    if (s.back_eq > 2.0f) s.back_eq = 2.0f;
    /* round to the precision the settings page shows */
    s.near_delay = roundf(s.near_delay * 10.0f) / 10.0f;
    s.far_hf_db = roundf(s.far_hf_db * 10.0f) / 10.0f;
    s.back_eq = roundf(s.back_eq * 100.0f) / 100.0f;
    return s;
}

/* per-bin partition counts, including the DLL's one-step-per-bin RLE (0x100031e0) */
static void build_counts(lhp_engine *e)
{
    int cut[6];
    uint8_t raw[LHP_BLOCK];
    for (int i = 0; i < 6; i++) {
        int32_t x = (int32_t)((uint32_t)e->info.tail_cut[2 * i] << 10);
        uint32_t t = ((uint32_t)(x >> 8)) >> 23;
        cut[i] = (int32_t)(x + t) >> 9;
    }
    for (int k = 0; k < 512; k++) {
        int c = 0;
        for (int i = 0; i < 6; i++) if (cut[i] > k) c++;
        if (!c) c = 1;
        raw[k] = (uint8_t)c;
        raw[1023 - k] = (uint8_t)c;
    }
    uint16_t thr[12];
    for (int i = 0; i < 12; i++) thr[i] = 0x400;
    int cur = raw[0], n = 0, k;
    for (k = 0; k < 512; k++)
        if (raw[k] < cur && n < 12) { thr[n++] = (uint16_t)k; cur--; }
    for (; k < 1024; k++)
        if (raw[k] > cur && n < 12) { thr[n++] = (uint16_t)(k | 0x4000); cur++; }
    cur = raw[0];
    int ti = 0, hs = 0, cs = 0;
    for (k = 0; k < LHP_BLOCK; k++) {
        uint16_t t = ti < 12 ? thr[ti] : 0x400;
        if (k >= (t & 0xbfff)) { cur += (t & 0x4000) ? 1 : -1; ti++; }
        if (cur < 0) cur = 0;
        if (cur > LHP_MAXPART) cur = LHP_MAXPART;
        e->cnt[k] = (uint8_t)cur;
        cs += cur;
        hs += cur > 1 ? cur - 1 : 0;
    }
    e->coef_total = (uint16_t)cs;
    e->hist_total = (uint16_t)hs;
}

int lhp_load_filter(lhp_engine *e, const uint8_t *blob, size_t len, float gain)
{
    lhp_br b;
    lhp_filter_info h;
    if (parse_header(&b, blob, len, &h)) return -1;
    if (h.room == 0) return -2;          /* stereo-mixdown filter: other code path, not ported */

    e->loaded = 0;
    e->info = h;
    e->gain = gain;
    build_counts(e);
    if (e->coef_total > LHP_TAIL_MAX || e->hist_total > LHP_HIST_MAX) return -3;
    memset(e->dir, 0, sizeof e->dir);
    memset(e->tail, 0, sizeof e->tail);
    memset(e->hist, 0, sizeof e->hist);
    memset(e->mix, 0, sizeof e->mix);

    /* Scratch: outL is rebuilt every block, so it is free during a load.
     * (Keeps this off the stack - MCU task stacks are small.) */
    float *t = (float *)e->outL;
    float *raw = (float *)e->outR;       /* unplaced surround response, for the back pair */
    lhp_cpx *X = e->X;
    const int back = e->map_back[0] >= 0 && e->map_back[1] >= 0;
    e->back_fold = !back || lhp_surround_equal(&e->sur, &e->back);

    /* 1) room tail: flags bit0, ear-major, 6 partitions of 1024 samples.
     *    Partition i goes to slot (cnt-1-i) of each bin with cnt > i (0x10003d54). */
    if (h.flags & 1) {
        for (int ear = 0; ear < 2; ear++)
            for (int i = 0; i < LHP_MAXPART; i++) {
                decode_part(&b, t);
                part_to_spectrum(e, t, X);
                int base = 0;
                for (int k = 0; k < LHP_BLOCK; k++) {
                    int c = e->cnt[k];
                    if (c > i) {
                        lhp_cpx2 *d = &e->tail[base + c - 1 - i];
                        if (ear == 0) { d->lr = TOCF(X[k].re); d->li = TOCF(X[k].im); }
                        else          { d->rr = TOCF(X[k].re); d->ri = TOCF(X[k].im); }
                    }
                    base += c;
                }
            }
    }
    /* 2) direct HRTFs: position-major (L,C,R,Ls,Rs), each gated by a flag bit
     *    (L and R share bit1). Missing ones are zero filters. (0x10003ca4) */
    static const int pmask[5] = {2, 4, 2, 8, 16};
    for (int p = 0; p < 5; p++)
        for (int ear = 0; ear < 2; ear++) {
            if (h.flags & pmask[p]) decode_part(&b, t);
            else memset(t, 0, LHP_BLOCK * sizeof(float));
            if (p >= 3 && !e->back_fold) {                   /* 7.1 back pair from the same HRTF */
                const int d = p == 3 ? LHP_DIR_LB : LHP_DIR_RB;
                memcpy(raw, t, LHP_BLOCK * sizeof(float));
                surround_place(raw, &e->back, (p == 3) == (ear == 0));
                part_to_spectrum(e, raw, X);
                for (int k = 0; k < LHP_BLOCK; k++) {
                    if (ear == 0) { e->dir[d][k].lr = TOCF(X[k].re); e->dir[d][k].li = TOCF(X[k].im); }
                    else          { e->dir[d][k].rr = TOCF(X[k].re); e->dir[d][k].ri = TOCF(X[k].im); }
                }
            }
            if (p >= 3)                                      /* Ls (3) / Rs (4) */
                surround_place(t, &e->sur, (p == 3) == (ear == 0));
            part_to_spectrum(e, t, X);
            for (int k = 0; k < LHP_BLOCK; k++) {
                if (ear == 0) { e->dir[p][k].lr = TOCF(X[k].re); e->dir[p][k].li = TOCF(X[k].im); }
                else          { e->dir[p][k].rr = TOCF(X[k].re); e->dir[p][k].ri = TOCF(X[k].im); }
            }
        }
    e->loaded = 1;
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Processing                                                               */
/* ------------------------------------------------------------------------ */
void lhp_stage_input_s16(lhp_engine *e, const int16_t *in, int n)
{
    int16_t (*buf)[LHP_BLOCK] = e->inbuf[e->cur];
    const int nch = e->nch;
    for (int f = 0; f < n; f++)
        for (int c = 0; c < nch; c++)
            buf[c][e->fill + f] = in[f * nch + c];
    e->fill += n;
}

static void stage_input_f32(lhp_engine *e, const float *in, int n)
{
    int16_t (*buf)[LHP_BLOCK] = e->inbuf[e->cur];
    const int nch = e->nch;
    for (int f = 0; f < n; f++)
        for (int c = 0; c < nch; c++) {
            long v = lrintf(in[f * nch + c]);
            buf[c][e->fill + f] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
        }
    e->fill += n;
}

/* Room tail: frequency-dependent partitioned convolution of the previous
 * block's room-send spectrum (DLL 0x10002144). Initialises outL/outR. */
LHP_HOT void lhp_stage_tail(lhp_engine *e)
{
    lhp_cpx *h = e->hist;
    const lhp_cpx2 *c = e->tail;
    for (int k = 0; k < LHP_BLOCK; k++) {
        const int cnt = e->cnt[k];
        float lr = 0, li = 0, rr = 0, ri = 0;
        const lhp_cpx x = e->mix[k];
        int n = cnt;
        if (n > 1) {                                   /* oldest, no shift */
            float hr = h->re, hi = h->im;
            lr += hr * CF(c->lr) - hi * CF(c->li); li += hr * CF(c->li) + hi * CF(c->lr);
            rr += hr * CF(c->rr) - hi * CF(c->ri); ri += hr * CF(c->ri) + hi * CF(c->rr);
            h++; c++; n--;
            while (n > 1) {                            /* shift history down */
                hr = h->re; hi = h->im;
                lr += hr * CF(c->lr) - hi * CF(c->li); li += hr * CF(c->li) + hi * CF(c->lr);
                rr += hr * CF(c->rr) - hi * CF(c->ri); ri += hr * CF(c->ri) + hi * CF(c->rr);
                h[-1] = *h;
                h++; c++; n--;
            }
        }
        if (n) {                                       /* newest = last block's send */
            lr += x.re * CF(c->lr) - x.im * CF(c->li); li += x.re * CF(c->li) + x.im * CF(c->lr);
            rr += x.re * CF(c->rr) - x.im * CF(c->ri); ri += x.re * CF(c->ri) + x.im * CF(c->rr);
            c++;
        }
        if (cnt > 1) h[-1] = x;
        e->mix[k].re = 0; e->mix[k].im = 0;
        e->outL[k].re = lr; e->outL[k].im = li;
        e->outR[k].re = rr; e->outR[k].im = ri;
    }
}

/* (prev + j*new) * e^{j*pi*k/2048} using the integer window table (0x10001d38).
 * The input is the sum of up to three channels (a position's own channel plus
 * folded-in LFE and/or 7.1 back channel); integer sums are exact in float, so
 * this matches the DLL, which folds LFE into the float input in place. */
typedef struct { const int16_t *nw[3], *pv[3]; int n; } lhp_src;

static LHP_HOT void lapped(const lhp_engine *e, const lhp_src *s, lhp_cpx *X)
{
    const int16_t *n0 = s->nw[0], *p0 = s->pv[0];
    const int16_t *n1 = s->n > 1 ? s->nw[1] : 0, *p1 = s->n > 1 ? s->pv[1] : 0;
    const int16_t *n2 = s->n > 2 ? s->nw[2] : 0, *p2 = s->n > 2 ? s->pv[2] : 0;
#define SUM(a, b, c, k) ((float)((int32_t)a[k] + (b ? (int32_t)b[k] : 0) + (c ? (int32_t)c[k] : 0)))
    X[0].re = SUM(p0, p1, p2, 0) * 32768.0f;
    X[0].im = SUM(n0, n1, n2, 0) * 32768.0f;
    if (s->n == 1) {
        for (int k = 1; k < LHP_BLOCK; k++) {
            const float a = (float)e->win[k];
            const float b = (float)e->win[LHP_BLOCK - k];
            const float pv = (float)p0[k], nw = (float)n0[k];
            X[k].re = pv * b - nw * a;
            X[k].im = pv * a + nw * b;
        }
        return;
    }
    for (int k = 1; k < LHP_BLOCK; k++) {
        const float a = (float)e->win[k];
        const float b = (float)e->win[LHP_BLOCK - k];
        const float pv = SUM(p0, p1, p2, k), nw = SUM(n0, n1, n2, k);
        X[k].re = pv * b - nw * a;
        X[k].im = pv * a + nw * b;
    }
#undef SUM
}

static LHP_HOT void accumulate(lhp_engine *e, const lhp_cpx2 *d, float g)   /* 0x100024dc */
{
    for (int k = 0; k < LHP_BLOCK; k++) {
        const float xr = e->X[k].re, xi = e->X[k].im;
        e->outL[k].re += xr * CF(d[k].lr) - xi * CF(d[k].li);
        e->outL[k].im += xr * CF(d[k].li) + xi * CF(d[k].lr);
        e->outR[k].re += xr * CF(d[k].rr) - xi * CF(d[k].ri);
        e->outR[k].im += xr * CF(d[k].ri) + xi * CF(d[k].rr);
        e->mix[k].re  += xr * g;
        e->mix[k].im  += xi * g;
    }
}

static void src_add(lhp_src *s, int16_t (*nw)[LHP_BLOCK], int16_t (*pv)[LHP_BLOCK], int ch)
{
    if (ch < 0) return;
    s->nw[s->n] = nw[ch];
    s->pv[s->n] = pv[ch];
    s->n++;
}

LHP_HOT void lhp_stage_direct(lhp_engine *e)   /* 0x10001c54 */
{
    int16_t (*nw)[LHP_BLOCK] = e->inbuf[e->cur];
    int16_t (*pv)[LHP_BLOCK] = e->inbuf[e->cur ^ 1];

    /* LFE goes into L+R, else into C (0x10001f00). A 7.1 back pair placed
     * like the surrounds is mixed into them. */
    const int lfe = e->map[LHP_POS_LFE];
    const int L = e->map[LHP_POS_L], R = e->map[LHP_POS_R];
    const int lfe_lr = lfe >= 0 && L >= 0 && R >= 0;
    const int back = e->map_back[0] >= 0 && e->map_back[1] >= 0;
    int last_key = -1;
    for (int p = 0; p < 5; p++) {
        const int ch = e->map[p];
        if (ch < 0) continue;
        int extra = -1, extra2 = -1;
        if (lfe >= 0 && (lfe_lr ? (p == LHP_POS_L || p == LHP_POS_R) : p == LHP_POS_C)) extra = lfe;
        if (back && e->back_fold && (p == LHP_POS_LS || p == LHP_POS_RS))
            extra2 = e->map_back[p - LHP_POS_LS];
        const int key = ch | (extra + 1) << 4 | (extra2 + 1) << 8;
        if (key != last_key) {
            last_key = key;
            lhp_src s = { .n = 0 };
            src_add(&s, nw, pv, ch);
            src_add(&s, nw, pv, extra);
            src_add(&s, nw, pv, extra2);
            lapped(e, &s, e->X);
            lhp_fft(e->X, e->fft_fwd);
        }
        accumulate(e, e->dir[p], e->info.pos_gain[p]);
    }
    if (back && !e->back_fold) {
        for (int i = 0; i < 2; i++) {
            lhp_src s = { .n = 0 };
            src_add(&s, nw, pv, e->map_back[i]);
            lapped(e, &s, e->X);
            lhp_fft(e->X, e->fft_fwd);
            accumulate(e, e->dir[LHP_DIR_LB + i], e->info.pos_gain[LHP_POS_LS + i]);
        }
    }
    e->cur ^= 1;
}

LHP_HOT void lhp_stage_inverse(lhp_engine *e)
{
    lhp_fft(e->outL, e->fft_inv);
    lhp_fft(e->outR, e->fft_inv);
}

static inline int16_t sat16(float v)
{
    long i = lrintf(v);
    if (i > 32767) i = 32767;
    if (i < -32768) i = -32768;
    return (int16_t)i;
}

#define LHP_UNSCALE 9.094947017729282e-13f   /* 2^-40, see part_to_spectrum */

LHP_HOT void lhp_stage_output_s16(lhp_engine *e, int16_t *out)   /* 0x1000266c */
{
    float b = 32768.0f, a = (float)e->win[0];
    for (int k = 0; k < LHP_BLOCK; k++) {
        float l = (e->outL[k].im * b - e->outL[k].re * a) * LHP_UNSCALE;
        float r = (e->outR[k].im * b - e->outR[k].re * a) * LHP_UNSCALE;
        out[2 * k] = sat16(l);
        out[2 * k + 1] = sat16(r);
        b = (float)e->win[LHP_BLOCK - (k + 1)];
        a = (float)e->win[k + 1];
    }
}

LHP_HOT void lhp_stage_output_f32(lhp_engine *e, float *out)
{
    float b = 32768.0f, a = (float)e->win[0];
    for (int k = 0; k < LHP_BLOCK; k++) {
        out[2 * k]     = (e->outL[k].im * b - e->outL[k].re * a) * LHP_UNSCALE;
        out[2 * k + 1] = (e->outR[k].im * b - e->outR[k].re * a) * LHP_UNSCALE;
        b = (float)e->win[LHP_BLOCK - (k + 1)];
        a = (float)e->win[k + 1];
    }
}

static void run_block(lhp_engine *e)
{
    lhp_stage_tail(e);
    lhp_stage_direct(e);
    lhp_stage_inverse(e);
    e->fill = 0;
}

int lhp_push_s16(lhp_engine *e, const int16_t *in, int n, int16_t *out)
{
    if (n > LHP_BLOCK - e->fill) n = LHP_BLOCK - e->fill;
    lhp_stage_input_s16(e, in, n);
    if (e->fill < LHP_BLOCK) return 0;
    if (!e->loaded) { memset(out, 0, LHP_BLOCK * 4); e->fill = 0; return LHP_BLOCK; }
    run_block(e);
    lhp_stage_output_s16(e, out);
    return LHP_BLOCK;
}

int lhp_push_f32(lhp_engine *e, const float *in, int n, float *out)
{
    if (n > LHP_BLOCK - e->fill) n = LHP_BLOCK - e->fill;
    stage_input_f32(e, in, n);
    if (e->fill < LHP_BLOCK) return 0;
    if (!e->loaded) { memset(out, 0, LHP_BLOCK * 8); e->fill = 0; return LHP_BLOCK; }
    run_block(e);
    lhp_stage_output_f32(e, out);
    return LHP_BLOCK;
}
