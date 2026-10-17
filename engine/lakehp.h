/*
 * lakehp - portable reimplementation of the Lake Technology "Dolby Headphone"
 * engine (DolbyHph.dll 1.20.0276, exports LakeOpen/LakeLoadFilter/LakeProcess),
 * reconstructed from disassembly for use on microcontrollers.
 *
 * This file contains NO filter data. Room filters (DH1/DH2/DH3) are loaded at
 * runtime from blobs that tools/extract_filters.py pulls out of *your own*
 * copy of the DLL.
 *
 * Signal flow (per 1024-frame block):
 *   input channels -> LFE folded into L/R (or C)
 *   each position p in {L,C,R,Ls,Rs} (+ Lb,Rb for 7.1, an addition):
 *       X_p = FFT1024( (prev_p + j*new_p) * e^{j*pi*k/2048} )     lapped transform
 *       OutL += X_p * HdirL_p ; OutR += X_p * HdirR_p              direct HRTF
 *       Mix  += X_p * g_p                                          room send
 *   OutL/R += sum_i Mix[n-1-i] * Htail_i   (per-bin partition count 1..6)
 *   y = Im( IFFT1024(Out) * e^{-j*pi*k/2048} )  -> 1024 stereo frames
 *
 * Latency: 1024 frames (one block) in block mode.
 */
#ifndef LAKEHP_H
#define LAKEHP_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LHP_BLOCK      1024          /* frames per processing block            */
#define LHP_NPOS       6             /* L, C, R, Ls, Rs, LFE                   */
#define LHP_MAXPART    6             /* max tail partitions per bin            */
#define LHP_MAXCH      8             /* max interleaved input channels (7.1)   */
#define LHP_NDIR       7             /* direct filter sets: L,C,R,Ls,Rs,Lb,Rb  */

/* Packed tail storage: sum over bins of the per-bin partition count. Every
 * filter in DolbyHph 1.20.0276 needs <= 4096 coefficient slots and <= 3072
 * history slots (worst case would be 6*1024 / 5*1024). lhp_load_filter()
 * rejects filters that do not fit. */
#ifndef LHP_TAIL_MAX
#define LHP_TAIL_MAX   4096
#endif
#ifndef LHP_HIST_MAX
#define LHP_HIST_MAX   3072
#endif

/* Speaker positions inside the engine */
enum { LHP_POS_L = 0, LHP_POS_C, LHP_POS_R, LHP_POS_LS, LHP_POS_RS, LHP_POS_LFE };
/* 7.1 back pair (an addition): direct filter sets 5 and 6, built from the
 * surround HRTFs with their own placement (see lhp_set_back). */
enum { LHP_DIR_LB = 5, LHP_DIR_RB = 6 };

/* Coefficient storage type. Define LHP_COEF_HALF to store filter spectra as
 * IEEE half floats (halves RAM for the two big coefficient tables; requires a
 * compiler with _Float16, e.g. GCC for ARM/Xtensa/x86). */
#ifdef LHP_COEF_HALF
typedef _Float16 lhp_coef_t;
#else
typedef float lhp_coef_t;
#endif

typedef struct { float re, im; } lhp_cpx;
typedef struct { lhp_coef_t lr, li, rr, ri; } lhp_cpx2;   /* left-ear + right-ear */

/* Parsed filter header (fields named where their use is known). */
typedef struct {
    int      version;        /* 1 or 2                                         */
    uint32_t total_len;      /* blob length in bytes                           */
    uint16_t header_len;
    int      hdr3, hdr4;     /* unknown / informational                        */
    int      room;           /* 1..3 = DH1..DH3, 0 = stereo-mixdown filter     */
    uint32_t sample_rate;    /* 0xFFFFFFFF = any                               */
    int      flags;          /* bit0: direct HRTFs present, bits1..4: tail parts */
    int      max_bits;       /* sample bit width cap                           */
    int      scale_exp;      /* samples scaled by 2^-(scale_exp-1)             */
    float    pos_gain[5];    /* room-send gain per position (L,C,R,Ls,Rs)      */
    uint16_t tail_cut[12];   /* tail cut-off bins (even entries used)          */
    float    misc[10];       /* mixdown matrix etc. (unused by DH filters)     */
    int      hdr37;
    char     name[64];
} lhp_filter_info;

/* Whole engine state. ~330 KB with float coefficients, ~240 KB with half.
 * Allocate statically; it is large. */
typedef struct {
    /* configuration */
    int8_t   map[LHP_NPOS];      /* position -> input channel index, -1 = none */
    int8_t   map_back[2];        /* 7.1 back left/right input channel, -1 = none */
    int      nch;                /* interleaved input channels                 */
    float    gain;               /* filter load gain                           */
    struct lhp_surround {        /* speaker-pair placement, see lhp_set_surround() */
        float near_delay;        /* near-ear delay in samples (shrinks the ITD), -12..24;
                                  * negative delays the far ear (moves forward)          */
        float far_hf_db;         /* far-ear gain above 2 kHz in dB (+ shrinks the ILD)   */
        float back_eq;           /* "behind" spectral cues, -1..2, 0 = none, 1 = standard */
    } sur, back;                 /* surround (Ls/Rs) and 7.1 back (Lb/Rb) pairs          */
    int      back_fold;          /* back pair placed like the surrounds: mixed into them */
    int      loaded;
    lhp_filter_info info;

    /* per-bin tail partition counts (effective, after RLE emulation) */
    uint8_t  cnt[LHP_BLOCK];
    uint16_t hist_total;         /* sum(cnt-1)                                 */
    uint16_t coef_total;         /* sum(cnt)                                   */

    /* tables */
    lhp_cpx  fft_fwd[LHP_BLOCK / 2];
    lhp_cpx  fft_inv[LHP_BLOCK / 2];
    lhp_cpx  rot[LHP_BLOCK];     /* 32768*e^{j*pi*k/2048}, for filter loading  */
    uint16_t win[LHP_BLOCK + 1]; /* trunc(32768*sin(pi*k/2048)), win[1024]=0   */

    /* signal state (inputs are 16-bit; float input is rounded on entry) */
    int16_t  inbuf[2][LHP_MAXCH][LHP_BLOCK];  /* double-buffered input      */
    int      cur;                /* index of buffer being filled               */
    int      fill;               /* frames in current buffer                   */

    lhp_cpx  X[LHP_BLOCK];       /* per-channel spectrum scratch               */
    lhp_cpx  mix[LHP_BLOCK];     /* room send spectrum (consumed next block)   */
    lhp_cpx  outL[LHP_BLOCK];
    lhp_cpx  outR[LHP_BLOCK];
    lhp_cpx  hist[LHP_HIST_MAX];  /* tail input history (room send)             */

    /* filters */
    lhp_cpx2 dir[LHP_NDIR][LHP_BLOCK]; /* direct HRTF spectra L,C,R,Ls,Rs,Lb,Rb */
    lhp_cpx2 tail[LHP_TAIL_MAX];  /* packed per-bin tail partitions             */
} lhp_engine;

/* ---- setup ---- */
void lhp_init(lhp_engine *e);                       /* tables + 5.1 map       */
void lhp_reset(lhp_engine *e);                      /* clear signal history   */

/* AC-3 style channel configuration: acmod (0..7) + 8 if LFE present.
 *  15 = 3/2+LFE with input order L C R Ls Rs LFE (DLL native)                */
int  lhp_set_acmod(lhp_engine *e, int cfg);
/* Custom map: map[pos] = input channel index or -1. pos order L,C,R,Ls,Rs,LFE.
 * For USB/WAVE 5.1 (L R C LFE Ls Rs) use {0,2,1,4,5,3} with nch=6.           */
int  lhp_set_map(lhp_engine *e, const int8_t map[LHP_NPOS], int nch);
/* 7.1 (an addition): input channels of the back pair, or -1/-1 for none.
 * Call after lhp_set_map; takes effect at the next lhp_load_filter().
 * Windows 7.1 order (FL FR FC LFE BL BR SL SR): map {0,2,1,6,7,3}, back 4, 5. */
int  lhp_set_map_back(lhp_engine *e, int lb, int rb);

/* Parse only the header of a filter blob. Returns 0 on success. */
int  lhp_parse_header(const uint8_t *blob, size_t len, lhp_filter_info *out);
/* Decode + transform a filter into the engine. gain=1.0 matches the DLL.
 * Returns 0 on success, <0 on error (bad blob / unsupported mixdown filter). */
int  lhp_load_filter(lhp_engine *e, const uint8_t *blob, size_t len, float gain);

/* Speaker placement - an addition, not in the DLL. Applied to the direct
 * HRTFs at the next lhp_load_filter(); the room tail is unchanged.
 * {0,0,0} = the factory surround position (about 118 degrees).
 * LHP_SURROUND_REAR (6, 4, 1) moves a pair to about 136 degrees.
 * The 7.1 back pair starts from the same surround HRTFs with its own
 * placement; with the same placement as the surrounds it costs nothing
 * extra (it is mixed into the surround inputs). */
typedef struct lhp_surround lhp_surround_t;
#define LHP_SURROUND_FACTORY ((lhp_surround_t){ 0.0f, 0.0f, 0.0f })
#define LHP_SURROUND_REAR    ((lhp_surround_t){ 6.0f, 4.0f, 1.0f })
void lhp_set_surround(lhp_engine *e, const lhp_surround_t *s);
void lhp_set_back(lhp_engine *e, const lhp_surround_t *s);
void lhp_set_surround_rear(lhp_engine *e, float amount);   /* amount x LHP_SURROUND_REAR */
/* Rough azimuth a placement corresponds to (118 = factory, 136 = rear preset),
 * and the placement for an azimuth (90..165 degrees). */
float lhp_surround_azimuth(const lhp_surround_t *s);
lhp_surround_t lhp_surround_for_azimuth(float deg);
int   lhp_surround_equal(const lhp_surround_t *a, const lhp_surround_t *b);

/* ---- processing ----
 * Push frames; whenever 1024 frames have accumulated a block is processed and
 * written to out (interleaved stereo). Returns number of output frames written
 * (0 or 1024). nframes must be <= space left in the block (1024 - e->fill). */
int  lhp_push_s16(lhp_engine *e, const int16_t *in, int nframes, int16_t *out);
int  lhp_push_f32(lhp_engine *e, const float *in, int nframes, float *out); /* +-32768 scale */

/* Convenience: process exactly one 1024-frame block. */
static inline void lhp_block_s16(lhp_engine *e, const int16_t *in, int16_t *out)
{ e->fill = 0; lhp_push_s16(e, in, LHP_BLOCK, out); }

/* Lower-level stage access (used by dual-core firmware builds). */
void lhp_stage_input_s16(lhp_engine *e, const int16_t *in, int nframes);
void lhp_stage_tail(lhp_engine *e);          /* tail conv -> outL/outR        */
void lhp_stage_direct(lhp_engine *e);        /* fwd FFTs, direct, mix         */
void lhp_stage_inverse(lhp_engine *e);       /* IFFT outL/outR                */
void lhp_stage_output_s16(lhp_engine *e, int16_t *out);
void lhp_stage_output_f32(lhp_engine *e, float *out);

void lhp_fft(lhp_cpx *d, const lhp_cpx *tw);  /* in-place 1024 radix-2 DIT     */

#ifdef __cplusplus
}
#endif
#endif
