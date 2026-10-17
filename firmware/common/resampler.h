/*
 * resampler - small stereo polyphase windowed-sinc resampler with an
 * adjustable ratio (for clock-drift tracking). int16 in/out, float inside.
 *
 * Push model:  rs_process(rs, in, n_in, &used, out, n_out_max) consumes up to
 * n_in input frames and produces up to n_out_max output frames.
 */
#ifndef RESAMPLER_H
#define RESAMPLER_H
#include <stdint.h>

#define RS_TAPS    32            /* filter length in input samples */
#define RS_PHASES  64            /* table phases (linear interpolation between) */

typedef struct {
    double step;                 /* input frames advanced per output frame */
    double step_nominal;
    double frac;                 /* fractional position in [0,1)           */
    float  hist[2][RS_TAPS * 2]; /* doubled ring for contiguous reads      */
    int    hpos;                 /* write position in [0, RS_TAPS)         */
    int    primed;               /* number of input frames in history      */
    float  table[RS_PHASES + 1][RS_TAPS];
} resampler_t;

void rs_init(resampler_t *rs, double in_rate, double out_rate);  /* builds tables: slow, not for audio callbacks */
void rs_reset(resampler_t *rs);                                   /* clears history/phase/trim: cheap */
/* Trim the ratio by `ppm` parts-per-million (positive = consume input faster). */
void rs_trim_ppm(resampler_t *rs, double ppm);
int  rs_process(resampler_t *rs, const int16_t *in, int n_in, int *n_used,
                int16_t *out, int n_out_max);
#endif
