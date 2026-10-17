#include "resampler.h"
#include <math.h>
#include <string.h>

static double bessel_i0(double x)
{
    double s = 1, t = 1;
    for (int k = 1; k < 30; k++) { t *= (x / (2 * k)) * (x / (2 * k)); s += t; }
    return s;
}

void rs_init(resampler_t *rs, double in_rate, double out_rate)
{
    memset(rs, 0, sizeof *rs);
    rs->step = rs->step_nominal = in_rate / out_rate;
    /* low-pass at 0.45 * min(in,out) */
    double fc = 0.45 * (in_rate < out_rate ? in_rate : out_rate) / in_rate;  /* cycles/input sample */
    const double beta = 7.0, i0b = bessel_i0(beta);
    for (int ph = 0; ph <= RS_PHASES; ph++) {
        double frac = (double)ph / RS_PHASES;
        double sum = 0;
        for (int k = 0; k < RS_TAPS; k++) {
            /* tap k sits at input offset (k - (RS_TAPS/2 - 1)) relative to floor(pos) */
            double x = (double)(k - (RS_TAPS / 2 - 1)) - frac;
            double sinc = (x == 0) ? 2 * fc : sin(2 * M_PI * fc * x) / (M_PI * x);
            double r = x / (RS_TAPS / 2.0);
            double w = (fabs(r) <= 1) ? bessel_i0(beta * sqrt(1 - r * r)) / i0b : 0;
            rs->table[ph][k] = (float)(sinc * w);
            sum += sinc * w;
        }
        for (int k = 0; k < RS_TAPS; k++) rs->table[ph][k] = (float)(rs->table[ph][k] / sum);
    }
}

void rs_reset(resampler_t *rs)
{
    memset(rs->hist, 0, sizeof rs->hist);
    rs->hpos = 0;
    rs->primed = 0;
    rs->frac = 0;
    rs->step = rs->step_nominal;
}

void rs_trim_ppm(resampler_t *rs, double ppm) { rs->step = rs->step_nominal * (1.0 + ppm * 1e-6); }

static inline void push(resampler_t *rs, float l, float r)
{
    rs->hist[0][rs->hpos] = rs->hist[0][rs->hpos + RS_TAPS] = l;
    rs->hist[1][rs->hpos] = rs->hist[1][rs->hpos + RS_TAPS] = r;
    rs->hpos = (rs->hpos + 1) % RS_TAPS;
    if (rs->primed < RS_TAPS) rs->primed++;
}

static inline int16_t sat(float v)
{
    if (v > 32767.f) return 32767;
    if (v < -32768.f) return -32768;
    return (int16_t)lrintf(v);
}

/* History holds the last RS_TAPS input frames; the newest is at hpos-1.
 * Output is interpolated at (newest - RS_TAPS/2) + frac, i.e. the filter is
 * centred with RS_TAPS/2 samples of look-ahead (constant latency). */
int rs_process(resampler_t *rs, const int16_t *in, int n_in, int *n_used,
               int16_t *out, int n_out_max)
{
    int used = 0, made = 0;
    while (made < n_out_max) {
        /* need input up to floor(pos)+RS_TAPS/2: advance while frac >= 1 */
        for (;;) {
            int priming = rs->primed < RS_TAPS;
            if (!priming && rs->frac < 1.0) break;
            if (used >= n_in) goto done;
            push(rs, in[2 * used], in[2 * used + 1]);
            used++;
            if (!priming) rs->frac -= 1.0;
        }
        double fp = rs->frac * RS_PHASES;
        int ph = (int)fp;
        float a = (float)(fp - ph);
        const float *t0 = rs->table[ph], *t1 = rs->table[ph + 1];
        const float *h0 = &rs->hist[0][rs->hpos], *h1 = &rs->hist[1][rs->hpos]; /* oldest first */
        float l = 0, r = 0;
        for (int k = 0; k < RS_TAPS; k++) {
            float c = t0[k] + a * (t1[k] - t0[k]);
            l += c * h0[k];
            r += c * h1[k];
        }
        out[2 * made] = sat(l);
        out[2 * made + 1] = sat(r);
        made++;
        rs->frac += rs->step;
    }
done:
    if (n_used) *n_used = used;
    return made;
}
