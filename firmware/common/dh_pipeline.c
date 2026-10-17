#include "dh_pipeline.h"
#include <string.h>
#include <math.h>

#define IN_FRAMES   (DHP_IN_BLOCKS * LHP_BLOCK)
/* Free-running 32-bit counters index the rings, so sizes must divide 2^32. */
_Static_assert((IN_FRAMES & (IN_FRAMES - 1)) == 0, "DHP_IN_BLOCKS must be a power of 2");
_Static_assert((DHP_OUT_FRAMES & (DHP_OUT_FRAMES - 1)) == 0, "DHP_OUT_FRAMES must be a power of 2");
_Static_assert(DHP_TARGET_FILL + LHP_BLOCK <= IN_FRAMES, "target fill too large for in ring");
_Static_assert(DHP_TARGET_FILL >= 3 * LHP_BLOCK / 2, "rooms mode needs at least 1.5 blocks of buffering");
_Static_assert(DHP_TARGET_FILL_LL >= DHP_LL_CHUNK, "low-latency target below one chunk");

#if defined(__GNUC__)
#define LOAD_ACQ(x)      __atomic_load_n(&(x), __ATOMIC_ACQUIRE)
#define STORE_REL(x, v)  __atomic_store_n(&(x), (v), __ATOMIC_RELEASE)
#else
#define LOAD_ACQ(x)      (x)
#define STORE_REL(x, v)  ((x) = (v))
#endif

static const int8_t k_map_usb51[LHP_NPOS] = DHP_MAP_USB51;
static const int8_t k_map_usb71[LHP_NPOS] = DHP_MAP_USB71;

static const dhp_filter_t *find_filter(const dhp_t *p, uint32_t rate, int room)
{
    for (int i = 0; i < p->nfilters; i++)
        if (p->filters[i].rate == rate && p->filters[i].room == room) return &p->filters[i];
    return 0;
}

static int load_room(dhp_t *p, int room)
{
    const dhp_filter_t *f = find_filter(p, 48000, room);
    if (!f) return -1;
    if (lhp_load_filter(p->eng, f->blob, f->len, 1.0f)) return -2;
    lhp_reset(p->eng);
    p->room = room;
    return 0;
}

int dhp_init(dhp_t *p, lhp_engine *eng, const dhp_filter_t *filters, int nfilters,
             uint32_t rate, int initial_room)
{
    memset(p, 0, sizeof *p);
    (void)rate;
    p->eng = eng;
    p->filters = filters;
    p->nfilters = nfilters;
    p->gain = 1.0f;
    p->lowlat_req = -1;
    p->beep_freq = 880.0f;
    p->nch = DHP_IN_CH;
    lhp_init(eng);
    lhp_set_map(eng, k_map_usb51, DHP_IN_CH);
    if (initial_room < 1 || initial_room > 3) initial_room = 1;
    return load_room(p, initial_room);
}

/* ------------------------------------------------------------------ USB */
static inline uint32_t in_level(const dhp_t *p)  { return LOAD_ACQ(p->in_w) - LOAD_ACQ(p->in_r); }
uint32_t dhp_out_available(const dhp_t *p)        { return LOAD_ACQ(p->out_w) - LOAD_ACQ(p->out_r); }

uint32_t dhp_usb_frames_wanted(dhp_t *p)
{
    const uint32_t target = LOAD_ACQ(p->lowlat) ? DHP_TARGET_FILL_LL : DHP_TARGET_FILL;
    uint32_t fill = in_level(p) + dhp_out_available(p);
    if (fill >= target) return 0;
    uint32_t want = target - fill;
    uint32_t space = IN_FRAMES - in_level(p);
    return want < space ? want : space;
}

uint32_t dhp_usb_frames_wanted_guarded(dhp_t *p, uint32_t usb_backlog, uint32_t high, uint32_t mid)
{
    uint32_t want = dhp_usb_frames_wanted(p);
    if (usb_backlog > high) {
        uint32_t spill = usb_backlog - mid;
        uint32_t space = IN_FRAMES - in_level(p);
        if (spill > space) spill = space;
        if (spill > want) want = spill;
    }
    return want;
}

uint32_t dhp_usb_write(dhp_t *p, const int16_t *frames, uint32_t n)
{
    uint32_t w = p->in_w;
    uint32_t space = IN_FRAMES - (w - LOAD_ACQ(p->in_r));
    if (n > space) { p->in_overruns++; n = space; }
    const int nch = p->nch;
    uint32_t idx = w & (IN_FRAMES - 1);
    uint32_t first = IN_FRAMES - idx;                    /* two copies at most */
    if (first > n) first = n;
    memcpy(&p->in_buf[idx * nch], frames, first * nch * sizeof(int16_t));
    if (n > first) memcpy(p->in_buf, &frames[first * nch], (n - first) * nch * sizeof(int16_t));
    STORE_REL(p->in_w, w + n);
    return n;
}

int dhp_set_channels(dhp_t *p, int nch)
{
    if (nch != 6 && nch != 8) return -1;
    if (nch > DHP_IN_CH_MAX) return -1;
    p->nch = nch;
    if (nch == 8) { lhp_set_map(p->eng, k_map_usb71, 8); lhp_set_map_back(p->eng, 4, 5); }
    else          lhp_set_map(p->eng, k_map_usb51, 6);
    p->in_r = p->in_w = 0;
    if (p->room) return load_room(p, p->room);           /* builds the back pair's filters */
    return 0;
}
int dhp_channels(const dhp_t *p) { return p->nch; }

void dhp_set_usb_volume(dhp_t *p, int16_t vol_db_q8, bool mute)
{
    float db = (float)vol_db_q8 / 256.0f;
    if (db > 0) db = 0;
    p->gain = powf(10.0f, db / 20.0f);
    p->mute = mute;
}

/* ------------------------------------------------------------------ control */
void dhp_request_room(dhp_t *p, int room) { if (room >= 1 && room <= 3) STORE_REL(p->room_req, room); }
int dhp_cycle_room(dhp_t *p)
{
    int cur = LOAD_ACQ(p->room_req);          /* a pending request counts as current */
    if (!cur) cur = LOAD_ACQ(p->room);
    int next = cur % 3 + 1;
    dhp_request_room(p, next);
    return next;
}
int dhp_room(const dhp_t *p) { return p->room; }

void dhp_set_low_latency(dhp_t *p, bool on) { STORE_REL(p->lowlat_req, on ? 1 : 0); }
bool dhp_toggle_low_latency(dhp_t *p)
{
    int req = LOAD_ACQ(p->lowlat_req);
    bool cur = req >= 0 ? req : LOAD_ACQ(p->lowlat);
    dhp_set_low_latency(p, !cur);
    return !cur;
}
bool dhp_low_latency(const dhp_t *p) { return LOAD_ACQ(p->lowlat); }

bool dhp_surround_is_factory(const lhp_surround_t *s)
{
    return s->near_delay == 0.0f && s->far_hf_db == 0.0f && s->back_eq == 0.0f;
}
void dhp_preset(int id, lhp_surround_t *sur, lhp_surround_t *back)
{
    if (id == DHP_PRESET_THEATER) { *sur = LHP_SURROUND_REAR; *back = lhp_surround_for_azimuth(150.0f); }
    else                          { *sur = LHP_SURROUND_FACTORY; *back = LHP_SURROUND_FACTORY; }
}
int dhp_preset_of(const lhp_surround_t *sur, const lhp_surround_t *back)
{
    for (int id = DHP_PRESET_FACTORY; id <= DHP_PRESET_THEATER; id++) {
        lhp_surround_t s, b;
        dhp_preset(id, &s, &b);
        if (lhp_surround_equal(sur, &s) && lhp_surround_equal(back, &b)) return id;
    }
    return DHP_PRESET_CUSTOM;
}
const char *dhp_preset_name(int id)
{
    return id == DHP_PRESET_FACTORY ? "factory" : id == DHP_PRESET_THEATER ? "theater" : "custom";
}
void dhp_set_placement(dhp_t *p, const lhp_surround_t *sur, const lhp_surround_t *back, bool beep)
{
    const uint32_t seq = p->sur_seq;
    STORE_REL(p->sur_seq, seq + 1);                     /* odd: writing */
    __atomic_thread_fence(__ATOMIC_RELEASE);
    p->sur_req = *sur;
    p->back_req = *back;
    p->sur_beep = beep;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    STORE_REL(p->sur_seq, seq + 2);
}
void dhp_get_placement(dhp_t *p, lhp_surround_t *sur, lhp_surround_t *back)
{
    if (sur) *sur = p->sur_req;
    if (back) *back = p->back_req;
}
void dhp_set_placement_now(dhp_t *p, const lhp_surround_t *sur, const lhp_surround_t *back)
{
    p->sur = p->sur_req = *sur;
    p->back = p->back_req = *back;
    lhp_set_surround(p->eng, sur);
    lhp_set_back(p->eng, back);
    if (p->room) load_room(p, p->room);
}
void dhp_set_surround(dhp_t *p, const lhp_surround_t *s, bool beep) { dhp_set_placement(p, s, &p->back_req, beep); }
void dhp_get_surround(dhp_t *p, lhp_surround_t *out) { *out = p->sur_req; }
void dhp_set_surround_now(dhp_t *p, const lhp_surround_t *s) { dhp_set_placement_now(p, s, &p->back_req); }
const char *dhp_room_name(int room)
{
    switch (room) { case 1: return "Small room"; case 2: return "Medium room";
                    case 3: return "Large room"; default: return "?"; }
}

/* ------------------------------------------------------------------ DSP */
#define BEEP_ON   (48000 * 70 / 1000)    /* 70 ms tone  */
#define BEEP_OFF  (48000 * 90 / 1000)    /* 90 ms gap   */
#define BEEP_LEAD (48000 * 60 / 1000)    /* wait for new room to settle */

static void add_beeps(dhp_t *p, float *buf, int n)
{
    if (!p->beep_left) return;
    const float w = 2.0f * 3.14159265f * p->beep_freq / 48000.0f, amp = 0.12f * 32768.0f;
    for (int i = 0; i < n && p->beep_left; i++) {
        uint32_t t = p->beep_pos++;
        if (t >= BEEP_LEAD) {
            uint32_t ph = (t - BEEP_LEAD) % (BEEP_ON + BEEP_OFF);
            if (ph < BEEP_ON) {
                /* 5 ms raised-cosine edges to avoid clicks */
                float env = 1.0f;
                const uint32_t edge = 240;
                if (ph < edge) env = 0.5f - 0.5f * cosf(3.14159265f * ph / edge);
                else if (ph > BEEP_ON - edge) env = 0.5f - 0.5f * cosf(3.14159265f * (BEEP_ON - ph) / edge);
                float s = amp * env * sinf(p->beep_phase);
                buf[2 * i] += s; buf[2 * i + 1] += s;
                p->beep_phase += w;
                if (p->beep_phase > 6.2831853f) p->beep_phase -= 6.2831853f;
            }
            if (ph == BEEP_ON + BEEP_OFF - 1) p->beep_left--;
        }
    }
}

static inline int16_t sat16(float v)
{
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lrintf(v);
}

static void start_beeps(dhp_t *p, int n, float freq)
{
    p->beep_left = n;
    p->beep_pos = 0;
    p->beep_phase = 0;
    p->beep_freq = freq;
}

static void emit(dhp_t *p, int n)               /* gain, beeps, fbuf -> out ring */
{
    const float g = p->mute ? 0.0f : p->gain;
    for (int i = 0; i < 2 * n; i++) p->fbuf[i] *= g;
    add_beeps(p, p->fbuf, n);
    uint32_t w = p->out_w;
    for (int i = 0; i < n; i++) {
        uint32_t idx = ((w + i) & (DHP_OUT_FRAMES - 1)) * 2;
        p->out_buf[idx]     = sat16(p->fbuf[2 * i]);
        p->out_buf[idx + 1] = sat16(p->fbuf[2 * i + 1]);
    }
    STORE_REL(p->out_w, w + n);
}

static void apply_mode_change(dhp_t *p)
{
    int req = LOAD_ACQ(p->lowlat_req);
    if (req < 0) return;
    if ((bool)req != p->lowlat) {
        if (req) {
            /* Enter low latency: throw away the deep buffers in one go. */
            STORE_REL(p->in_r, LOAD_ACQ(p->in_w));
            STORE_REL(p->out_flush_req, p->out_flush_req + 1);
            start_beeps(p, 2, 1320.0f);
        } else {
            /* Back to rooms: restart the engine and pad the output with
             * silence so the sink has slack while the deeper buffer refills. */
            lhp_reset(p->eng);
            uint32_t pad = DHP_TARGET_FILL - LHP_BLOCK;
            uint32_t space = DHP_OUT_FRAMES - dhp_out_available(p);
            if (pad > space) pad = space;
            uint32_t w = p->out_w;
            for (uint32_t i = 0; i < pad; i++) {
                uint32_t idx = ((w + i) & (DHP_OUT_FRAMES - 1)) * 2;
                p->out_buf[idx] = p->out_buf[idx + 1] = 0;
            }
            STORE_REL(p->out_w, w + pad);
            start_beeps(p, p->room, 880.0f);
        }
        STORE_REL(p->lowlat, (bool)req);
    }
    int expect = req;
    __atomic_compare_exchange_n(&p->lowlat_req, &expect, -1, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

static inline void downmix(const int16_t *f, float *lr, bool stereo, int nch)
{
    if (stereo) { lr[0] = (float)f[0]; lr[1] = (float)f[1]; return; }   /* stereo source: pass through */
    /* FL FR FC LFE BL BR [SL SR] -> L R, -6 dB overall to leave headroom */
    float c = 0.7071f * ((float)f[2] + (float)f[3]);
    float sl = (float)f[4], sr = (float)f[5];
    if (nch == 8) { sl += (float)f[6]; sr += (float)f[7]; }
    lr[0] = 0.5f * ((float)f[0] + c + 0.7071f * sl);
    lr[1] = 0.5f * ((float)f[1] + c + 0.7071f * sr);
}

bool dhp_dsp_poll(dhp_t *p)
{
    int req = LOAD_ACQ(p->room_req);
    if (req) {
        if (req != p->room && load_room(p, req) == 0 && !p->lowlat)
            start_beeps(p, req, 880.0f);         /* 1, 2 or 3 beeps */
        /* clear only if nobody asked for another room while we were loading */
        int expect = req;
        __atomic_compare_exchange_n(&p->room_req, &expect, 0, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
    }
    apply_mode_change(p);
    const uint32_t seq = LOAD_ACQ(p->sur_seq);
    if (seq != p->sur_applied && !(seq & 1)) {
        lhp_surround_t s = p->sur_req, b = p->back_req;
        const bool beep = p->sur_beep;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (LOAD_ACQ(p->sur_seq) == seq) {              /* not rewritten meanwhile */
            p->sur_applied = seq;
            p->sur = s;
            p->back = b;
            lhp_set_surround(p->eng, &s);
            lhp_set_back(p->eng, &b);
            if (p->room && load_room(p, p->room) == 0 && beep && !p->lowlat)
                start_beeps(p, dhp_surround_is_factory(&s) && dhp_surround_is_factory(&b) ? 1 : 2, 660.0f);
        }
    }

    const uint32_t r = p->in_r;
    const uint32_t level = in_level(p);
    const uint32_t space = DHP_OUT_FRAMES - dhp_out_available(p);

    if (p->lowlat) {
        uint32_t n = level < DHP_LL_CHUNK ? level : DHP_LL_CHUNK;
        if (!n) return false;
        if (space < n) {
            if (!space) { STORE_REL(p->in_r, r + n); p->out_drops++; return false; }
            n = space;
        }
        const int nch = p->nch;
        for (uint32_t i = 0; i < n; i++)
            downmix(&p->in_buf[((r + i) & (IN_FRAMES - 1)) * nch], &p->fbuf[2 * i], p->stereo_input, nch);
        STORE_REL(p->in_r, r + n);
        emit(p, (int)n);
        p->blocks++;
        return true;
    }

    if (level < LHP_BLOCK) return false;
    if (space < LHP_BLOCK) {
        /* nobody is draining us: drop the block so the input keeps moving */
        STORE_REL(p->in_r, r + LHP_BLOCK);
        p->out_drops++;
        return false;
    }

    /* The block is contiguous unless it straddles the end of the ring. */
    const uint32_t off = r & (IN_FRAMES - 1);
    const uint32_t first = off + LHP_BLOCK > IN_FRAMES ? IN_FRAMES - off : LHP_BLOCK;
    lhp_engine *e = p->eng;
    e->fill = 0;
    lhp_stage_input_s16(e, &p->in_buf[off * p->nch], (int)first);
    if (first < LHP_BLOCK) lhp_stage_input_s16(e, p->in_buf, (int)(LHP_BLOCK - first));
    STORE_REL(p->in_r, r + LHP_BLOCK);           /* input copied into the engine */
    lhp_stage_tail(e);
    lhp_stage_direct(e);
    lhp_stage_inverse(e);
    e->fill = 0;
    lhp_stage_output_f32(e, p->fbuf);
    emit(p, LHP_BLOCK);
    p->blocks++;
    return true;
}

/* ------------------------------------------------------------------ sink */
uint32_t dhp_read_stereo(dhp_t *p, int16_t *dst, uint32_t n)
{
    uint32_t fr = LOAD_ACQ(p->out_flush_req);
    if (fr != p->out_flush_ack) {                 /* DSP switched to low latency */
        STORE_REL(p->out_r, LOAD_ACQ(p->out_w));
        p->out_flush_ack = fr;
    }
    uint32_t r = p->out_r;
    uint32_t avail = LOAD_ACQ(p->out_w) - r;
    uint32_t k = n < avail ? n : avail;
    for (uint32_t i = 0; i < k; i++) {
        uint32_t idx = ((r + i) & (DHP_OUT_FRAMES - 1)) * 2;
        dst[2 * i] = p->out_buf[idx];
        dst[2 * i + 1] = p->out_buf[idx + 1];
    }
    if (k < n) {
        memset(&dst[2 * k], 0, (n - k) * 2 * sizeof(int16_t));
        p->out_underruns++;
    }
    STORE_REL(p->out_r, r + k);
    return k;
}
