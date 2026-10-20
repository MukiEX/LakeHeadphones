#!/usr/bin/env python3
"""
make_open_filters.py - build open-licence room filters for the lakehp engine
from a measured HRTF set (SOFA, e.g. the SONICOM database), and package them
as a stand-in DLL that every LakeHP tool can import like the original one.

  python3 tools/make_open_filters.py p0010.sofa -o LakeHP_OpenRooms.dll

What it makes, per room (Small / Medium / Large) and rate (48 kHz, 44.1 kHz):
  - direct filters (1024 taps) for L, C, R, Ls, Rs: the measured head-related
    impulse response for that speaker plus the room's early reflections
    (image-source model, each reflection with the HRIR of its own direction),
    diffuse-field equalized for headphones;
  - a shared late tail (6 x 1024 taps per ear): binaural diffuse reverb built
    from many directions' HRIRs, with frequency-dependent decay.
They are written in the original filter-blob format, inside a valid PE32 DLL
(no code; just the blobs, a version resource and the licence notice). The
flasher page, extract_filters.py, the Jellyfin and neko filters all scan any
"MZ" file for blobs, so this DLL drops in wherever the original one goes.

The HRTF data is the user's choice of SOFA file; for SONICOM it is licensed
CC BY-SA 3.0 (Imperial College London), so the generated filters carry the
same licence and attribution.

Needs numpy, scipy, h5py.
"""
import argparse, math, struct, sys, os, time
import numpy as np
import h5py
from scipy.signal import resample_poly

FS = 48000
BLOCK = 1024
NPART = 6
C_SOUND = 343.0

# Speaker azimuths, SOFA convention (counter-clockwise from the front, left positive)
POS_AZ = {"L": 30.0, "C": 0.0, "R": -30.0, "Ls": 120.0, "Rs": -120.0}
POS_ORDER = ["L", "C", "R", "Ls", "Rs"]

# Rooms. dims: room size (m); dist: speaker distance; refl: wall reflection
# coefficient (pressure) at mid frequencies; order: image-source order;
# rt: late reverb RT60 (s) at 125, 500, 2k, 8k Hz; d2t: energy of the direct
# filter (0-21 ms) over the late tail, for the front-left speaker, in dB.
# Proportions follow what the original rooms measure (dry, short tails:
# -30 dB at 75-100 ms) so switching between the two sets stays comfortable.
ROOMS = {
    1: dict(name="Open Small",  dims=(4.2, 3.8, 2.6), dist=1.4, refl=0.62, order=3,
            rt=(0.20, 0.17, 0.14, 0.09), d2t=13.0),
    2: dict(name="Open Medium", dims=(6.5, 5.2, 2.9), dist=2.0, refl=0.28, order=2,
            rt=(0.30, 0.26, 0.21, 0.13), d2t=15.0),
    3: dict(name="Open Large",  dims=(9.5, 7.5, 3.6), dist=2.6, refl=0.92, order=3,
            rt=(0.21, 0.19, 0.15, 0.10), d2t=10.0),
}
LISTEN_Z = 1.2

# Tail partitions kept per frequency bin (engine budget: sum <= 4096 slots).
# Bins 0..511 run from DC to Nyquist; 6 partitions up to ~6 kHz, then fewer.
TAIL_CUTS = [512, 384, 320, 256, 192, 128]


# --------------------------------------------------------------------------- HRTF set
class HRTFSet:
    def __init__(self, path):
        f = h5py.File(path, "r")
        self.fs = float(f["Data.SamplingRate"][()][0])
        if abs(self.fs - FS) > 1:
            sys.exit(f"{path}: HRIRs must be 48 kHz (got {self.fs})")
        self.ir = np.array(f["Data.IR"][()], dtype=np.float64)        # M x 2 x N
        pos = np.array(f["SourcePosition"][()], dtype=np.float64)
        self.az, self.el = pos[:, 0], pos[:, 1]
        a, e = np.radians(self.az), np.radians(self.el)
        self.vec = np.stack([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], 1)
        self.attrs = {k: (v.decode("utf-8", "replace") if isinstance(v, bytes) else str(v))
                      for k, v in f.attrs.items()}
        self.name = os.path.splitext(os.path.basename(path))[0]
        self.eq = None

    def nearest(self, az, el=0.0):
        a, e = math.radians(az), math.radians(el)
        v = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
        return int(np.argmax(self.vec @ v))

    def hrir(self, az, el=0.0):
        i = self.nearest(az, el)
        h = self.ir[i]
        return np.stack([np.convolve(h[0], self.eq), np.convolve(h[1], self.eq)])


def third_octave_smooth(f, p, frac=3):
    out = np.empty_like(p)
    for i, f0 in enumerate(f):
        if f0 <= 0:
            out[i] = p[i]; continue
        m = (f >= f0 * 2 ** (-0.5 / frac)) & (f <= f0 * 2 ** (0.5 / frac))
        out[i] = p[m].mean()
    return out


def min_phase(mag, n_out):
    """Minimum-phase impulse response from a magnitude on an rfft grid."""
    nfft = (len(mag) - 1) * 2
    cep = np.fft.irfft(np.log(np.maximum(mag, 1e-6)), nfft)
    w = np.zeros(nfft); w[0] = 1; w[1:nfft // 2] = 2; w[nfft // 2] = 1
    h = np.fft.irfft(np.exp(np.fft.rfft(cep * w)), nfft)[:n_out]
    fade = n_out // 4
    h[-fade:] *= 0.5 + 0.5 * np.cos(np.linspace(0, np.pi, fade))
    return h


def diffuse_field_eq(hs):
    """Headphone EQ: inverse of the (solid-angle weighted) diffuse-field power
    average over all directions and both ears, 1/3-octave smoothed, limited to
    +-15 dB (+8 dB above 16 kHz), unity at 1 kHz, rolled off below 40 Hz."""
    nfft = 4096
    f = np.fft.rfftfreq(nfft, 1 / FS)
    w = np.cos(np.radians(hs.el))
    P = (np.abs(np.fft.rfft(hs.ir, nfft, axis=2)) ** 2 * w[:, None, None]).sum((0, 1)) / (2 * w.sum())
    P = third_octave_smooth(f, P)
    g = 1 / np.sqrt(P)
    g /= g[np.argmin(abs(f - 1000))]
    gdb = 20 * np.log10(g)
    lim = np.where(f > 16000, 8.0, 15.0)
    gdb = np.clip(gdb, -15, lim)
    # below 40 Hz roll off at 12 dB/octave: the measurements carry no information
    # there, and a boost reaching DC would only add inaudible sub-bass
    lo = f < 40
    g40 = gdb[np.argmin(abs(f - 40))]
    gdb[lo] = g40 - 12 * np.log2(40 / np.maximum(f[lo], 1))
    gdb[f > 20000] = np.minimum(gdb[f > 20000], gdb[np.argmin(abs(f - 20000))] - (f[f > 20000] - 20000) / 1000 * 6)
    hs.eq = min_phase(10 ** (gdb / 20), 512)
    hs.eq_db = (f, gdb)


# --------------------------------------------------------------------------- room model
def frac_delay_add(dst, h, t, g):
    """dst += g * h delayed by t samples (fractional, 4-point Lagrange)."""
    n0 = int(math.floor(t)); fr = t - n0
    c = [-fr * (fr - 1) * (fr - 2) / 6, (fr + 1) * (fr - 1) * (fr - 2) / 2,
         -(fr + 1) * fr * (fr - 2) / 2, (fr + 1) * fr * (fr - 1) / 6]
    for k, ck in enumerate(c):
        s = n0 - 1 + k
        if s < 0 or s >= len(dst): continue
        m = min(len(h), len(dst) - s)
        dst[s:s + m] += g * ck * h[:m]


def wall_filter(order, n=257):
    """Air + wall absorption for `order` bounces: a gentle HF loss per bounce."""
    f = np.linspace(0, FS / 2, n)
    db = -order * (1.5 * (f / 4000) ** 1.3).clip(0, 9)
    return min_phase(10 ** (db / 20), 64) if order else None


def direct_filter(hs, room, az_spk):
    """Direct sound + early reflections for one speaker, first 1024 samples."""
    Lx, Ly, Lz = room["dims"]
    lis = np.array([0.55 * Lx, Ly / 2, LISTEN_Z])
    a = math.radians(az_spk)
    src = lis + room["dist"] * np.array([math.cos(a), math.sin(a), 0.0])
    out = np.zeros((2, BLOCK + 512))
    t0 = 8.0                                         # keep the HRIR onset where it is, plus a little
    r0 = room["dist"]
    wf = {}
    N = room["order"]
    for ux in (0, 1):
        for uy in (0, 1):
            for uz in (0, 1):
                for lx in range(-N, N + 1):
                    for ly in range(-N, N + 1):
                        for lz in range(-N, N + 1):
                            o = abs(lx - ux) + abs(lx) + abs(ly - uy) + abs(ly) + abs(lz - uz) + abs(lz)
                            if o > N: continue
                            img = np.array([(1 - 2 * ux) * src[0] + 2 * lx * Lx,
                                            (1 - 2 * uy) * src[1] + 2 * ly * Ly,
                                            (1 - 2 * uz) * src[2] + 2 * lz * Lz])
                            v = img - lis
                            r = float(np.linalg.norm(v))
                            dt = (r - r0) / C_SOUND * FS
                            if dt + 300 > BLOCK - 16: continue
                            azr = math.degrees(math.atan2(v[1], v[0]))
                            elr = math.degrees(math.asin(v[2] / r))
                            h = hs.hrir(azr, elr)
                            g = (r0 / r) * (room["refl"] ** o)
                            if o:
                                if o not in wf: wf[o] = wall_filter(o)
                                h = np.stack([np.convolve(h[0], wf[o]), np.convolve(h[1], wf[o])])
                            for ear in (0, 1):
                                frac_delay_add(out[ear], h[ear], t0 + dt, g)
    d = out[:, :BLOCK].copy()
    fade = 64
    d[:, -fade:] *= 0.5 + 0.5 * np.cos(np.linspace(0, np.pi, fade))
    return d


def late_tail(hs, room, rng, ndir=96):
    """Binaural diffuse tail, samples BLOCK .. 7*BLOCK (6 partitions per ear)."""
    n = BLOCK * (NPART + 1)
    acc = np.zeros((2, n + 1024))
    # directions spread over the sphere (Fibonacci), each with its own noise
    gold = math.pi * (3 - math.sqrt(5))
    for i in range(ndir):
        z = 1 - 2 * (i + 0.5) / ndir
        az = math.degrees(i * gold) % 360
        el = math.degrees(math.asin(z))
        if el < -45: el = -45 + (el + 90) * 0.2               # the set stops at -45 degrees
        h = hs.hrir(az, el)
        x = rng.standard_normal(n)
        for ear in (0, 1):
            y = np.convolve(x, h[ear])[:n + 1024]
            acc[ear, :len(y)] += y
    acc = acc[:, :n] / math.sqrt(ndir)
    # frequency-dependent decay: octave-ish bands with their own RT60
    t = np.arange(n) / FS
    edges = [0, 250, 1000, 4000, FS / 2]
    rts = room["rt"]
    nfft = 1 << (n - 1).bit_length() + 1
    f = np.fft.rfftfreq(nfft, 1 / FS)
    out = np.zeros((2, n))
    for b in range(4):
        lo, hi = edges[b], edges[b + 1]
        # smooth complementary band masks (raised-cosine crossovers, half an octave wide)
        def ramp(fc):
            if fc <= 0: return np.ones_like(f)
            if fc >= FS / 2: return np.zeros_like(f)
            x = np.clip((np.log2(np.maximum(f, 1)) - np.log2(fc)) / 0.5 + 0.5, 0, 1)
            return 0.5 + 0.5 * np.cos(np.pi * x)
        mask = ramp(hi) - ramp(lo) if lo > 0 else ramp(hi)
        env = np.exp(-6.9078 * t / rts[b])
        for ear in (0, 1):
            band = np.fft.irfft(np.fft.rfft(acc[ear], nfft) * mask, nfft)[:n]
            out[ear] += band * env
    tail = out[:, BLOCK:].copy()                       # starts after the direct filter
    rise = 96
    tail[:, :rise] *= np.linspace(0, 1, rise)
    fall = BLOCK                                       # fade over the last partition
    tail[:, -fall:] *= 0.5 + 0.5 * np.cos(np.linspace(0, np.pi, fall))
    return tail


# --------------------------------------------------------------------------- blob format
class BitWriter:
    def __init__(self): self.d = bytearray(); self.bit = 0
    def u8(self, v): self.d.append(v & 0xFF); self.bit = 0
    def u16(self, v): self.u8(v); self.u8(v >> 8)
    def bits(self, v, n):                              # LSB-first
        for i in range(n):
            if self.bit == 0: self.d.append(0)
            self.d[-1] |= ((v >> i) & 1) << self.bit
            self.bit = (self.bit + 1) & 7


MAXBITS = 20


def put_part(w, x, scale_exp):
    """One 1024-sample partition: 64 groups of 16, per-group exponent (5 bits)."""
    q = np.rint(np.asarray(x) * (1 << (scale_exp - 1))).astype(np.int64)
    lim = (1 << (MAXBITS - 1)) - 1
    q = np.clip(q, -lim, lim)
    for g in range(0, BLOCK, 16):
        v = q[g:g + 16]
        m = int(np.max(np.abs(v)))
        e = 0 if m == 0 else max(2, int(m).bit_length() + 1)     # signed, e bits
        w.bits(e, 5)
        for s in v:
            if e: w.bits(int(s) & ((1 << e) - 1), e)


def make_blob(room_no, rate_code, name, direct, tail, gains):
    """direct: 5 x 2 x 1024, tail: 2 x 6144 (float, unit = full scale)."""
    peak = max(np.max(np.abs(direct)), np.max(np.abs(tail)))
    # samples decode as int * 2^-(scale_exp-1); put the peak near the top of MAXBITS
    scale_exp = int(math.floor(MAXBITS - 1 - math.log2(peak))) + 1
    scale_exp = max(1, min(31, scale_exp))
    nm = name.encode("ascii") + b"\0"
    w = BitWriter()
    w.u8(2); w.u16(0); w.u16(0)                        # version, total length (patched)
    w.u16(70 + len(nm))                                # header length
    w.u8(0); w.u8(0); w.u8(room_no); w.u8(rate_code)
    w.u8(0x1F)                                         # tail + direct L/R, C, Ls, Rs
    w.u8(MAXBITS); w.u8(scale_exp)
    for g in gains: w.u16(int(round(g * 16384)) & 0xFFFF)
    cuts = [c // 2 for c in TAIL_CUTS]
    for i in range(12): w.u16(cuts[i // 2] if i % 2 == 0 else 0)
    for _ in range(10): w.u16(0)
    w.u8(0); w.u8(len(nm) - 1)
    for c in nm: w.u8(c)
    for ear in (0, 1):
        for i in range(NPART):
            put_part(w, tail[ear][i * BLOCK:(i + 1) * BLOCK], scale_exp)
    for p in range(5):
        for ear in (0, 1):
            put_part(w, direct[p][ear], scale_exp)
    for _ in range(8): w.u8(0)                         # the decoder reads ahead
    n = len(w.d)
    w.d[1:5] = struct.pack("<I", n)
    return bytes(w.d)


# --------------------------------------------------------------------------- PE container
def build_pe(blobs, notice, version, product, description):
    """A minimal valid PE32 DLL: no code, one .rdata section with the notice and
    the blobs, and a .rsrc section with a VERSIONINFO resource."""
    def align(x, a): return (x + a - 1) // a * a
    FA, SA = 0x200, 0x1000
    rdata = bytearray(notice.encode("utf-8") + b"\0")
    while len(rdata) % 16: rdata.append(0)
    for b in blobs:
        rdata += b
        while len(rdata) % 16: rdata.append(0)

    # ---- VS_VERSIONINFO
    def ws(s): return s.encode("utf-16le") + b"\0\0"
    def node(key, value=b"", vtype=1, children=b"", vlen=None):
        hdr_key = ws(key)
        body = bytearray()
        head_len = 6 + len(hdr_key)
        pad1 = (-(head_len)) % 4
        body += b"\0" * pad1 + value
        pad2 = (-(len(value))) % 4 if children else 0
        body += b"\0" * pad2 + children
        total = head_len + len(body)
        if vlen is None: vlen = len(value) // 2 if vtype == 1 else len(value)
        out = struct.pack("<HHH", total, vlen, vtype) + hdr_key + body
        return out + b"\0" * ((-len(out)) % 4)
    vparts = [int(x) for x in version.split(".")] + [0, 0, 0, 0]
    ms = (vparts[0] << 16) | vparts[1]; ls = (vparts[2] << 16) | vparts[3]
    ffi = struct.pack("<13I", 0xFEEF04BD, 0x00010000, ms, ls, ms, ls, 0x3F, 0, 0x4, 0x2, 0, 0, 0)
    strings = [("CompanyName", "LakeHP project"), ("FileDescription", description),
               ("FileVersion", version), ("InternalName", "LakeHP_OpenRooms"),
               ("LegalCopyright", "HRTF data: SONICOM, Imperial College London, CC BY-SA 3.0"),
               ("OriginalFilename", "LakeHP_OpenRooms.dll"), ("ProductName", product),
               ("ProductVersion", version), ("Comments", notice.splitlines()[0])]
    st = b"".join(node(k, ws(v)) for k, v in strings)
    sfi = node("StringFileInfo", children=node("040904B0", children=st), vtype=1, vlen=0)
    vfi = node("VarFileInfo", children=node("Translation", struct.pack("<HH", 0x0409, 0x04B0), vtype=0, vlen=4),
               vtype=1, vlen=0)
    vsinfo = node("VS_VERSION_INFO", ffi, vtype=0, children=sfi + vfi, vlen=len(ffi))

    # ---- resource directory: type 16 (RT_VERSION) / id 1 / lang 0x409
    def rdir(entries):  # entries: (id, offset|flag)
        return struct.pack("<IIHHHH", 0, 0, 0, 0, 0, len(entries)) + b"".join(struct.pack("<II", i, o) for i, o in entries)
    d1 = rdir([(16, 0x80000000 | 0x18)])
    d2 = rdir([(1, 0x80000000 | 0x30)])
    d3 = rdir([(0x409, 0x48)])
    rsrc_hdr = d1 + d2 + d3                       # 0x18 + 0x18 + 0x18 = 0x48
    data_entry_off = len(rsrc_hdr)                 # 0x48
    data_off = data_entry_off + 16

    n_sec = 2
    hdr_size = align(0x80 + 4 + 20 + 224 + 40 * n_sec, FA)
    rdata_raw = align(len(rdata), FA)
    rdata_rva = SA
    rsrc_rva = align(rdata_rva + len(rdata), SA)
    rsrc = bytearray(rsrc_hdr)
    rsrc += struct.pack("<IIII", rsrc_rva + data_off, len(vsinfo), 0, 0)
    rsrc += vsinfo
    rsrc_raw = align(len(rsrc), FA)
    image_size = align(rsrc_rva + len(rsrc), SA)

    dos = bytearray(0x80)
    dos[0:2] = b"MZ"
    struct.pack_into("<H", dos, 0x02, 0x90); struct.pack_into("<H", dos, 0x04, 3)
    struct.pack_into("<H", dos, 0x08, 4); struct.pack_into("<H", dos, 0x0C, 0xFFFF)
    struct.pack_into("<H", dos, 0x10, 0xB8); struct.pack_into("<H", dos, 0x18, 0x40)
    struct.pack_into("<I", dos, 0x3C, 0x80)
    stub = b"\x0e\x1f\xba\x0e\x00\xb4\x09\xcd\x21\xb8\x01\x4c\xcd\x21This program cannot be run in DOS mode.\r\r\n$"
    dos[0x40:0x40 + len(stub)] = stub
    coff = struct.pack("<HHIIIHH", 0x14C, n_sec, int(time.time()), 0, 0, 224, 0x2102)  # i386, DLL|32BIT|EXEC
    dirs = [(0, 0)] * 16
    dirs[2] = (rsrc_rva, len(rsrc))
    opt = struct.pack("<HBBIIIIIIIIIHHHHHHIIIIHHIIIIII",
                      0x10B, 14, 0, 0, len(rdata) + len(rsrc), 0, 0, 0, rdata_rva,
                      0x10000000, SA, FA, 6, 0, 1, 0, 6, 0, 0, image_size, hdr_size, 0,
                      2, 0x0540, 0x100000, 0x1000, 0x100000, 0x1000, 0, 16)
    opt += b"".join(struct.pack("<II", a, s) for a, s in dirs)
    def sec(name, vsize, rva, rsize, rptr, chars):
        return struct.pack("<8sIIIIIIHHI", name, vsize, rva, rsize, rptr, 0, 0, 0, 0, chars)
    secs = sec(b".rdata", len(rdata), rdata_rva, rdata_raw, hdr_size, 0x40000040)
    secs += sec(b".rsrc", len(rsrc), rsrc_rva, rsrc_raw, hdr_size + rdata_raw, 0x40000040)
    head = bytes(dos) + b"PE\0\0" + coff + opt + secs
    head += b"\0" * (hdr_size - len(head))
    return head + bytes(rdata) + b"\0" * (rdata_raw - len(rdata)) + bytes(rsrc) + b"\0" * (rsrc_raw - len(rsrc))


# --------------------------------------------------------------------------- main
def build_room(hs, room_no, seed):
    R = ROOMS[room_no]
    rng = np.random.default_rng(seed)
    direct = np.stack([direct_filter(hs, R, POS_AZ[p]) for p in POS_ORDER])
    tail = late_tail(hs, R, rng)
    # tail level: direct-filter energy of the front-left speaker over tail energy
    ed = float((direct[0] ** 2).sum())
    et = float((tail ** 2).sum())
    tail *= math.sqrt(ed / et / 10 ** (R["d2t"] / 10))
    return direct, tail


def to_rate(direct, tail, rate):
    if rate == 48000: return direct, tail
    up, dn = 147, 160
    d2 = np.stack([[resample_poly(direct[p][e], up, dn)[:BLOCK] for e in (0, 1)] for p in range(5)])
    d2 = np.pad(d2, ((0, 0), (0, 0), (0, BLOCK - d2.shape[2])))
    full = [np.concatenate([np.zeros(BLOCK), tail[e]]) for e in (0, 1)]
    t2 = np.stack([resample_poly(full[e], up, dn)[BLOCK:BLOCK * (NPART + 1)] for e in (0, 1)])
    t2 = np.pad(t2, ((0, 0), (0, BLOCK * NPART - t2.shape[1])))
    fall = BLOCK
    t2[:, -fall:] *= 0.5 + 0.5 * np.cos(np.linspace(0, np.pi, fall))
    return d2, t2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sofa", help="48 kHz SimpleFreeFieldHRIR SOFA file (e.g. SONICOM pXXXX.sofa)")
    ap.add_argument("-o", "--out", default="LakeHP_OpenRooms.dll")
    ap.add_argument("--gain", type=float, nargs=3, default=None, metavar=("S", "M", "L"),
                    help="output gain per room (linear); default: the calibrated values below")
    ap.add_argument("--bins", help="also write the raw blobs to this directory")
    a = ap.parse_args()

    hs = HRTFSet(a.sofa)
    diffuse_field_eq(hs)
    gains = a.gain or ROOM_GAIN
    blobs = []
    for room_no in (1, 2, 3):
        direct, tail = build_room(hs, room_no, seed=1000 + room_no)
        direct = direct * gains[room_no - 1]; tail = tail * gains[room_no - 1]
        for rate, code in ((48000, 0), (44100, 1)):
            d, t = to_rate(direct, tail, rate)
            if rate == 44100:
                d = d * RATE441_TRIM[room_no - 1]; t = t * RATE441_TRIM[room_no - 1]
            b = make_blob(room_no, code, ROOMS[room_no]["name"], d, t, [1.0] * 5)
            blobs.append(b)
            if a.bins:
                os.makedirs(a.bins, exist_ok=True)
                fn = f"lhp_filter_open{room_no}_{ROOMS[room_no]['name'].split()[1].lower()}_{rate}.bin"
                open(os.path.join(a.bins, fn), "wb").write(b)
            print(f"room {room_no} {ROOMS[room_no]['name']:12s} {rate} Hz: {len(b)} bytes", file=sys.stderr)
    lic = hs.attrs.get("License", "see the SOFA file")
    notice = (f"LakeHP open room filters, built from HRTF set '{hs.name}' "
              f"({hs.attrs.get('DatabaseName', '')}, {hs.attrs.get('Organization', '')}).\n"
              f"HRTF data licence: {lic}. These filters are a derivative of that data and "
              f"carry the same licence. Contains no code and no data from any other DLL.\n")
    dll = build_pe(blobs, notice, "1.0.0.0", "LakeHP open room filters",
                   f"LakeHP open room filters (SONICOM {hs.name})")
    open(a.out, "wb").write(dll)
    print(f"wrote {a.out}: {len(dll)} bytes, {len(blobs)} filters", file=sys.stderr)


# Output gain per room, calibrated so each room plays at the same loudness as
# the original's room of the same size: K-weighted (BS.1770) level of 5.1 pink
# noise, fronts full, surrounds -3 dB, LFE -6 dB. Only these level numbers were
# measured against the original; none of its data is used.
ROOM_GAIN = (10 ** (-0.50 / 20), 10 ** (0.43 / 20), 10 ** (-2.47 / 20))
# extra gain for the 44.1 kHz versions, matched the same way against the
# original's 44.1 kHz rooms
RATE441_TRIM = (10 ** (0.72 / 20), 10 ** (0.69 / 20), 10 ** (0.72 / 20))

if __name__ == "__main__":
    main()
