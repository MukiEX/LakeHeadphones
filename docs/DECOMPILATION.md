# DolbyHph.dll: what's inside

The uploaded `randoapp.dll` is **Lake Technology's Dolby Headphone engine**:

| | |
|---|---|
| Original name | `DolbyHph.dll` (export table name `dolbyhph.dll`) |
| Version | 1.20.0276, internal name `DHW98-x86-01` |
| Built | 2004-11-08, MSVC 6, PE32 i386 DLL |
| Vendor | Lake Technology Ltd (acquired by Dolby in 2004) |

The engine is about 30 KB of code. The other ~550 KB is 22 compressed filter sets.

## Exports (stdcall)

| Export | RVA | What it does |
|---|---|---|
| `LakeOpen()` | 0x40c0 | Clears state, picks a CPU code path (x87 / MMX / SSE via CPUID), builds tables, applies the default mode, loads the default filter from the registry |
| `LakeSetMode(mode*)` | 0x3ed0 | 0x24-byte struct: `{size, chan_cfg, cpu, ?, custom_map*, gain f32, out_variant, out_scale f32, rate f32}` |
| `LakeLoadFilter(type, ptr, idx, ?)` | 0x3b90 | type 0 = built-in table index, 1 = file, 2 = memory blob. Gated by a licence-acceptance registry check |
| `LakeProcess(in, out, ring_start, ring_end, flags)` | 0x43d0 | Wrapper for `LakeDH` with a 1024-frame request. flags bit 0 = int16 in, bit 1 = int16 out |
| `LakeDH(...)` | 0x4248 | Streaming core. Buffers input until 1024 frames, then runs one block |
| `LakeFlush()` | 0x420c | Clears history, input buffers and levels |
| `LakeGetMode / LakeGetFilterInfo / LakeFindFilter / LakeClose` | | Housekeeping and registry lookup |

## Channel configuration (table @ 0x100a1f80)

`chan_cfg` is the **AC-3 `acmod` + 8 × `lfeon`**. Each entry is `{0, nch, map[6]}`, where `map[pos]` gives the input channel for each internal position. The positions are, in order, **L, C, R, Ls, Rs, LFE**. Config 15 (3/2 + LFE) expects input order L C R Ls Rs LFE. A mono surround channel (acmod 4/5) is fed to both Ls and Rs.

## Built-in filters (table @ 0x100a1c80, 22 × 12 bytes: `{blob*, rate, room}`)

- Index 0 is a "Stereo mixdown" (room 0). It uses a separate code path and is not ported.
- Indices 1–21 cover **DH1, DH2 and DH3 at 48k, 44.1k, 32k, 22.05k, 16k, 11.025k and 8k**.

DH1, DH2 and DH3 are the Small, Medium and Large rooms.

### Blob format

All multi-byte fields are little-endian.

```
s8  version (1|2)        u32 total_len           u16 header_len
s8  ?  s8 ?  s8 room     s8 rate_code            s8 flags
s8  max_bits             s8 scale_exp (v2 only)
q14 pos_gain[5]          (room-send gain per position, /16384)
u16 tail_cut[12]         (even entries used: per-bin partition cut-offs / 2)
q14 misc[10]             (mixdown matrix, unused by DH filters)
s8  ?  s8 name_len-1     char name[]     ("DH1", "DH2", "DH3", "Stereo mixdown")
--- bitstream (LSB-first), groups of 16 samples:
    e = bits(4|5) + (v1 ? 1 : 0); nb = min(e, max_bits); shift = e - nb
    16 × sample = sext(bits(nb)) << shift, scaled by 2^-(scale_exp-1)
--- order: if flags&1: tail[ear 0..1][partition 0..5]  (1024 samples each)
           then direct[pos L,C,R,Ls,Rs][ear 0..1], each gated by flags bit {1,2,1,3,4}
```

## DSP (x87 reference path: table @ 0x100a1eb8)

Everything runs on blocks of N = 1024 frames, with one block of latency.

1. **Input (0x10001e10).** De-interleaves into a double-buffered `float[2][6][1024]`. The buffer is indexed by *input* channel.
2. **LFE fold (0x10001f00).** LFE is added into the L and R input buffers, or into C if L/R are missing.
3. **Lapped forward transform (0x10001d38).** For each distinct input channel:
   `X[k] = FFT1024( (prev[k] + j·new[k]) · W[k] )`, where `W[k] = 32768·e^{jπk/2048}`. W comes from an integer sin table (`win[k] = trunc(32768·sin(πk/2048))`).
   The FFT is a plain radix-2 DIT: bit-reverse first, natural-order output, unscaled.
4. **Direct path (0x100024dc).** Per position p:
   - `OutL += X·HdL[p]`, `OutR += X·HdR[p]`
   - `Mix += X·gain[p]`
5. **Room tail (0x10002144).** This runs *before* step 4 and uses the **previous** block's `Mix`.
   - Each bin k has its own partition count c(k) between 1 and 6. It is largest at low frequencies and symmetric, `c(k) = c(1023-k)`.
   - It is derived from `tail_cut` and stored as a run-length list that changes by at most one step per bin.
   - The convolution: `Out = Σ_{i<c(k)} Mix[n-1-i]·Ht[k][c-1-i]`
   - The history lives in one packed array of size `Σ(c-1)`. The coefficients are packed with size `Σ c`, which is 3992 at 44.1/48 kHz and 4096 at 32 kHz.
6. **Inverse.**
   - Run IFFT1024 on `OutL` and `OutR`.
   - `y[k] = Im( Y[k] · 32768·e^{-jπk/2048} )`
   - Round to nearest (x87 `fistp`) and saturate to int16.

Filter spectra are produced with the same lapped transform as the audio:
`H = FFT( h[k]·2^-55·gain·W[k] )`.
The port stores them ×2^40, an exact power-of-two shift so they fit in FP16, and takes the 2^40 back out at the output.

The MMX/3DNow and SSE paths (tables @ 0x100a20b0, 0x100a2140, 0x100a21f8) compute the same thing.

## Verification

The original DLL was run on Linux through a freestanding i386 PE loader written for this (not shipped). The loader stubs the Win32 imports and calls the internal blob loader directly to skip the registry licence gate. Its output was then compared with `engine/lakehp.c`:

| Test | Max |error| | SNR vs DLL |
|---|---|---|
| DH1/DH2/DH3 @ 48k, 5.1 impulses + noise | 1 LSB | 101–104 dB |
| DH1 @ 44.1k stereo, DH3 @ 44.1k 5.0, DH3 @ 48k dual-mono | 1 LSB | 101–103 dB |
| `LHP_COEF_HALF` (FP16 coefficients) | 3 LSB | 72 dB |

The ±1 LSB is rounding: x87 computes in extended precision, while the port uses single precision.
