# LakeHP technical breakdown

This document describes the final configuration: a 5.1/7.1 USB headphone virtualizer that sends its output to Bluetooth headphones.

**Contents**

1. [What it is](#1-what-it-is)
2. [The engine (decompilation and port)](#2-the-engine-decompilation-and-port)
3. [Filter data handling](#3-filter-data-handling)
4. [Speaker placement](#4-speaker-placement)
5. [7.1 support](#5-71-support)
6. [Pipeline](#6-pipeline)
7. [Pico 2 W firmware architecture](#7-pico-2-w-firmware-architecture)
8. [USB device](#8-usb-device)
9. [Bluetooth](#9-bluetooth)
10. [Controls, LED and beeps](#10-controls-led-and-beeps)
11. [Settings page](#11-settings-page)
12. [Stored settings](#12-stored-settings)
13. [Flasher page and tools](#13-flasher-page-and-tools)
14. [Other builds](#14-other-builds)
15. [Memory and CPU budget](#15-memory-and-cpu-budget)
16. [Verification status](#16-verification-status)
17. [File map](#17-file-map)

---

## 1. What it is

A microcontroller that appears to a PC as a USB 5.1 or 7.1 sound card.
- It renders the speaker channels to binaural stereo with a C port of the Lake Technology headphone engine (DLL version 1.20.0276), which has three rooms: Small, Medium and Large.
- It sends the result to Bluetooth headphones.

```
PC / console --USB 5.1/7.1 (UAC2, 48 kHz)--> [ DSP: virtual rooms ] --A2DP/SBC--> headset
             --USB stereo (UAC1)---------->  (stereo mode, for consoles)
```

The primary build is a single **Raspberry Pi Pico 2 W** board: an RP2350 plus a CYW43439, which supports Classic Bluetooth. Two other builds share the same engine and pipeline ([section 14](#14-other-builds)).

---

## 2. The engine (decompilation and port)

### Source DLL

- Original name `DolbyHph.dll` (uploaded as `randoapp.dll`).
- Version 1.20.0276, internal name `DHW98-x86-01`.
- Built 2004-11-08 with MSVC 6 as a 32-bit x86 Windows DLL (PE32 i386).
- About 30 KB of code plus about 550 KB of compressed filter sets (22 blobs).
- Exports: `LakeOpen`, `LakeSetMode`, `LakeLoadFilter`, `LakeProcess`, `LakeDH`, `LakeFlush`, `LakeGetMode`, `LakeGetFilterInfo`, `LakeFindFilter`, `LakeClose`.

### Filter blobs

- Index 0 is a "stereo mixdown" filter. It uses a separate code path and is not ported.
- Indices 1–21 hold the rooms DH1, DH2 and DH3 at 48, 44.1, 32, 22.05, 16, 11.025 and 8 kHz. Only the 48 kHz set is used.
- The header contains:
  - version, length, room, rate code, flags, max bits, scale exponent;
  - five room-send gains (Q14) and 12 tail cut-offs;
  - a mixdown matrix and a name.
- The body is an LSB-first bitstream in groups of 16 samples. Each group has its own exponent.
- Order inside the blob:
  1. The room tail, as `[ear 0..1][partition 0..5]` with 1024 samples each.
  2. The direct HRTFs, as `[position L, C, R, Ls, Rs][ear 0..1]`.

Full format: [DECOMPILATION.md](DECOMPILATION.md).

### Signal flow

The engine processes one 1024-frame block at a time, with one block of latency.

1. **Input.** De-interleave into a double buffer, indexed by input channel.
2. **LFE fold.** Add LFE into L and R, or into C if L/R are missing.
3. **Lapped forward transform.** For each distinct input channel:
   `X[k] = FFT1024( (prev[k] + j·new[k]) · 32768·e^{jπk/2048} )`.
   The window comes from an integer sine table: `trunc(32768·sin(πk/2048))`.
4. **Room tail.** This runs first, using the previous block's room send.
   - Each frequency bin has its own partition count, from 1 to 6. Counts are largest at low frequencies.
   - Coefficients and history are stored packed.
   - All speaker positions share the tail.
5. **Direct path.** For each position p:
   - `OutL += X·HdirL[p]`
   - `OutR += X·HdirR[p]`
   - `Mix += X·send_gain[p]`
6. **Inverse.**
   - IFFT1024 of OutL and OutR.
   - Compute `y = Im(Y · 32768·e^{-jπk/2048})`.
   - Round and saturate to int16.

The FFT is a plain radix-2 decimation-in-time transform with bit-reversed input, natural-order output and no scaling.

### Port (`engine/lakehp.c`, `lakehp.h`)

- **Code:** plain C99. It contains no filter data, and the engine state is allocated statically.
- **Filter scaling:** spectra are stored ×2⁴⁰, an exact power-of-two shift. The factor is removed at the output.
- **Input buffers:** int16, double-buffered, up to 8 channels.
  - LFE and folded channels are summed on the fly inside the lapped transform.
  - Integer sums are exact in float, so the result matches the DLL's in-place float fold exactly.
- **FFT shortcut:** butterflies whose twiddle is exactly 1 skip their multiplies.
  - That is about 20% of all butterflies.
  - The output is bit-identical, and the FFT runs about 14% faster.
- **Pico placement:** DSP inner loops run from SRAM (`LHP_PICO_RAMFUNC`).

### Accuracy

- The port was checked against the original DLL, run through a standalone i386 PE loader.
- It is within ±1 LSB (101–104 dB SNR) across all three rooms and several rates and channel layouts.
- The 1 LSB is x87 extended-precision rounding against single precision.
- After all the 7.1 changes and the FFT shortcut, 5.1 output is still bit-identical to the verified port.

---

## 3. Filter data handling

No deliverable contains filter data. The filters always come from the user's own DLL.

**Flash filter region**
- Location: `0x10300000` (flash offset `0x300000`), up to 512 KB. It sits apart from the firmware, so firmware-only updates keep the filters.
- Header (little-endian):
  ```
  magic "LHPF", version 1, count, total bytes, CRC-32 of bytes [20, total)
  count × { rate, room, offset, length }
  ```
- `lhp_filter_region.h` parses it at boot. Error codes: −1 magic, −2 version, −3 size, −4 CRC.

**Without filters,** the firmware still runs as plain low-latency stereo, and the LED blinks 8 times.

**Build-time alternative.** `extract_filters.py` can write `firmware/generated/lakehp_filters.h`, which is git-ignored. When that header exists, the Pico build compiles the filters in; `LHP_FILTERS_EMBEDDED` is detected automatically.

---

## 4. Speaker placement

This is an addition to the engine; the original DLL has no placement controls.

### Measured factory HRTFs

| Speakers | ITD | ILD | Angle |
|---|---|---|---|
| Fronts | 0.23 ms | ~9–10 dB | ~30° |
| Surrounds | 0.50 ms | ~15 dB | ~118° (lateral ~62°) |

The surrounds also carry mild rear spectral cues. On headphones, the surround position tends to sound "beside you".

### Placement model (`lhp_surround_t`)

The model is applied to a speaker pair's time-domain direct HRTFs whenever a filter loads. The room tail is not changed.

| Parameter | Range | Effect |
|---|---|---|
| `near_delay` | −12…+24 samples @ 48 kHz | **Positive:** fractional delay (linear interpolation) on the near ear. The ITD shrinks and the speaker moves further back.<br>**Negative:** delays the far ear instead. The ITD grows and the speaker moves forward. |
| `far_hf_db` | −6…+12 dB | High shelf at 2 kHz on the far ear. A boost shrinks the ILD (further back); a cut grows it. |
| `back_eq` | −1…2 (1 = textbook) | RBJ peaking filters on both ears: +2.5 dB @ 1 kHz (Q 1.2), −3 dB @ 4.5 kHz (Q 0.9), +2 dB @ 11 kHz (Q 1.5). Negative values invert them, giving a front-ish colouring. |

### Angle ↔ parameters

`lhp_surround_for_azimuth()` maps an angle to parameters, and `lhp_surround_azimuth()` maps back.

- The ITD comes from the Woodworth head model: `ITD = (r/c)·(θ + sin θ)`.
  - r = 8.75 cm and c = 343 m/s.
  - θ is the lateral angle, which is 180° − azimuth for rear positions.
- Each parameter is computed from the azimuth:
  - `near_delay = (0.500 ms − ITD(azimuth)) · 48000`
  - `far_hf_db = (azimuth − 118) · 4/18`
  - `back_eq = (azimuth − 118)/18` behind 118°, or `(azimuth − 118)/28` in front of it
- Results are clamped to the ranges above and rounded to the precision shown on the settings page. The angle range is 90–165°.

| Azimuth | Delay (samples) | Far HF | Rear cues | Note |
|---|---|---|---|---|
| 90° | −7.5 | −6.0 dB | −1.00 | |
| 100° | −5.2 | −4.0 dB | −0.64 | |
| 110° | −2.5 | −1.8 dB | −0.29 | |
| **118°** | 0.0 | 0.0 dB | 0.00 | factory |
| **136°** | 6.0 | 4.0 dB | 1.00 | the "rear" placement, confirmed by ear |
| **150°** | 11.5 | 7.1 dB | 1.78 | |
| 165° | 17.6 | 10.4 dB | 2.00 | |

### Presets

| Preset | Surround pair (sides in 7.1) | 7.1 rear pair |
|---|---|---|
| **Factory** | ~118°, filters as made `{0, 0, 0}` | same as the surrounds (mixed in) |
| **Theater** | 136° `{6, 4, 1}` | 150° `{11.5, 7.1, 1.78}` |

Any other values count as **Custom**; the Surround lab sets them.

---

## 5. 7.1 support

USB channel order follows Windows' 7.1 surround layout: **FL FR FC LFE BL BR SL SR**, channel mask `0x63F`.

### Channel mapping

- **Sides (SL/SR, channels 6/7)** feed the engine's surround positions Ls/Rs.
- **Backs (BL/BR, channels 4/5)** feed a new rear pair:
  ```
  map  {L,C,R,Ls,Rs,LFE} = {0, 2, 1, 6, 7, 3}
  back {Lb,Rb}           = {4, 5}
  ```
- The rear pair has its own direct filter sets (sets 5 and 6). They are built from the surround HRTFs with the rear pair's own placement, and they use the surround room-send gains.

### Cost

- **Rear pair at the same placement as the surrounds (Factory):** the back channels are summed into the surround inputs, the same way LFE is folded.
  - This costs nothing extra.
  - It is exactly equal to sending that signal to a 5.1 surround (tested: zero difference).
- **Rear pair at its own placement (Theater or Custom):** this needs two more forward FFTs and accumulations per block, about +25–30% DSP time.

### 5.1 mapping

The 5.1 layout is FL FR FC LFE BL BR, mapped as `{0, 2, 1, 4, 5, 3}`, with no rear pair.

### Switching modes

- Switching between 5.1 and 7.1 happens on the settings page. The Pico saves the choice and restarts as the other device.
- Set Windows' speaker setup to match: *Configure → 7.1 Surround*.

---

## 6. Pipeline

`firmware/common/dh_pipeline.c` holds lock-free single-producer/single-consumer rings:

```
USB --> in ring (4 × 1024 frames × 6 or 8 ch, int16) --> DSP (1024-frame block)
    --> out ring (4096 stereo frames) --> sink (Bluetooth encoder)
```

- **Rate locking.**
  - The USB side only pulls frames to keep the total amount buffered at a fixed target, so it reads at exactly the sink's rate.
  - The USB async feedback endpoint then locks the host to that rate.
  - Target: 2560 frames in rooms mode, 512 in low-latency mode.
- **Guarded pull.** When the USB FIFO backlog passes ¾, extra frames spill into the in-ring's spare space. This covers Bluetooth stalls: in testing, 40 ms stalls every 700 ms lost no USB data.
- **Ring wrap.** A block that crosses the end of the ring is copied in two segments, with no intermediate copy buffer.
- **Low-latency stereo.**
  - Skips the engine and works in 256-frame chunks.
  - Downmix with −6 dB headroom: `L = 0.5·(FL + 0.707·(FC + LFE) + 0.707·S)`, mirrored for R, where `S = BL (+ SL in 7.1)`.
  - In stereo USB mode, FL/FR pass through at unity.
- **Changes from other threads.** Room and placement changes can be requested from any thread and are applied on the DSP thread. Placement changes use a seqlock and reload the room filter.
- **Host volume and mute** (UAC2 feature unit, in 1/256 dB) are applied before the sink.

---

## 7. Pico 2 W firmware architecture

Files are in `firmware/pico2w/`.

### Clocks

- 200 MHz system clock with the core at 1.15 V.
- The CYW43 PIO SPI divider is set to 2 + 171/256, which keeps its bus at the stock speed.

### Core 0

- **USB interrupts.**
  - The TinyUSB task (`tud_task`) runs from a 250 µs repeating-timer interrupt.
  - A shared USBCTRL interrupt handler (lowest order) also runs `tud_task`, so the isochronous OUT endpoint is re-armed immediately.
  - Both handlers share one priority, so `tud_task` never nests.
- **Also in USB context:**
  - the USB → pipeline pump;
  - the media-key report sender;
  - the settings page's USB-side frame exchange;
  - the debug CDC flush.
- **Main loop:**
  - SBC encoding, about 0.45 ms per frame. When the headset uses 44.1 kHz, audio is resampled from 48 kHz first.
  - The button, LED state and settings page (lwIP).
  - Restart handling and the debug status line.

### Core 1

- The virtualizer DSP.
- BTstack and CYW43. Their async-context interrupt is bound to the core that initialises them.
- LED writes. The LED is a CYW43 GPIO, so each write is an SPI transaction.
- A static 16 KB stack.

### Flash safety

- Settings are written with `flash_safe_execute`.
- Core 1 is registered as a multicore-lockout victim.
- To sample BOOTSEL, core 0 locks out core 1 *before* disabling interrupts, in case core 1 is mid flash write.

### Logging

- Core 0 never blocks on logging.
- The debug build (`-DLHP_DEBUG_CDC=1`) adds a USB serial port.
  - printf/LOG0 copy into an 8 KB ring.
  - `dbg_cdc_task` drains the ring from USB context, without waiting for DTR.

### TinyUSB 0.18 fix for RP2

- **Problem:** isochronous endpoints are never closed. Windows opens, closes and reopens the stream, which made the driver panic ("ep already available") and froze the device.
- **Fix:** the close callback aborts the audio endpoints and clears their buffer-control words.

---

## 8. USB device

The code is in `firmware/common/usb_audio51.c`.

### 5.1 / 7.1 mode (UAC2 composite with IAD)

VID `0x1209` / PID `0x0001` is the pid.codes test PID, for personal use only. The interface order is built at run time:

| Order | Function | Interfaces | Notes |
|---|---|---|---|
| 1 | RNDIS settings adapter | 2 | Only when the settings page is on. **Must be first:** Windows' RNDIS driver fails with Code 10 otherwise. |
| 2 | UAC2 audio | 2 | AC + AS (alt 0 / alt 1) |
| 3 | HID media keys | 1 | Consumer control, EP `0x86`, 10 ms |
| 4 | CDC debug log | 2 | Debug builds only |

#### How the configuration is assembled

- Each function's descriptor is written for interface 0, then renumbered as it is appended. The renumbering covers the IAD, interface descriptors, and the CDC call-management and union descriptors.
- TinyUSB receives the audio function's descriptor length at run time, because 5.1 and 7.1 differ in feature-unit size.

#### Audio

- 48 kHz, 16-bit, 6 channels (mask `0x3F`) or 8 channels (mask `0x63F`).
- Async isochronous OUT endpoint `0x01`: 588 bytes in 5.1, 784 bytes in 7.1 (full speed has room for 7.1 at 48 kHz).
- Feedback endpoint `0x81`.
- The feature unit has master mute and volume only (−50…0 dB in 1 dB steps). The clock source is fixed at 48 kHz.
- Software FIFO of 9408 bytes: 16 packets in 5.1, 12 in 7.1. The feedback endpoint keeps it half full.

#### Device identity

- `bcdDevice` changes with the layout so Windows never reuses a cached one:
  - base `0x0100`;
  - `| 0x02` with the network adapter;
  - `| 0x10` in 7.1;
  - `| 0x20` with media keys;
  - stereo mode uses `0x0101`.
- Strings: "LakeHP 5.1" or "LakeHP 7.1", "LakeHP media keys", "LakeHP settings (192.168.20.1)", "LakeHP debug log".

### Host feedback format (auto-detected)

- Windows and Linux use 16.16 in 4 bytes.
- Apple hosts (macOS, iOS, iPadOS) use 10.14 in 3 bytes, with a 3-byte feedback endpoint.
- Detection: a string-descriptor request right after the device descriptor means an Apple host.
- `-DUSB_AUDIO51_MACOS=1` forces the Apple format.

### Stereo mode (UAC1, for consoles such as the Switch / Switch 2)

- **Device:** a plain single-function USB Audio 1.0 stereo speaker at 48 kHz/16-bit.
  - Implemented as an app-level class driver; AC and AS open separately, with no IAD.
  - The adaptive endpoint is 196 bytes.
- **Drift handling:**
  - A 2048-frame ring targeting 576 frames, with a slack of 192.
  - Drift is corrected by dropping or duplicating one frame, at most every 50 ms.
- **Audio path:** the stereo signal plays on the virtual front speakers in the room.
- **Left out in this mode:** the settings adapter and the media keys. The product name is "LakeHP Stereo".

---

## 9. Bluetooth

`firmware/pico2w/bt_a2dp.c` runs on BTstack.

### A2DP source

- **Rates:** SBC at 48 kHz is preferred, because it matches the DSP. If the headset asks for 44.1 kHz:
  - A polyphase windowed-sinc resampler converts the audio (32 taps, 64 phases, with drift trim).
  - Measured quality is 71–75 dB SNR.
- **SBC settings:** all block lengths and subband counts, stereo or joint stereo, bitpool up to 53.
- **Hand-off between cores:** core 0 encodes into 16 × 256-byte slots, and core 1 packs them into RTP packets every 5 ms.
- **Identity:** local name "LakeHP 5.1 xx:xx:…", class of device `0x200408`.

### Pairing

- **Discovery:** when no headset is remembered, the Pico listens in 5.1 s inquiry windows (`INQUIRY_1280MS × 4`).
- **Choice:** at the end of each window it picks the best audio device by score:
  `class · 1000 + RSSI + 128`.
  - Class scores: headphones/headset = 3, portable = 2, speaker/HiFi = 1, hands-free/car = 0.
- **Memory:** the last headset is stored in BTstack's key-value storage (TLV).
- **Recovery:**
  - Reconnect is retried every 3 s.
  - If the headset suspends the stream, the Pico resumes it after 1 s.
  - If the headset releases the stream, the Pico drops the link and reconnects.
- **Connect chime:** G5 (784 Hz) then D6 (1175 Hz), 300 ms in total. It is mixed in at the encoder, so it plays even when the PC is silent.

### AVRCP target: headset buttons as PC media keys

- **Connection:**
  - The Pico registers an AVRCP Target SDP record (category 1, player).
  - It connects AVRCP after A2DP signalling; the headset may connect first.
- **Volume:** there is no Controller record, so headsets keep their own local volume instead of switching to absolute volume.
- **Mapping:** headset button commands (pass-through) become USB HID consumer usages:

| AVRCP operation | HID usage |
|---|---|
| PLAY, PAUSE | `0xCD` Play/Pause |
| STOP | `0xB7` |
| FORWARD / BACKWARD | `0xB5` next / `0xB6` previous |
| FAST_FORWARD / REWIND | `0xB3` / `0xB4` |
| VOLUME_UP / VOLUME_DOWN | `0xE9` / `0xEA` (only headsets that send them) |
| MUTE | `0xE2` |

- **Holding buttons:** press and release are forwarded separately, so holding a button works.
- **Event path:** events pass from core 1 to core 0 through a 16-entry queue with a spin-locked write side. Reports go out from USB context.
- **Stuck keys:** a key held for more than ~3 s without a release is released automatically.
- **Playback status reported to the headset:**
  - PLAYING after PLAY or on connect.
  - PAUSED after PAUSE or STOP.

BTstack resources: 6 L2CAP channels, 4 L2CAP services, 2 AVRCP connections.

---

## 10. Controls, LED and beeps

The button is BOOTSEL, or an optional GPIO button to GND (`-DLHP_BUTTON_GPIO=n`). Multiple presses count as one gesture if they are no more than 350 ms apart.

| Gesture | Action |
|---|---|
| 1 press | Next room: Small → Medium → Large. 1/2/3 beeps at 880 Hz, plus matching LED blinks. |
| 2 presses | Low-latency stereo on/off. On: 2 beeps at 1320 Hz. Off: the room's beeps. |
| 3 presses | USB mode 5.1/7.1 ↔ stereo (UAC1). Saved; 6 blinks, then restart. |
| 4 presses | Placement preset Factory ↔ Theater. Saved. 1 low beep (660 Hz) = Factory, 2 = Theater. |
| 5 presses | Settings-page network adapter on/off. Saved; 5 blinks, then restart. |
| Hold 2 s | Forget the headset and enter pairing mode. |
| Headset buttons | Play/pause, next, previous and so on, as PC media keys. |

- **Beeps:** 70 ms tone and 90 ms gap, with 5 ms raised-cosine edges, after a 60 ms lead-in.
- **No filters loaded:** room and preset presses give 8 blinks instead; only low-latency stereo is available.

| LED | Meaning |
|---|---|
| Fast flicker (100 ms) | Pairing mode |
| Short blip every second | Reconnecting to the remembered headset |
| Slow blink (500 ms) | Connected, idle |
| Solid | Streaming |
| N × (150 ms on/off) | Confirmation blinks |

---

## 11. Settings page

The page lives at **http://192.168.20.1**.

### Transport

- **Network adapter:** an RNDIS USB network adapter, part of the 5.1/7.1 composite device. RNDIS is built into Windows and also works on Linux, but not macOS.
- **lwIP threading:**
  - lwIP runs with `NO_SYS` in core 0's main loop, while TinyUSB stays in interrupt context.
  - Frames pass through one RX buffer (renewed from USB context) and a two-entry TX queue.
- **Addressing:**
  - The device is `192.168.20.1/24`.
  - Its DHCP server hands out `.2` and `.3`, with no router and no DNS, so the PC keeps its internet route.
  - The MAC address is derived from the board ID and is locally administered.
- **HTTP server:** raw TCP, 2 connections, 1536-byte request/response buffers.
  - `GET /` serves the embedded page (`web/index.html`, compiled in as hex).
  - `GET /api/state` returns the JSON state; the page polls it every second.
  - `POST /api/set` takes JSON changes.

### Page

The page uses a dark GP2040-style UI. It has four tabs: Sound, Surround lab, Headset and System.

#### Sound tab
- Arrangement cards for Stereo, 5.1 and 7.1. Each switch asks for confirmation and restarts the Pico.
- Room selector: Small, Medium, Large.
- Low-latency stereo on/off.
- "Save as power-up default".

#### Surround lab tab
- **Preset cards:** Factory, Theater and Custom. Custom is selected automatically when the values match neither preset.
- **Per pair** ("Surround speakers", or "Side speakers" in 7.1, plus "Rear speakers", which is active in 7.1 only):
  - an angle slider, 90–165°;
  - collapsible fine-tune sliders: near-ear delay (−12…24 samples), far-ear treble (−6…12 dB), behind cues (−100…200%).
- **Head diagram:** a top view with all speakers. The surrounds are green and the rear pair amber.
- **Live updates:**
  - Changes apply live, debounced to 150 ms.
  - The page runs the same angle mapping as the engine, so it updates instantly.
- **Save / Back to saved,** with an unsaved-changes indicator.

#### Headset tab
- Bluetooth state, headset address and codec (rate and bitpool).
- "Forget headset and pair".

#### System tab
- USB stream status and packet rate.
- DSP load on core 1 and encoder load on core 0.
- Firmware version.
- Turn the settings adapter off, or restart.

### API (`POST /api/set`)

| Key | Values |
|---|---|
| `room` | 1…3 |
| `lowlat` | 0 / 1 |
| `az`, `delay`, `hf`, `eq` | Surround pair. `az` sets all three. |
| `baz`, `bdelay`, `bhf`, `beq` | 7.1 rear pair |
| `preset` | `"factory"` / `"theater"` |
| `arr` | `"stereo"` / `"51"` / `"71"`. Saved, then the Pico restarts. |
| `action` | `"save"` / `"revert"` / `"forget"` / `"weboff"` / `"reboot"` |

Saving to flash causes a ~50 ms gap in the sound.

---

## 12. Stored settings

Settings are stored in layout version 3: one flash sector at `PICO_FLASH_SIZE_BYTES − 8 × 4096`, well below BTstack's storage. Each word is a u32:

| Word | Content |
|---|---|
| 0 | Magic `"LHPM"` (`0x4D50484C`) |
| 1 | Stereo USB mode |
| 2 | Surround moved (legacy flag) |
| 3 | Layout version = 3 |
| 4–6 | Surround pair: near_delay, far_hf_db, back_eq (float bits) |
| 7 | Settings adapter off |
| 8 | Room 1…3 |
| 9 | Low-latency stereo |
| 10 | 7.1 USB device |
| 11–13 | Rear pair: near_delay, far_hf_db, back_eq |

- **Bad values:** an out-of-range placement falls back to Factory.
- **Older layouts load as Theater:**
  - layout v2 with surrounds at 136°;
  - layout v1 with the "rear" flag set.

---

## 13. Flasher page and tools

### `lakehp_flasher.html`

- A single file of about 3 MB that works offline. The DLL is never uploaded; everything runs in the browser.
- A JavaScript port of the extractor finds and validates the 48 kHz room filters, then builds the filter region ([section 3](#3-filter-data-handling)).
- It appends that region to the firmware image (UF2) the user picks:
  - Normal or Debug (both carried in the page), or the user's own build.
  - Main-family blocks are renumbered; picotool's absolute-family block is kept.
- It saves the result as `lakehp_pico2w_ready.uf2`.
- The page also carries the source zip and the controls list.
- In testing, the browser build was byte-identical to the Python build.

### `tools/`

| Tool | Purpose |
|---|---|
| `extract_filters.py` | Lists or extracts the filters. Output: a C header (`--rates`), raw `.bin` files (`--bins`), a region blob (`--region`), or a ready UF2 (`--uf2` / `--uf2-out`). |
| `make_flasher.py` | Builds the flasher page. It refuses any UF2 that already contains filters. |
| `lhp_wav.c` | Renders 16-bit 5.1/stereo WAVs on a PC. `--rear N` applies N × the 136° placement. |
| `vendor_tinyusb_s3.py` | Vendors Espressif's TinyUSB 0.18.0.6 for the ESP32-S3, with a FIFO-sizing patch. The result is not committed. |

---

## 14. Other builds

These use the same engine and pipeline but have not run on hardware yet.

### ESP32-S3 + classic ESP32 (ESP-IDF 5.5.x)

- The S3 is the USB 5.1 sound card and runs the DSP. It sends 48 kHz stereo over I2S to the classic ESP32, which is the A2DP source (Bluedroid).
- S3 controls: a BOOT press selects the next room; holding BOOT toggles low-latency stereo.

| ESP32-S3 | | ESP32 | Signal |
|---|---|---|---|
| GPIO5 | → | GPIO26 | I2S BCLK |
| GPIO6 | → | GPIO25 | I2S WS / LRCK |
| GPIO7 | → | GPIO22 | I2S DATA |
| GND | — | GND | Ground |
| 5V | → | 5V | Optional power |

### ESP32-S3 + nRF5340 DK, LE Audio (NCS 3.4.1)

- The S3 is the I2S slave with 16-bit slots; the nRF5340 is the I2S master.
- The nRF runs Nordic's `nrf_audio` unicast client (LC3) with `overlay-lakehp.conf`. The overlay sets the I2S source to 48 kHz and connects to the headset by name.

| nRF5340 DK | | ESP32-S3 | Signal |
|---|---|---|---|
| P1.07 | → | GPIO5 | I2S SCK (1.536 MHz) |
| P1.05 | → | GPIO6 | I2S LRCK |
| P1.09 | ← | GPIO7 | I2S DATA |
| GND | — | GND | Ground |

**Smaller module options:**
- The MK13 has castellated edges and is hand-solderable.
- The MDBT53-P1M has bottom pads only and needs reflow or a carrier board.
- Either needs an SWD programmer and a board overlay.
- A small RP2350 could replace the S3 as the front end, using a PIO I2S slave output.

Wiring diagrams: [wiring.html](wiring.html) and `wiring_*.svg`.

---

## 15. Memory and CPU budget

Figures are for the Pico 2 W.

### RAM (520 KB SRAM)

- About 465 KB is allocated statically, including 7.1 support. The engine state alone is about 284 KB.
- Heap left: about 29 KB in the release build and about 20 KB in the debug build.
- Changes that made room for 7.1:
  - engine input buffers stored as int16;
  - no block-copy buffer in the pipeline;
  - the debug log ring cut to 8 KB;
  - the USB FIFO kept at its old size in bytes.
- lwIP: `MEM_SIZE` 4000, a 4-buffer pbuf pool, TCP MSS 1460, 2 HTTP connections.

### CPU (1024-frame block = 21.3 ms at 48 kHz)

| Mode | DSP time |
|---|---|
| 5.1 | 13.2–13.7 ms measured (~62–64% of core 1) |
| 7.1 Factory | Same as 5.1 |
| 7.1 rear pair moved (Theater / Custom) | Estimated +25–30%, about 80% of core 1. The FFT shortcut offsets part of this. |

Measured on hardware:
- USB at 1000 packets/s and DSP at 46–47 blocks/s.
- SBC at 375 frames/s (48 kHz) or 344.5 frames/s (44.1 kHz).
- No dropped or padded frames.

### Latency estimates

These cover the device plus the link; the headset adds its own buffering.

| Build | Rooms | Low-latency |
|---|---|---|
| Pico 2 W, A2DP | ~70 ms | ~30 ms |
| ESP32-S3 + ESP32, A2DP | ~140 ms | ~70 ms |
| ESP32-S3 + nRF5340, LE Audio | ~35 ms + 40–60 ms link | ~15 ms + 40–60 ms link |

---

## 16. Verification status

### Verified on hardware

Tested on a Pico 2 W with Windows 11 and a 48 kHz SBC headset:
- Clean 5.1 playback: no hangs, packet loss or clicks.
- Room switching.
- The 136° surround placement, confirmed by ear.

### Verified on a PC

- **Engine vs DLL:** within ±1 LSB, and 5.1 output is still bit-identical after all changes.
- **Pipeline:**
  - matches the engine exactly;
  - rate locking holds;
  - survives 40 ms stall bursts;
  - low-latency output is an exact downmix.
- **Resampler:** 71–75 dB SNR.
- **7.1:**
  - A Factory back channel equals a 5.1 surround (zero difference).
  - A rear pair at 150° equals a 5.1 surround at 150° (zero difference).
  - Side + back together equals the sum of the parts (within 1 LSB).
  - The 7.1 pipeline output matches the engine directly.
  - The angle mapping round-trips within 1°.
- **USB descriptors:** decoded and checked for 5.1 and 7.1, with and without the network adapter. This covered interface numbering, channel masks and endpoint sizes.
- **Settings page:** exercised in a headless browser against a mock API.
- **Builds:** Pico release and debug builds compile and link (pico-sdk 2.3.1, clang).

### Not yet tested on hardware

- The settings page / RNDIS, including the interface-0 fix for Code 10.
- 7.1 mode, and the DSP load with Theater's rear pair.
- The presets and the Surround lab.
- Media keys (AVRCP).
- iPhone (Apple feedback format).
- Switch 2 stereo mode.
- The pairing changes.
- The ESP32 and nRF5340 builds.

---

## 17. File map

```
engine/lakehp.c, lakehp.h        engine, placement, 7.1 rear pair
firmware/common/
  dh_pipeline.c/.h               rings, rate lock, rooms, presets, beeps, 5.1/7.1
  usb_audio51.c/.h               UAC2 5.1/7.1, UAC1 stereo, HID media keys, composite builder
  usb_audio51_tusb_config.h      TinyUSB settings shared by the builds
  lhp_filter_region.h            filter region parser + CRC
  resampler.c/.h                 48 → 44.1 kHz resampler
firmware/pico2w/
  main.c                         cores, USB service, button, LED, settings, web hooks
  bt_a2dp.c/.h                   A2DP source, pairing, chime, AVRCP target
  webcfg.c/.h, lwipopts.h        RNDIS + lwIP + DHCP + HTTP
  web/index.html                 settings page
  dbg_cdc.c/.h                   debug USB serial log
  btstack_config.h, tusb_config.h, CMakeLists.txt
firmware/esp32s3_usb_dsp/        ESP32-S3 USB + DSP → I2S
firmware/esp32_a2dp_tx/          ESP32 I2S → A2DP
firmware/nrf5340_le_audio/       nrf_audio overlay + README
tools/                           extract_filters.py, make_flasher.py, flasher_template.html,
                                 lhp_wav.c, vendor_tinyusb_s3.py
test/                            pipeline, stall, lowlat, resampler, surround, surround71; host_run
docs/                            TECHNICAL.md, DECOMPILATION.md, wiring.html, wiring SVGs
```

### Building the Pico firmware

```sh
cmake -S firmware/pico2w -B build-pico -DPICO_BOARD=pico2_w [-DLHP_DEBUG_CDC=1]
cmake --build build-pico -j
```

This needs pico-sdk 2.3.x with the `tinyusb`, `btstack`, `cyw43-driver` and `lwip` submodules.
