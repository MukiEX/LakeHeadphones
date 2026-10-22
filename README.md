# LakeHP: a 5.1/7.1 headphone virtualizer on a microcontroller

This turns a small board into a **USB 5.1 or 7.1 sound card**. It renders the channels to binaural stereo with a portable C reimplementation of the Lake Technology headphone engine (`DolbyHph.dll` 1.20.0276; its three rooms), then sends the result to **Bluetooth headphones** over A2DP. A button cycles **Small → Medium → Large room**.

```
PC / console ──USB 5.1/7.1 (UAC2, 48 kHz)──▶ [ DSP: virtual rooms ] ──A2DP/SBC──▶ BT headset
```

* `engine/` is the decompiled engine as plain C99 (`lakehp.c/.h`). It matches the original `DolbyHph.dll` to ±1 LSB (about 102 dB SNR). See [docs/DECOMPILATION.md](docs/DECOMPILATION.md).
* `firmware/pico2w/` is the **single-board** build for the **Raspberry Pi Pico 2 W** (RP2350 + CYW43439, which has Classic Bluetooth).
* `firmware/rp2350_usbhost/` is the **USB output** build for the **Waveshare RP2350-USB-A**: the sound card on its USB-C port, and the rendered stereo goes out of its USB-A port to a **USB DAC dongle** or a **2.4 GHz wireless headset's USB dongle**. No Bluetooth, so much lower latency.
* `firmware/esp32s3_usb_dsp/` + `firmware/esp32_a2dp_tx/` is the **two-board** build: an ESP32-S3 handles USB and DSP and sends I2S to a classic ESP32 that does the Bluetooth.
* `firmware/esp32s3_usb_dsp/` + `firmware/nrf5340_le_audio/` is the **LE Audio** build: the same S3 feeds an nRF5340 DK running Nordic's `nrf_audio` unicast client (LC3), for LE Audio headsets such as the Sony XM5/XM6.

Wiring diagrams for all three are in [docs/wiring.html](docs/wiring.html), with standalone SVGs alongside it in `docs/`.

The full technical breakdown (engine, placement, 7.1, USB, Bluetooth, settings page, budgets, test status) is in [docs/TECHNICAL.md](docs/TECHNICAL.md).

> **Why not an ESP32-S3 alone?** The S3 has only Bluetooth LE, and ordinary headsets need Bluetooth Classic (A2DP). Only the original ESP32 has Classic, and it has no USB device port. The nRF52840 is BLE-only too. The Pico 2 W is the one cheap board with both native USB and Classic BT.

## Quick start without compiling

Open **`lakehp_flasher.html`** in any desktop browser, drop in your copy of the DLL, and click **Build**. The page pulls the three 48 kHz room filters out of the DLL, adds them to the prebuilt firmware it carries (Pico 2 W or RP2350-USB-A), and saves a ready UF2. Hold BOOTSEL, plug in the board, and drag that file onto the RP2350 drive. It all runs locally: nothing is uploaded. The page also carries this source tree as a download.

The filters go into their own flash area (0x10300000 on the Pico 2 W, 0x10100000 on the RP2350-USB-A; each firmware carries its address and the tools read it), separate from the firmware, so later firmware-only UF2s keep them. Without filters the firmware still runs, as plain low-latency stereo, and the LED blinks 8 times.

To regenerate the page after changing the firmware: `python3 tools/make_flasher.py --normal <uf2> --debug <debug uf2> [--usba-normal <uf2> --usba-debug <uf2>] --src <zip>`. The UF2s must be built without `lakehp_filters.h`.

## Open room filters (no original DLL needed)

[`filters/LakeHP_OpenRooms.dll`](filters/) is a drop-in stand-in for the original DLL, built from the open SONICOM HRTF dataset (CC BY-SA 3.0). It has three rooms (Open Small / Medium / Large) at 48 and 44.1 kHz, loudness-matched to the original's rooms, in the same filter format. Use it anywhere the original goes: the flasher page, `extract_filters.py`, the Jellyfin filter, neko. To build it again from another head or with other rooms, use `tools/make_open_filters.py`; details are in [filters/README.md](filters/README.md).

## 0. Filter data (for building from source)

The original filters are not included in this repo. Extract them from **your own** copy of the DLL, or use the open DLL above:

```sh
python3 tools/extract_filters.py /path/to/DolbyHph.dll --rates 48000 -o firmware/generated/lakehp_filters.h
```

This writes about 97 KB (three rooms at 48 kHz) into a git-ignored header. The ESP32-S3 build needs it. The Pico build compiles it in when present; otherwise it reads the filters from flash, which you add with the flasher page or `extract_filters.py DolbyHph.dll --uf2 build-pico/lakehp_pico2w.uf2 --uf2-out ready.uf2`. Keep it out of anything you publish. `--list` shows everything in the DLL; `--bins dir/ --rates all` dumps raw blobs for the PC preview tool.

## 1. Pico 2 W (single board)

Tested build: pico-sdk 2.3.1, board `pico2_w`.

```sh
export PICO_SDK_PATH=/path/to/pico-sdk          # with submodules: tinyusb, btstack, cyw43-driver
cmake -S firmware/pico2w -B build-pico -DPICO_BOARD=pico2_w
cmake --build build-pico -j
# hold BOOTSEL, plug in, copy build-pico/lakehp_pico2w.uf2 to the RP2350 drive
```

Options: `-DLHP_DEFAULT_ROOM=1|2|3`, and `-DLHP_BUTTON_GPIO=15` to use a button from GP15 to GND instead of BOOTSEL. Logs go to UART0 (GP0 TX, 115200 baud).

**Debug build:** add `-DLHP_DEBUG_CDC=1`. A USB serial port appears next to the sound card (Windows calls it "USB Serial Device"). Open it in any terminal to get every USB audio request plus a status line every 2 s: USB packet rate, DSP and encoder load, and Bluetooth stats.

How it runs (tested on hardware with Windows 11 and an A2DP headset):
- The CPU runs at 200 MHz with the core at 1.15 V. The CYW43 SPI divider is scaled to keep its bus speed unchanged.
- **Core 0:** TinyUSB runs from a 250 µs timer interrupt. The main loop SBC-encodes the Bluetooth audio (~0.45 ms per frame) and handles the button. The RP2 USB driver re-arms the isochronous endpoint only from `tud_task()`, so anything that delays it over ~1 ms loses a USB packet, which you hear as fuzz.
- **Core 1:** the virtualizer DSP (~13.7 ms per 21 ms block in 5.1, about 64% of the core; roughly 25% more in 7.1 with the rear pair moved) and BTstack + CYW43.
- `usb_audio51.c` includes a fix for TinyUSB 0.18 on RP2: the stream's endpoints are disarmed when the host closes it. Without that, Windows' open/close/reopen at the start of playback panicked the USB driver and froze the device.
- RAM: about 465 KB static (7.1-capable), leaving ~30 KB heap (~20 KB in the debug build).

## 1b. Waveshare RP2350-USB-A (USB DAC / 2.4 GHz headset dongle)

```
PC / console ──USB-C: 5.1/7.1 sound card──▶ [ RP2350: virtual rooms ] ──USB-A: USB audio──▶ DAC or headset dongle
```

The USB-A port is a USB host made from PIO state machines ([Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB)), driving any **USB Audio Class 1 or 2 playback device at full speed**: phone-style USB-C DACs (through a C-to-A adapter) and the USB dongles of wireless gaming headsets. LakeHP picks 48 kHz stereo 16-bit when offered, then 24/32-bit or multichannel (it fills the first two channels), and 44.1 kHz through its resampler as a last resort. It unmutes the device and sets its volume to 0 dB; the PC's volume control works as usual.

```sh
python3 tools/vendor_usbhost_libs.py       # once: pinned TinyUSB (audio host class) + Pico-PIO-USB with isochronous support
export PICO_SDK_PATH=/path/to/pico-sdk      # 2.3.x with lib/lwip
cmake -S firmware/rp2350_usbhost -B build-usbhost [-DLHP_DEBUG_CDC=1]
cmake --build build-usbhost -j
python3 tools/extract_filters.py <DLL> --uf2 build-usbhost/lakehp_usbhost.uf2   # adds the rooms at 0x10100000
```

Or use the flasher page and pick **RP2350-USB-A**. Flash it through the **USB-C** port (hold BOOT while plugging in).

- **Controls** are the Pico 2 W's (BOOT button), except **hold 2 s**, which restarts the output stream.
- **LED** (RGB): green = playing (cyan in stereo mode), yellow = starting, red blip each second = nothing plugged in, magenta = the device has no playback format LakeHP can use, fast red = retrying.
- **Settings page:** same page; its *Output* tab shows the device, its format, timing diagnostics, a test tone and a restart button. Both boards show which DLL the room filters came from.
- **Latency:** about 50 ms with the rooms (the 1024-sample convolution blocks; no Bluetooth buffering on top) and about 6 ms in low-latency stereo, plus whatever the DAC or headset adds.
- **Limits:** full-speed devices only (every USB DAC and dongle is full speed; a hub in between works). One output device at a time. The dongle's microphone and buttons are not passed through yet.

## 2. ESP32-S3 + ESP32 (two boards)

ESP-IDF **v5.5.x**.

**ESP32-S3 (USB + DSP).** The S3's USB block only has a 1 KB FIFO, and stock TinyUSB can't fit a 6-channel endpoint into it. A script vendors Espressif's TinyUSB 0.18.0.6 with a one-line FIFO-sizing patch:

```sh
python3 tools/vendor_tinyusb_s3.py
cd firmware/esp32s3_usb_dsp
idf.py set-target esp32s3 && idf.py build flash monitor     # flash via the "UART" port
```

The S3 DevKit's **"USB" port is the sound card**. Flash and read logs through its **"UART"** port.

**Classic ESP32 (Bluetooth transmitter).**

```sh
cd firmware/esp32_a2dp_tx
idf.py set-target esp32 && idf.py menuconfig   # optional: headset name filter, pins
idf.py build flash monitor
```

**Wiring** (drawing: [docs/wiring_esp32s3_esp32.svg](docs/wiring_esp32s3_esp32.svg)):

| ESP32-S3 DevKitC-1 | | ESP32 DevKitC (WROOM-32) | Signal |
|---|---|---|---|
| GPIO5 | → | GPIO26 | I2S BCLK (3.072 MHz) |
| GPIO6 | → | GPIO25 | I2S WS / LRCK (48 kHz) |
| GPIO7 | → | GPIO22 | I2S DATA |
| GND | — | GND | ground (required) |
| 5V | → | 5V / VIN | optional: power the ESP32 from the S3's USB |

Pins can be changed in `menuconfig` on both sides. Keep the wires short (under 15 cm).

## 3. ESP32-S3 + nRF5340 DK (LE Audio)

This needs the nRF Connect SDK **v3.4.1** for the nRF5340 DK, and the S3 built as an I2S **slave** with 16-bit slots:

```sh
cd firmware/esp32s3_usb_dsp
idf.py set-target esp32s3
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.nrf5340" build flash
```

The nRF side is Nordic's `applications/nrf_audio` with the overlay `firmware/nrf5340_le_audio/overlay-lakehp.conf`. It switches the audio source to I2S at 48 kHz and connects to the headset by name. Build commands, the headset checklist and known limits are in [firmware/nrf5340_le_audio/README.md](firmware/nrf5340_le_audio/README.md).

| nRF5340 DK | | ESP32-S3 | Signal |
|---|---|---|---|
| P1.07 | → | GPIO5 | I2S SCK (1.536 MHz, nRF is master) |
| P1.05 | → | GPIO6 | I2S LRCK (48 kHz) |
| P1.09 | ← | GPIO7 | I2S DATA (processed stereo) |
| GND | — | GND | ground (required) |

Drawing: [docs/wiring_esp32s3_nrf5340.svg](docs/wiring_esp32s3_nrf5340.svg). Leave the DK's VDD switch at 3 V and power each board from its own USB.

## 4. Using it

1. Put your headset in **pairing mode** and power the device.
   - **Pairing (Pico):** with no remembered headset (first start, or after holding the button 2 s), the Pico listens in 5 s windows. At the end of each window it picks one audio device, preferring headphones and headsets over speakers, TVs and car kits, and the strongest signal among equals, so keep the headset close. It doesn't just grab the first one that answers. When it connects you hear two rising tones in the headset.
   - **Pico LED:** fast flicker = pairing mode · a short blip every second = reconnecting to the remembered headset · slow blink = connected, idle · solid = streaming.
   - The headset is remembered and reconnected automatically. To switch to a different one, hold the button for 2 s.
2. On the PC, pick **"LakeHP 5.1"** as the output.
   - **Windows:** open Sound → the device → **Configure → 5.1 Surround** (or **7.1 Surround** in 7.1 mode), and turn off Spatial Sound and Enhancements for the device.
   - **Linux:** select the "5.1 Surround" (or 7.1) profile.
3. Set games and players to 5.1 (or 7.1) output. Stereo content also works: it is placed on virtual front speakers.

| Control | Pico 2 W (BOOTSEL) | ESP32-S3 (BOOT) | ESP32 TX (BOOT) | nRF5340 DK |
|---|---|---|---|---|
| Single press | next room (1/2/3 beeps + LED blinks) | next room (beeps, saved in NVS) | — | — |
| Double press | low-latency stereo on/off | — | — | — |
| Triple press | USB mode: 5.1/7.1 ↔ stereo (restarts, saved) | — | — | — |
| 4 presses | placement preset: Factory ↔ Theater (saved) | — | — | — |
| 5 presses | settings page adapter on/off (restarts, saved) | — | — | — |
| Hold 2 s | forget headset, re-pair | low-latency stereo on/off | forget headset, re-pair | — |
| Buttons 1–4 | | | | vol −/+, play/pause, mute |

Volume and mute from the PC are applied before the Bluetooth link.

**Headset buttons → media keys (Pico).** The Pico is an AVRCP target, so the headset's play/pause, next and previous buttons (and stop, fast-forward/rewind) arrive as Bluetooth remote-control commands. They're passed to the PC as USB media keys through a small "LakeHP media keys" HID interface, so they control whatever is playing, like a keyboard's media keys would. Play and pause both send Play/Pause. Headsets that send volume up/down as commands change the PC volume; most keep volume in the headset itself, which is left alone (the Pico doesn't ask for absolute volume). Not in stereo (console) mode.

**Stereo USB mode (Pico).** Some hosts only support the older USB Audio Class 1.0 (UAC1) standard and won't see the 5.1 device at all, including the Nintendo Switch and many other consoles. Triple-press BOOTSEL and the Pico saves the choice, blinks 6 times and restarts as **"LakeHP Stereo"**, a 48 kHz stereo UAC1 speaker. The stereo signal still goes through the virtual room as front speakers. Triple-press again to get the 5.1/7.1 device back. The mode is remembered across power cycles.

**Settings page (Pico).** Open **http://192.168.20.1**. The Pico adds a small USB network adapter (RNDIS: built into Windows, also works on Linux; not on macOS) next to the sound card and hands your PC an address. The page sets the speaker arrangement (stereo / 5.1 / 7.1), the room and low-latency mode, and has a **Surround lab** with placement presets and an angle slider per speaker pair (plus fine-tune sliders) that you move while you listen, plus headset pairing and status. *Save* stores it all as the power-up default. Five presses on BOOTSEL turn the adapter off (or back on). In stereo mode the adapter is left out, for consoles. Building it needs the SDK's `lib/lwip` submodule (`git submodule update --init lib/lwip`); without it the build leaves the page out.

**7.1 (Pico).** Pick 7.1 on the settings page: the Pico restarts as an eight-channel sound card (FL FR FC LFE BL BR SL SR, 48 kHz; full-speed USB has room for it). The side channels play on the surround speakers, and the back channels get a **rear pair** of their own, made from the same surround filters with its own placement. With the rear pair at the surround position (the Factory preset) the backs are simply mixed into the surrounds and cost nothing extra; moved elsewhere they need two more transforms per block, about 25% more DSP. Set Windows' speaker setup to 7.1.

**Speaker placement (Pico).** The factory surround filters sit at the ITU surround angle: they measure as ITD 0.50 ms and ILD about 15 dB, so about 118° from straight ahead. On headphones that often sounds like *beside* you. The engine can move a speaker pair anywhere from 90° to 165°: a shorter time and level difference between the ears moves it behind you (with the "behind" spectral cues: more 1 kHz and 11 kHz, less 4.5 kHz), a longer one moves it forward. It is applied to the filters at load time, and the room reverb is unchanged. Two presets:

| Preset | Surrounds (sides in 7.1) | 7.1 rear pair |
|---|---|---|
| **Factory** | ~118° (the filters as made) | same as the surrounds |
| **Theater** | 136° | 150° |

Pressing 4 times toggles Factory ↔ Theater (one low beep = Factory, two = Theater); the Surround lab sets anything in between. The setting is saved; saving causes a short (~50 ms) gap. On a PC: `lhp_wav --rear 1 filter.bin in.wav out.wav` (136°).

**Low-latency stereo.** This mode skips the room engine and sends a plain stereo downmix: L = FL + 0.707·(C + LFE) + 0.707·SL, and the mirror for R. It works in 256-frame chunks with a much smaller buffer target, so it removes the engine's 1024-frame block and most of the buffering. Two high beeps mean it's on. Turning it off brings back the room you had, with that room's beeps. In the host simulation, device buffering drops from about 57 ms (rooms) to about 18 ms (LL).

**Latency** (estimates: device + link, and the headset adds its own buffer on top):

| Build | Rooms | Low-latency stereo |
|---|---|---|
| Pico 2 W, A2DP | ~70 ms + headset (100–200 ms) | ~30 ms + headset |
| ESP32-S3 + ESP32, A2DP | ~140 ms + headset | ~70 ms + headset (with `LHP_TX_TARGET_FILL` ≈ 1200) |
| ESP32-S3 + nRF5340, LE Audio | ~35 ms + ~40–60 ms link | ~15 ms + ~40–60 ms link |

Buffer knobs: `LHP_TARGET_FILL` (S3 menuconfig), `DHP_TARGET_FILL` (`firmware/common/dh_pipeline.h`, Pico) and `LHP_TX_TARGET_FILL` (ESP32 TX menuconfig). Lower values cut latency but make dropouts more likely during Bluetooth stalls.

## 5. Tests and PC preview

```sh
cc -O2 -Iengine -o lhp_wav tools/lhp_wav.c engine/lakehp.c -lm
python3 tools/extract_filters.py DolbyHph.dll --bins bins --rates all
./lhp_wav bins/lhp_filter_dh2_medium_48000.bin movie_51.wav binaural.wav   # 16-bit 5.1/stereo WAV

cc -O2 -Iengine -o pipeline_test test/pipeline_test.c firmware/common/dh_pipeline.c engine/lakehp.c -lm && ./pipeline_test bins
cc -O2 -Iengine -o stall_test test/stall_test.c firmware/common/dh_pipeline.c engine/lakehp.c -lm && ./stall_test bins
cc -O2 -Iengine -o lowlat_test test/lowlat_test.c firmware/common/dh_pipeline.c engine/lakehp.c -lm && ./lowlat_test bins
cc -O2 -o resampler_test test/resampler_test.c firmware/common/resampler.c -lm && ./resampler_test
```

## What was verified, and what wasn't

**Verified in this build:**
- The engine is bit-accurate to the original DLL (run through an i386 PE loader) across rooms, rates and channel layouts.
- The Pico 2 W firmware **compiles and links** against pico-sdk 2.3.1, TinyUSB, BTstack and the CYW43 driver (clang build).
- The RP2350-USB-A firmware **compiles and links** (pico-sdk 2.3.1, TinyUSB 0.21 at the pinned commit, patched Pico-PIO-USB). Its isochronous packet encoder passes a host test for 2–404-byte packets, and the flasher page and `extract_filters.py` produce identical UF2s for it.
- Both ESP-IDF projects pass a full compiler front-end check against the ESP-IDF v5.5.5 and TinyUSB headers.
- The USB descriptors were decoded and checked.
- Host simulations passed: pipeline rate locking, 40 ms Bluetooth stalls, low-latency mode switching (the LL output is an exact downmix and no USB data is lost), and resampler quality (71–75 dB SNR).
- The nRF5340 overlay uses only Kconfig symbols that exist in NCS v3.4.1, and the S3's `sdkconfig.defaults.nrf5340` variant passes the same front-end check. The nRF build itself was not compiled here.

**Hardware status:** the Pico 2 W build plays clean audio on real hardware (Windows 11, SBC 48 kHz headset). The RP2350-USB-A build plays clean audio to a Turtle Beach Stealth 400 dongle (synchronous USB Audio 1.0) on real hardware. The ESP32 and nRF5340 builds have not been run on hardware yet. Expect some first-flash tuning: headset quirks, USB enumeration on your OS, buffer sizes. macOS needs `-DUSB_AUDIO51_MACOS=1` for its full-speed feedback format.

## Layout

```
engine/            lakehp.c/.h        portable headphone engine (no filter data)
tools/             extract_filters.py make_open_filters.py vendor_tinyusb_s3.py vendor_usbhost_libs.py lhp_wav.c
                   flasher_template.html + make_flasher.py -> lakehp_flasher.html
firmware/common/   dh_pipeline (rings, rate lock, rooms, beeps), usb_audio51 (UAC2 5.1), resampler
firmware/pico2w/   single-board build (TinyUSB + BTstack A2DP source)
firmware/rp2350_usbhost/   Waveshare RP2350-USB-A: USB sound card -> USB DAC / headset dongle (PIO-USB host)
firmware/esp32s3_usb_dsp/  ESP32-S3: USB + DSP -> I2S
firmware/esp32_a2dp_tx/    ESP32:  I2S -> A2DP (Bluedroid)
firmware/nrf5340_le_audio/ nRF5340 DK: overlay for Nordic nrf_audio (I2S -> LE Audio unicast)
filters/           LakeHP_OpenRooms.dll (open room filters, CC BY-SA 3.0) + README
docs/TECHNICAL.md          full technical breakdown
docs/DECOMPILATION.md      engine internals and the filter blob format
docs/wiring.html, docs/wiring_*.svg   wiring diagrams for all three builds
test/              host tests
```

*Dolby and Dolby Headphone are trademarks of Dolby Laboratories. This is an independent interoperability and reverse-engineering project for personal use. It ships no Dolby code or data, and the filters come from a DLL you already have.*
