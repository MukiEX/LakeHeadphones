# Open room filters

`LakeHP_OpenRooms.dll` is a stand-in for the original DLL, built entirely from open data. It holds three rooms (**Open Small**, **Open Medium**, **Open Large**) at 48 kHz and 44.1 kHz. They are written in the same filter-blob format and packed in a valid, code-free PE32 DLL.

Anything that reads the original DLL reads this one too:

- the flasher page (`lakehp_flasher.html`);
- `tools/extract_filters.py` (`--uf2`, `--region`, `-o header`, `--bins`);
- the Jellyfin `lakehp` ffmpeg filter (drop it in `/config/lakehp/`);
- neko's `nekosurround` element (`dh-dll=`).

The room numbers are the same as the original's (1/2/3 = small/medium/large), so room switching, placement presets, the Surround lab and 7.1 all work unchanged.

## How it was made

`tools/make_open_filters.py`, from subject **p0010** of the SONICOM HRTF dataset. p0010 is the subject closest to the average of the 30 subjects shipped with the DAFx 2026 HRTF tutorial (within 1.3 dB).

- **Speakers:** measured head-related impulse responses at L/R ±30°, C 0° and surrounds ±120°.
- **Headphone EQ:** a diffuse-field equalizer built from all 828 measured directions.
  - It is limited to ±15 dB and rolled off below 40 Hz.
- **Early reflections:** each speaker gets an image-source room model (up to third order).
  - Every reflection uses the measured response for its own direction.
  - The reflections are placed in that speaker's 1024-tap direct filter.
- **Late tail:** shared by all speakers.
  - It is binaural diffuse reverb from 96 directions over the sphere.
  - It has per-band decay times and fades out by 149 ms.

### Calibration against the original

Only measurements of the original were used: decay, early/late energy and K-weighted loudness. None of its filter data is used.

| Room | Early/late (front L) | −30 dB decay | Loudness |
|---|---|---|---|
| Small | +13.0 dB (original +13.5) | 78 ms (73) | matched ±0.01 dB |
| Medium | +15.0 dB (15.2) | 94 ms (97) | matched |
| Large | +10.0 dB (9.8) | 91 ms (91) | matched |

The direct sound's cues match the original's closely:

| Speaker | Open ITD / ILD | Original ITD / ILD |
|---|---|---|
| Front | 0.29 ms / 9.3 dB | 0.29 ms / 10.0 dB |
| Surround | 0.60 ms / 14.0 dB | 0.60 ms / 14.9 dB |

Tonal balance is within about ±2 dB of the original from 125 Hz to 16 kHz. Open Large has about +4 dB more room boom at 63 Hz.

## Rebuilding (other heads, other rooms)

```sh
pip install numpy scipy h5py
python3 tools/make_open_filters.py /path/to/pXXXX.sofa -o LakeHP_OpenRooms.dll [--bins outdir]
```

Any 48 kHz SOFA `SimpleFreeFieldHRIR` file works.

- **Different head:** pick another SONICOM subject, or your own measurement.
- **Different rooms:** edit the `ROOMS` table at the top of the script: size, speaker distance, wall reflection, reverb times, direct/tail ratio.
- **Loudness:** the calibration gains (`ROOM_GAIN`, `RATE441_TRIM`) were measured for p0010. Other heads land within about 1 dB.

## Licence

The HRTF data comes from the **SONICOM HRTF dataset** (Imperial College London, Audio Experience Design group). It is licensed **CC BY-SA 3.0** (Creative Commons Attribution-ShareAlike 3.0 Unported, https://creativecommons.org/licenses/by-sa/3.0/). It was taken from the copy redistributed with the DAFx 2026 HRTF tutorial (github.com/yzyouzhang/dafx2026-hrtf-tutorial).

`LakeHP_OpenRooms.dll` and the filters inside it are derived from that data, so they are distributed under the same **CC BY-SA 3.0** licence, with attribution to SONICOM / Imperial College London. The DLL carries this notice in its version resource and at the start of its data section.

The generator script is part of LakeHP and is under the project's licence.
