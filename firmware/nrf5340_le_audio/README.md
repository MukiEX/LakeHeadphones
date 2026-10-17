# LE Audio build: ESP32-S3 + nRF5340 DK

```
PC ──USB 5.1──▶ ESP32-S3 (USB + Dolby Headphone DSP) ──I2S──▶ nRF5340 DK ──LE Audio / LC3──▶ headset
```

This build is for headsets with **LE Audio**. Audio travels on isochronous channels with a negotiated presentation delay, instead of A2DP's large jitter buffer. My estimate is about **60–100 ms end to end**, compared with 200–400 ms over A2DP. These are not measurements.

The nRF5340 runs Nordic's own `nrf_audio` **unicast client** from nRF Connect SDK v3.4.x, unmodified. Only the settings in `overlay-lakehp.conf` are added. The ESP32-S3 runs the same firmware as the A2DP build, switched to I2S slave with 16-bit slots.

## 1. Check your headset first

LE Audio support is still uneven. Before building anything, confirm the headset streams LE Audio from an LE Audio phone (Android 13+ with LE Audio turned on, e.g. a recent Pixel or Galaxy). For example, Sony's WF-1000XM5 earbuds have LE Audio in beta firmware, but I couldn't confirm it shipped for the WH-1000XM5 over-ears. If the phone can't use LE Audio with the headset, this build won't either.

## 2. ESP32-S3

```sh
python3 tools/vendor_tinyusb_s3.py
cd firmware/esp32s3_usb_dsp
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.nrf5340" set-target esp32s3
idf.py build flash monitor
```

`sdkconfig.defaults.nrf5340` sets I2S slave, 16-bit slots and a 32 ms rooms-mode buffer. The 32 ms is possible because the nRF's clock is steady.

## 3. nRF5340 DK

You need nRF Connect SDK **v3.4.x** (`west init -m https://github.com/nrfconnect/sdk-nrf --mr v3.4.1`, then `west update`). The application is `nrf/applications/nrf_audio`. Older SDKs call it `nrf5340_audio` and use different board names.

1. Edit `CONFIG_BT_DEVICE_NAME` in `overlay-lakehp.conf` to your headset's exact name.
2. Build and flash. Use your own path to this folder:

```sh
cd <ncs>/nrf/applications/nrf_audio
west build -b nrf5340dk/nrf5340/cpuapp --sysbuild -d build_lakehp --pristine -- \
  -DEXTRA_CONF_FILE="unicast_client/overlay-unicast_client.conf;/path/to/lakehp/firmware/nrf5340_le_audio/overlay-lakehp.conf" \
  -DFILE_SUFFIX=release
west flash -d build_lakehp
```

Drop `-DFILE_SUFFIX=release` to get Nordic's debug build with logs on the DK's USB serial port.

## 4. Wiring

Drawing: [../../docs/wiring_esp32s3_nrf5340.svg](../../docs/wiring_esp32s3_nrf5340.svg) (also in `docs/wiring.html`).

| ESP32-S3 DevKitC-1 | | nRF5340 DK | Signal |
|---|---|---|---|
| GPIO5 | ← | P1.07 | I2S SCK (1.536 MHz, driven by the nRF) |
| GPIO6 | ← | P1.05 | I2S LRCK (48 kHz, driven by the nRF) |
| GPIO7 | → | P1.09 | I2S SDIN (processed stereo) |
| GND | — | GND | ground (required) |

- Power each board from its own USB.
- The nRF5340's GPIOs run at the DK's VDD. Set the DK to **3 V** with its VDD switch. At 1.8 V you need a level shifter between the boards.
- P1.08 (MCK) stays unconnected.

## 5. Using it

- **S3 BOOT button:**
  - Short press: next room.
  - Hold 2 s: **low-latency stereo**. This skips the Dolby engine's 1024-sample blocks and saves about 40 ms more, at the cost of the virtual rooms.
- **DK buttons** follow Nordic's nrf_audio UI: Button 1/2 volume down/up, Button 3 play/pause, Button 4 mute.
- After the first connection the headset is bonded and reconnects automatically. To pair a different headset, **hold Button 4 while the DK starts** to clear bonds (or erase with `west flash --erase`), then change `CONFIG_BT_DEVICE_NAME` if it's a different model.

## Known limits

- The `nrf_audio` application is Nordic's reference design. Interoperability with commercial headsets varies by headset firmware. Nordic recommends testing against a second DK running `unicast_server` first.
- Earbud pairs are two devices in one coordinated set. The unicast client handles this through CSIP, but only once both buds advertise.
- The Dolby Headphone engine still adds its 21 ms block in rooms mode. Low-latency stereo mode removes it.
