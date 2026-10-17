#!/usr/bin/env python3
"""
vendor_tinyusb_s3.py - put a patched copy of Espressif's TinyUSB (v0.18.0.6)
into firmware/esp32s3_usb_dsp/components/tinyusb.

Why: the ESP32-S3 USB-OTG block has only 1 KB of FIFO. Stock TinyUSB sizes the
RX FIFO for two max-size OUT packets, so a 6-channel 48 kHz 16-bit isochronous
endpoint (588 bytes) can't be opened. One packet of RX buffering (the DWC2
databook minimum) is plenty for one isochronous packet per 1 ms frame, and fits.
"""
import os, re, subprocess, sys

TAG = "v0.18.0.6"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEST = os.path.join(ROOT, "firmware", "esp32s3_usb_dsp", "components", "tinyusb")
MARK = "LakeHP: single-buffered RX FIFO"

def main():
    if not os.path.isdir(DEST):
        os.makedirs(os.path.dirname(DEST), exist_ok=True)
        subprocess.check_call(["git", "clone", "--depth", "1", "--branch", TAG,
                               "https://github.com/espressif/tinyusb.git", DEST])
    f = os.path.join(DEST, "src", "portable", "synopsys", "dwc2", "dcd_dwc2.c")
    src = open(f, encoding="utf-8").read()
    if MARK in src:
        print("already patched:", f); return
    pat = re.compile(r"return 13 \+ 1 \+ 2 \* \(\(largest_ep_size / 4\) \+ 1\) \+ 2 \* ep_count;")
    if not pat.search(src):
        sys.exit("patch target not found - TinyUSB version changed?")
    src = pat.sub("return 13 + 1 + ((largest_ep_size / 4) + 1) + 2 * ep_count; /* %s */" % MARK, src)
    open(f, "w", encoding="utf-8").write(src)
    print("patched", f)

if __name__ == "__main__":
    main()
