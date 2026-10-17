#!/usr/bin/env python3
"""
make_flasher.py - build the all-in-one lakehp_flasher.html.

  python3 tools/make_flasher.py --normal build/lakehp_pico2w.uf2 \
      --debug build-debug/lakehp_pico2w.uf2 --src lakehp-5.1-virtualizer.zip \
      -o lakehp_flasher.html

Both UF2s must be built WITHOUT firmware/generated/lakehp_filters.h, so they
read the room filters from flash; the page adds those from the user's DLL.
Refuses to embed a UF2 that already contains filter data.
"""
import argparse, base64, os, struct, sys

here = os.path.dirname(os.path.abspath(__file__))


def check_uf2(path):
    d = open(path, "rb").read()
    if len(d) % 512 or struct.unpack_from("<I", d, 0)[0] != 0x0A324655:
        sys.exit(f"{path}: not a UF2")
    for i in range(0, len(d), 512):
        addr = struct.unpack_from("<I", d, i + 12)[0]
        if 0x10300000 <= addr < 0x10380000:
            sys.exit(f"{path}: already has data in the filter area - build it without lakehp_filters.h")
    img = b"".join(d[i + 32:i + 288] for i in range(0, len(d), 512))
    if b"LHPF" in img and b"DH1\0" in img:
        sys.exit(f"{path}: looks like it has filters compiled in")
    return d


ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--normal", required=True)
ap.add_argument("--debug", required=True)
ap.add_argument("--src", required=True)
ap.add_argument("--version", default="2026.10")
ap.add_argument("-o", "--out", default="lakehp_flasher.html")
a = ap.parse_args()

t = open(os.path.join(here, "flasher_template.html"), encoding="utf-8").read()
t = t.replace("__FW_VERSION__", a.version)
t = t.replace("__FW_NORMAL__", base64.b64encode(check_uf2(a.normal)).decode())
t = t.replace("__FW_DEBUG__", base64.b64encode(check_uf2(a.debug)).decode())
t = t.replace("__SRC_ZIP__", base64.b64encode(open(a.src, "rb").read()).decode())
open(a.out, "w", encoding="utf-8").write(t)
print(f"wrote {a.out} ({os.path.getsize(a.out) // 1024} KB)")
