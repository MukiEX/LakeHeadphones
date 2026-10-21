#!/usr/bin/env python3
"""
vendor_usbhost_libs.py - fetch the two libraries the RP2350-USB-A build needs
into firmware/rp2350_usbhost/lib (not kept in the repo):

  lib/tinyusb       TinyUSB at a pinned commit: the first releases with the
                    USB audio *host* class (pico-sdk 2.3 ships 0.18, without it).
  lib/Pico-PIO-USB  the PIO USB host at a pinned commit, plus
                    patches/pico-pio-usb-iso.patch: isochronous transfers
                    (upstream has control, bulk and interrupt only).

Needs git and network access to github.com. Safe to run again.
"""
import os, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "firmware", "rp2350_usbhost")
LIB = os.path.join(FW, "lib")
PATCH = os.path.join(FW, "patches", "pico-pio-usb-iso.patch")
LIBS = [
    ("tinyusb", "https://github.com/hathach/tinyusb.git", "782f0f1cd71b43d4339549135bbe8e313e88d344", None),
    ("Pico-PIO-USB", "https://github.com/sekigon-gonnoc/Pico-PIO-USB.git", "5a37a66dc5d3fbe0ef3cdbeda923a757440f984f", PATCH),
]


def git(cwd, *args, check=True):
    return subprocess.run(["git", *args], cwd=cwd, check=check, capture_output=True, text=True)


def fetch(name, url, rev):
    dest = os.path.join(LIB, name)
    if not os.path.isdir(os.path.join(dest, ".git")):
        os.makedirs(dest, exist_ok=True)
        git(dest, "init", "-q")
        git(dest, "remote", "add", "origin", url)
    head = git(dest, "rev-parse", "HEAD", check=False).stdout.strip()
    if head != rev:
        print(f"fetching {name} @ {rev[:10]} ...")
        git(dest, "fetch", "-q", "--depth", "1", "origin", rev)
        git(dest, "checkout", "-q", "--force", "FETCH_HEAD")
    return dest


def main():
    for name, url, rev, patch in LIBS:
        dest = fetch(name, url, rev)
        if not patch:
            print(f"{name}: ok")
            continue
        if git(dest, "apply", "--reverse", "--check", patch, check=False).returncode == 0:
            print(f"{name}: ok (iso patch already applied)")
            continue
        r = git(dest, "apply", patch, check=False)
        if r.returncode:
            sys.exit(f"{name}: the patch does not apply:\n{r.stderr}\n"
                     f"(local changes? reset with: git -C {dest} checkout -- .)")
        print(f"{name}: ok (iso patch applied)")


if __name__ == "__main__":
    main()
