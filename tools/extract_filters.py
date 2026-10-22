#!/usr/bin/env python3
"""
extract_filters.py - pull the Dolby Headphone room filters (DH1/DH2/DH3) out of
YOUR OWN copy of DolbyHph.dll (Lake Technology, 1.20.0276) and emit them as a
C header (and optional .bin files) for the lakehp engine.

No third-party modules required.

  python3 extract_filters.py DolbyHph.dll -o lakehp_filters.h            # 48k + 44.1k
  python3 extract_filters.py DolbyHph.dll --rates 48000 -o filters.h
  python3 extract_filters.py DolbyHph.dll --bins outdir/                 # raw blobs
  python3 extract_filters.py DolbyHph.dll --list

  # Pico 2 W without compiling: add the filters to a prebuilt firmware UF2
  python3 extract_filters.py DolbyHph.dll --uf2 lakehp_pico2w.uf2 --uf2-out lakehp_ready.uf2
  (tools/lakehp_flasher.html does the same in a browser.)

Works by scanning the image for filter blobs and validating each header, so
other builds of the engine with the same blob format should work too.
"""
import argparse, os, struct, sys

RATES = {0: 48000, 1: 44100, 2: 32000, 3: 22050, 4: 16000, 5: 11025, 6: 8000,
         16: 192000, 17: 96000, -1: 0xFFFFFFFF}
ROOM_NAMES = {0: "MIXDOWN", 1: "DH1_SMALL", 2: "DH2_MEDIUM", 3: "DH3_LARGE"}


def s8(b):
    return b - 256 if b > 127 else b


def parse_header(d, off):
    """Mirror of lhp_parse_header(). Returns dict or None."""
    try:
        ver = d[off]
        if ver not in (1, 2):
            return None
        total = struct.unpack_from("<I", d, off + 1)[0]
        if total < 0x40 or total > 0x40000 or off + total > len(d):
            return None
        hlen = struct.unpack_from("<H", d, off + 5)[0]
        if hlen < 0x30 or hlen > 0x200 or hlen >= total:
            return None
        p = off + 7
        h3, h4, room, rc, flags, maxbits = (s8(x) for x in d[p:p + 6]); p += 6
        if rc not in RATES or room < 0 or room > 3:
            return None
        if ver >= 2:
            scale = s8(d[p]); p += 1
        else:
            scale = maxbits
        if not (1 <= maxbits <= 32 and 1 <= scale <= 31):
            return None
        gains = [struct.unpack_from("<h", d, p + 2 * i)[0] / 16384 for i in range(5)]; p += 10
        cuts = list(struct.unpack_from("<12H", d, p)); p += 24
        p += 20                      # 10 x q14 misc
        p += 1                       # hdr37
        nlen = s8(d[p]) + 1; p += 1
        name = d[p:p + nlen].split(b"\0")[0].decode("latin-1", "replace"); p += nlen
        if p - off != hlen:
            return None
        return dict(off=off, version=ver, total=total, hlen=hlen, room=room,
                    rate=RATES[rc], flags=flags & 0xff, maxbits=maxbits,
                    scale=scale, gains=gains, cuts=cuts, name=name)
    except (IndexError, struct.error):
        return None


def load_pe_image(path):
    """Return the raw file bytes plus version string if found."""
    d = open(path, "rb").read()
    if d[:2] != b"MZ":
        sys.exit("not a PE file")
    ver = None
    i = d.find("FileVersion".encode("utf-16le"))
    if i > 0:
        j = d.find(b"\0\0\0", i + 24)
        ver = d[i + 24:j + 1].decode("utf-16le", "ignore").strip("\0 ")
    return d, ver


def scan(d):
    found, off = [], 0
    while off < len(d) - 0x60:
        h = parse_header(d, off)
        if h:
            found.append(h)
            off += h["total"]
        else:
            off += 1
    return found


REGION_ADDR = 0x10300000          # default: Pico 2 W builds without a region tag
REGION_TAG = b"LHP-FREGION\0"    # firmware/common/lhp_filter_region.h
UF2_MAGIC = (0x0A324655, 0x9E5D5157, 0x0AB16F30)
UF2_FAMILY_ABSOLUTE = 0xE48BFF57


def crc32(b):
    import zlib
    return zlib.crc32(b) & 0xFFFFFFFF


def region_label(dll_name, ver, sel):
    """Where the filters came from, shown on the settings page: file name,
    version and the room names (when the DLL has them and they carry no
    brand name). Same rule as region label in flasher_template.html."""
    s = dll_name + (" " + ver if ver else "")
    names = [h["name"].strip() for h in sorted(sel, key=lambda h: h["room"])]
    if names and all(names) and not any("dolby" in x.lower() for x in names):
        s += " \u00b7 " + " / ".join(names)
    s = "".join(c if 32 <= ord(c) < 127 else ("-" if c == "\u00b7" else "?") for c in s)
    return s[:95]


def build_region(d, sel, label=None):
    """Filter region image, see firmware/common/lhp_filter_region.h."""
    n = len(sel)
    off = 20 + 16 * n
    table, blobs = b"", b""
    for h in sel:
        blob = d[h["off"]:h["off"] + h["total"]]
        table += struct.pack("<4I", h["rate"], h["room"], off + len(blobs), len(blob))
        blobs += blob + b"\0" * (-len(blob) % 4)
    body = table + blobs
    if label:                                   # optional source label after the blobs
        t = label.encode("ascii", "replace")[:95]
        body += b"LHPS" + struct.pack("<I", len(t)) + t + b"\0" * (-len(t) % 4)
    total = 20 + len(body)
    return struct.pack("<5I", 0x4650484C, 1, n, total, crc32(body)) + body


def uf2_region_addr(base):
    """Where this firmware reads its filters from: (addr, max) from its tag,
    or the Pico 2 W default for older builds."""
    img = b"".join(base[i + 32:i + 32 + struct.unpack_from("<I", base, i + 16)[0]]
                   for i in range(0, len(base) - 511, 512)
                   if struct.unpack_from("<I", base, i + 28)[0] != UF2_FAMILY_ABSOLUTE)
    k = img.find(REGION_TAG)
    if k < 0 or k + 20 > len(img):
        return REGION_ADDR, 512 * 1024
    return struct.unpack_from("<2I", img, k + 12)


def uf2_add_region(base, region, addr=None):
    """Append the region to a firmware UF2 and renumber the blocks."""
    if len(base) % 512:
        sys.exit("base UF2 has a partial block")
    if addr is None:
        addr, max_len = uf2_region_addr(base)
        if len(region) > max_len:
            sys.exit(f"the filters ({len(region)} bytes) do not fit this firmware's region ({max_len} bytes)")
    blocks = [bytearray(base[i:i + 512]) for i in range(0, len(base), 512)]
    fams = {}
    for b in blocks:
        m0, m1, flags, a, size, bn, nb, fam = struct.unpack_from("<8I", b, 0)
        if (m0, m1) != UF2_MAGIC[:2] or struct.unpack_from("<I", b, 508)[0] != UF2_MAGIC[2]:
            sys.exit("not a UF2 file")
        if fam != UF2_FAMILY_ABSOLUTE:
            fams[fam] = fams.get(fam, 0) + 1
            if a + size > addr and a < addr + len(region):
                sys.exit("firmware overlaps the filter region")
    if not fams:
        sys.exit("UF2 has no image blocks")
    fam = max(fams, key=fams.get)
    region += b"\xff" * (-len(region) % 256)
    for i in range(0, len(region), 256):
        b = bytearray(512)
        struct.pack_into("<8I", b, 0, UF2_MAGIC[0], UF2_MAGIC[1], 0x2000, addr + i, 256, 0, 0, fam)
        b[32:288] = region[i:i + 256]
        struct.pack_into("<I", b, 508, UF2_MAGIC[2])
        blocks.append(b)
    main = [b for b in blocks if struct.unpack_from("<I", b, 28)[0] == fam]
    for i, b in enumerate(main):
        struct.pack_into("<2I", b, 20, i, len(main))
    return b"".join(blocks)


def c_ident(h):
    r = h["rate"]
    rs = "ANY" if r == 0xFFFFFFFF else str(r)
    return f"LHP_FILTER_{ROOM_NAMES[h['room']]}_{rs}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dll")
    ap.add_argument("-o", "--out", help="C header to write")
    ap.add_argument("--rates", default="48000,44100", help="comma list, or 'all'")
    ap.add_argument("--bins", help="directory to write raw .bin blobs")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--region", help="write the flash filter-region image (.bin)")
    ap.add_argument("--uf2", help="prebuilt firmware UF2 to add the filters to")
    ap.add_argument("--uf2-out", help="output UF2 (firmware + filters)")
    ap.add_argument("--addr", type=lambda x: int(x, 0),
                    help="filter region flash address (default: read from the firmware, else 0x10300000)")
    a = ap.parse_args()

    d, ver = load_pe_image(a.dll)
    filters = scan(d)
    if not filters:
        sys.exit("no Lake filter blobs found - is this DolbyHph.dll?")
    print(f"{os.path.basename(a.dll)}: version {ver or '?'}; {len(filters)} filter blobs", file=sys.stderr)
    for h in filters:
        print(f"  @0x{h['off']:06x} len {h['total']:6d}  room {h['room']} "
              f"rate {h['rate'] if h['rate'] != 0xFFFFFFFF else 'any':>6}  flags 0x{h['flags']:02x} "
              f"{h['name']!r}", file=sys.stderr)
    if a.list:
        return

    if (a.region or a.uf2) and a.rates == "48000,44100":
        a.rates = "48000"                     # the Pico firmware only uses 48 kHz
    want = None if a.rates == "all" else {int(x) for x in a.rates.split(",")}
    sel = [h for h in filters if h["room"] in (1, 2, 3) and (want is None or h["rate"] in want)]
    if not sel:
        sys.exit("no room filters for the requested rates")

    if a.bins:
        os.makedirs(a.bins, exist_ok=True)
        for h in sel:
            fn = os.path.join(a.bins, c_ident(h).lower() + ".bin")
            open(fn, "wb").write(d[h["off"]:h["off"] + h["total"]])
            print("wrote", fn, file=sys.stderr)

    if a.region or a.uf2:
        region = build_region(d, sel, region_label(os.path.basename(a.dll), ver, [h for h in sel if h["rate"] == 48000]))
        if a.region:
            open(a.region, "wb").write(region)
            print(f"wrote {a.region} ({len(region)} bytes)", file=sys.stderr)
        if a.uf2:
            out = a.uf2_out or os.path.splitext(a.uf2)[0] + "_with_filters.uf2"
            base = open(a.uf2, "rb").read()
            addr = a.addr if a.addr is not None else uf2_region_addr(base)[0]
            open(out, "wb").write(uf2_add_region(base, region, addr))
            print(f"wrote {out}: firmware + {len(sel)} room filters at 0x{addr:08x}", file=sys.stderr)

    if a.out:
        lines = ["/* AUTO-GENERATED by extract_filters.py from your copy of DolbyHph.dll",
                 f" * (version {ver or '?'}). Contains Dolby/Lake filter data:",
                 " * for personal use only - do not redistribute. */",
                 "#pragma once", "#include <stdint.h>", "#include <stddef.h>", ""]
        tbl = []
        for h in sel:
            ident = c_ident(h)
            blob = d[h["off"]:h["off"] + h["total"]]
            lines.append(f"/* room {h['room']}, {h['rate']} Hz, {len(blob)} bytes */")
            lines.append(f"static const uint8_t {ident}[{len(blob)}] = {{")
            for i in range(0, len(blob), 20):
                lines.append("  " + ",".join(f"0x{b:02x}" for b in blob[i:i + 20]) + ",")
            lines.append("};")
            tbl.append((h, ident, len(blob)))
        lines += ["", "typedef struct { uint32_t rate; uint8_t room; const uint8_t *blob; size_t len; } lhp_filter_entry;",
                  "static const lhp_filter_entry lhp_filters[] = {"]
        for h, ident, n in tbl:
            lines.append(f"  {{ {h['rate']}u, {h['room']}, {ident}, {n} }},")
        src = region_label(os.path.basename(a.dll), ver, [h for h in sel if h["rate"] == 48000])
        src = src.replace("\\", "/").replace('"', "'")
        lines += ["};", "#define LHP_NUM_FILTERS (sizeof(lhp_filters)/sizeof(lhp_filters[0]))",
                  f'#define LHP_FILTER_SOURCE "{src}"', "",
                  "static inline const lhp_filter_entry *lhp_find_filter(uint32_t rate, int room) {",
                  "  for (size_t i = 0; i < LHP_NUM_FILTERS; i++)",
                  "    if (lhp_filters[i].rate == rate && lhp_filters[i].room == room) return &lhp_filters[i];",
                  "  return 0;", "}", ""]
        open(a.out, "w").write("\n".join(lines))
        print(f"wrote {a.out} ({len(sel)} filters)", file=sys.stderr)


if __name__ == "__main__":
    main()
