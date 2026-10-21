/*
 * lhp_filter_region.h - room filters stored in their own flash region.
 *
 * Lets one prebuilt firmware image work for everyone: the room filters come
 * from the user's own copy of the original DLL and are written to a fixed
 * flash address next to the firmware (tools/lakehp_flasher.html builds that
 * UF2 in the browser; tools/extract_filters.py --region does the same).
 *
 * Layout (little-endian), all offsets from the region start:
 *   0  u32 magic 'LHPF'      4  u32 version (1)     8  u32 count
 *  12  u32 total bytes      16  u32 CRC-32 of bytes [20, total)
 *  20  count x { u32 rate, u32 room, u32 offset, u32 length }
 *      blobs (each 4-byte aligned)
 */
#ifndef LHP_FILTER_REGION_H
#define LHP_FILTER_REGION_H
#include <stdint.h>
#include <string.h>
#include "dh_pipeline.h"

#define LHP_REGION_MAGIC   0x4650484Cu      /* "LHPF" */
#define LHP_REGION_VERSION 1u

/* Each firmware says where it reads the region from, so the tools can find
 * it in any build's UF2: the tag string, then u32 address and u32 max size.
 * Firmwares without a tag (older Pico 2 W builds) use 0x10300000. */
#define LHP_REGION_TAG_STR "LHP-FREGION"
typedef struct { char tag[12]; uint32_t addr, max; } lhp_region_tag_t;
#define LHP_REGION_TAG(_addr, _max) \
    __attribute__((used)) const lhp_region_tag_t lhp_region_tag = { LHP_REGION_TAG_STR, (_addr), (_max) }
/* call once from main(): keeps the tag through --gc-sections */
#define LHP_REGION_TAG_KEEP() __asm volatile("" :: "r"(&lhp_region_tag))

static inline uint32_t lhp_region_rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static inline uint32_t lhp_region_crc32(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

/* Fills out[] with the filters found; returns the count, or a negative error:
 * -1 no region, -2 bad size, -3 CRC mismatch, -4 bad table. */
static inline int lhp_region_parse(const uint8_t *base, uint32_t max_size, dhp_filter_t *out, int max_out)
{
    if (lhp_region_rd32(base) != LHP_REGION_MAGIC || lhp_region_rd32(base + 4) != LHP_REGION_VERSION) return -1;
    const uint32_t count = lhp_region_rd32(base + 8), total = lhp_region_rd32(base + 12);
    if (total < 20 || total > max_size || count == 0 || 20 + 16 * count > total) return -2;
    if (lhp_region_crc32(base + 20, total - 20) != lhp_region_rd32(base + 16)) return -3;
    int n = 0;
    for (uint32_t i = 0; i < count && n < max_out; i++) {
        const uint8_t *e = base + 20 + 16 * i;
        const uint32_t off = lhp_region_rd32(e + 8), len = lhp_region_rd32(e + 12);
        if (off < 20 + 16 * count || off + len > total || off + len < off) return -4;
        out[n++] = (dhp_filter_t){ lhp_region_rd32(e), (uint8_t)lhp_region_rd32(e + 4), base + off, len };
    }
    return n;
}
#endif
