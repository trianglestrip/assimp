#include "Scan.h"

#include <cstring>

namespace ap::obj {

LineKind classify(const char* p, const char* end) {
    if (p >= end) return LineKind::Other;
    switch (p[0]) {
    case 'v':
        if (p + 1 < end) {
            if (isSpace(p[1]))        return LineKind::V;
            if (p[1] == 'n' && p + 2 <= end && (p + 2 == end || isSpace(p[2]))) return LineKind::VN;
            if (p[1] == 't' && p + 2 <= end && (p + 2 == end || isSpace(p[2]))) return LineKind::VT;
        }
        return LineKind::Other;
    case 'f':
        if (p + 1 < end && isSpace(p[1])) return LineKind::F;
        return LineKind::Other;
    case 'u':
        if (end - p >= 7 && std::memcmp(p, "usemtl", 6) == 0 && isSpace(p[6]))
            return LineKind::UseMtl;
        return LineKind::Other;
    case 'o':
        if (p + 1 < end && isSpace(p[1])) return LineKind::Object;
        return LineKind::Other;
    case 'g':
        if (p + 1 < end && isSpace(p[1])) return LineKind::Group;
        return LineKind::Other;
    case 'm':
        if (end - p >= 7 && std::memcmp(p, "mtllib", 6) == 0 && isSpace(p[6]))
            return LineKind::Mtllib;
        return LineKind::Other;
    default:
        return LineKind::Other;
    }
}

// number of vertex tokens on an 'f' line (tokens starting with digit/'-'),
// scanned 8 bytes per iteration (SWAR). Each word is turned into per-byte
// flag masks (flag = top bit 0x80 of each byte): sepHigh marks separators
// (' ' / '\t' / '\r'), startHigh marks the first byte of each maximal
// non-space run (non-sep byte whose predecessor was a separator; the run
// state carried across word boundaries is prevByteNonSep, false at p so a
// token starting at p itself counts). A run start is counted when its byte
// is an ASCII digit or '-'. All per-byte additions stay <= 0xFE, so no
// carries bleed between lanes and every mask is exact per byte; bytes with
// the high bit set are never digits (matching signed-char tests).
uint32_t countFaceVerts(const char* p, const char* end) {
    constexpr uint64_t kOnes  = 0x0101010101010101ull; // 0x01 per byte
    constexpr uint64_t kHighs = 0x8080808080808080ull; // 0x80 per byte
    constexpr uint64_t kLow7  = 0x7F7F7F7F7F7F7F7Full; // 0x7F per byte
    const uint64_t kSp    = uint64_t(' ') * kOnes;
    const uint64_t kTab   = uint64_t('\t') * kOnes;
    const uint64_t kCr    = uint64_t('\r') * kOnes;
    const uint64_t kMinus = uint64_t('-') * kOnes;

    // exact per-byte equality: 0x80 flag in every byte of w equal to c
    auto eqHigh = [&](uint64_t w, uint64_t c) {
        const uint64_t t = w ^ c;
        return ~(((t & kLow7) + kLow7) | t) & kHighs;
    };

    uint32_t n = 0;
    bool prevByteNonSep = false; // byte before p: none, so acts as separator
    while (end - p >= 8) {
        uint64_t w;
        std::memcpy(&w, p, sizeof w); // unaligned-safe load; p+8 <= end
        const uint64_t sepHigh = eqHigh(w, kSp) | eqHigh(w, kTab) | eqHigh(w, kCr);
        // digit-or-'-': 7-bit range test for 0x30..0x39 (b + 0x50 flags
        // b >= 0x30, b + 0x46 flags b >= 0x3A) plus exact '-' equality;
        // & ~w stops high-bit bytes from aliasing onto the digit range
        const uint64_t b7 = w & kLow7;
        const uint64_t dmHigh = (((b7 + 0x50 * kOnes) & ~(b7 + 0x46 * kOnes) & ~w & kHighs)
                                 | eqHigh(w, kMinus));
        const uint64_t nonSepHigh = ~sepHigh & kHighs;
        // token START: a non-sep byte whose PREDECESSOR is a separator.
        // Within a word the predecessor flag comes from << 8 (byte k-1);
        // byte 0's predecessor is the previous word's last byte. (The
        // shift direction matters: >> 8 would mark run ENDS, which
        // silently inflated the triangle counts.)
        uint64_t prevHigh = nonSepHigh << 8;
        if (prevByteNonSep) prevHigh |= 0x80ull;   // block byte 0 start
        const uint64_t startHigh = nonSepHigh & ~prevHigh;
        prevByteNonSep = (nonSepHigh >> 56) != 0;  // this word's last byte
        // one 0x80 hit bit per counted byte: >> 3 makes lanes 0x10, the
        // multiply by kOnes sums them into the top byte carry-free, and
        // >> 60 extracts the count (<= 8 per word)
        n += uint32_t((((startHigh & dmHigh) >> 3) * kOnes) >> 60);
        p += 8;
    }
    for (; p < end; ++p) { // tail: fewer than 8 bytes, byte-at-a-time
        const unsigned char c = (unsigned char)*p;
        const bool sep = (c == ' ' || c == '\t' || c == '\r');
        if (!sep) {
            if (!prevByteNonSep && ((c - '0') <= 9u || c == '-')) ++n;
            prevByteNonSep = true;
        } else {
            prevByteNonSep = false;
        }
    }
    return n;
}

} // namespace ap::obj
