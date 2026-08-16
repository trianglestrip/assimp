#pragma once
// ============================================================
// Misc - format-agnostic utilities shared by ModelParser
// implementations: text tokenization over a memory mapping,
// line-aligned chunk splitting for parallel scans, timing.
// Everything here is header-only inline and allocation-light.
// ============================================================

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "FastParse.h"               // ap::fast number kernels

namespace ap {

using Clock = std::chrono::steady_clock;

inline uint64_t microsSince(Clock::time_point t0) {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            Clock::now() - t0).count());
}

// whitespace per common text formats (' ' / '\t' / '\r'; '\n' is the
// line terminator callers stop at, never a separator)
inline bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r'; }

// Advance *p to the start of the next non-empty token; returns nullptr
// at range end. Token start/len written to tok/tokLen.
inline const char* nextToken(const char* p, const char* end,
                             const char*& tok, size_t& tokLen) {
    while (p < end && isSpace(*p)) ++p;
    if (p >= end) return nullptr;
    tok = p;
    while (p < end && !isSpace(*p)) ++p;
    tokLen = size_t(p - tok);
    return p;
}

// Parse helpers: fast kernels with std::from_chars fallback; return 0
// when nothing parses (the established parser convention).
inline float parseFloat(const char* s, size_t n) {
    return ap::fast::parseFloat(s, n);
}
inline int64_t parseInt(const char* s, size_t n) {
    return ap::fast::parseInt(s, n);
}

// Split [0, size) into `n` byte ranges, each boundary snapped forward
// to the next '\n' so a range never starts or ends mid-line (a partial
// line at a boundary belongs to the earlier chunk). This is the unit
// of work for chunk-parallel line scans over an mmap.
inline std::vector<std::pair<size_t, size_t>> splitLineRanges(
    const char* data, size_t size, size_t n) {
    std::vector<std::pair<size_t, size_t>> out;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        size_t b = size * i / n;
        size_t e = size * (i + 1) / n;
        if (b > size) b = size;
        // snap begin forward to the next line start
        while (b > 0 && b < size && data[b - 1] != '\n') ++b;
        // snap end forward to include the partial line
        while (e < size && e > 0 && data[e - 1] != '\n') ++e;
        if (e < b) e = b;
        out.emplace_back(b, e);
    }
    return out;
}

} // namespace ap
