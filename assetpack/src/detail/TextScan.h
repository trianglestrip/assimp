#pragma once
// ============================================================
// TextScan - format-agnostic text utilities shared by the ModelParser
// implementations: tokenization over a memory mapping and the fast
// number kernels. Header-only inline because the OBJ face walker
// (forEachFaceVertex in Scan.h) is a template that inlines them.
// ============================================================

#include <chrono>
#include <cstddef>
#include <cstdint>

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

} // namespace ap
