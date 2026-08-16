#pragma once
// FastParse.h
//
// Fast paths for the numeric token shapes that dominate OBJ data
// ("16937", "-0.0359734", "0.688357", "1e-3", "1.", ".5", ...).
// Tokens are guaranteed to be non-empty maximal runs of non-space bytes
// with no leading whitespace. Anything that does not match a fast shape
// is handed to std::from_chars, and a from_chars failure yields 0 --
// the exact semantics of the previous wrappers in ObjParser.cpp
// (result pointer ignored, pre-initialized value returned on error).
//
// Header-only, no exceptions, no allocations, no SIMD intrinsics.
// MSVC /std:c++20 /W4 clean.

#include <charconv>
#include <cstddef>
#include <cstdint>

namespace ap::fast {
namespace detail {

// --- Fallbacks: byte-for-byte the semantics of the old wrappers. ---

inline float parseFloatFallback(const char* s, std::size_t n) noexcept
{
    float v = 0.0f; // from_chars leaves v unmodified on failure -> 0
    const std::from_chars_result r = std::from_chars(s, s + n, v);
    return (r.ec == std::errc()) ? v : 0.0f;
}

inline std::int64_t parseIntFallback(const char* s, std::size_t n) noexcept
{
    std::int64_t v = 0; // from_chars leaves v unmodified on failure -> 0
    const std::from_chars_result r = std::from_chars(s, s + n, v);
    return (r.ec == std::errc()) ? v : 0;
}

// 10^k for k = 0..22, each EXACTLY representable as a double:
// 10^22 = 5^22 * 2^22 with 5^22 = 2'384'185'791'015'625 < 2^53
// (10^23 would need 5^23 > 2^53 and is not exact). Negative decimal
// exponents are applied by DIVIDING by the exact positive power: 10^-k
// itself is generally not exactly representable (0.1 is not), but a
// single correctly rounded division keeps the error at ~1 ulp of
// double, far below float precision.
inline constexpr double kPow10[23] = {
    1e0,  1e1,  1e2,  1e3,  1e4,  1e5,  1e6,  1e7,
    1e8,  1e9,  1e10, 1e11, 1e12, 1e13, 1e14, 1e15,
    1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22
};

} // namespace detail

// ---------------------------------------------------------------------------
// parseInt: [+-]? followed by 1..18 decimal digits and nothing else.
// 18 digits max out at 999'999'999'999'999'999 < 2^63, so the accumulate
// loop cannot overflow; that digit-count check is the overflow guard.
// Anything else -- empty/sign-only token, 19+ digits (possibly outside
// the int64 range), any non-digit byte -- falls back to std::from_chars
// (int64), whose failure yields 0. Note the fast path accepts a leading
// '+' per the token grammar; from_chars itself would reject it, but OBJ
// data never carries '+' in practice.
// ---------------------------------------------------------------------------
inline std::int64_t parseInt(const char* s, std::size_t n) noexcept
{
    std::size_t i = 0;
    bool neg = false;
    if (n > 0 && (s[0] == '+' || s[0] == '-')) {
        neg = (s[0] == '-');
        i = 1;
    }

    const std::size_t digits = n - i; // 0 when the token is empty or sign-only
    if (digits == 0 || digits > 18u) {
        return detail::parseIntFallback(s, n);
    }

    std::uint64_t m = 0;
    for (std::size_t k = i; k < n; ++k) {
        const unsigned d =
            static_cast<unsigned>(static_cast<unsigned char>(s[k])) - static_cast<unsigned>('0');
        if (d > 9u) {
            return detail::parseIntFallback(s, n); // non-digit where a digit was required
        }
        m = m * 10u + d;
    }

    const std::int64_t v = static_cast<std::int64_t>(m); // m < 2^63, safe
    return neg ? -v : v;
}

// ---------------------------------------------------------------------------
// parseFloat
//
// Grammar: [+-]? ( digits [ '.' digits? ] | '.' digits ) [ (e|E) [+-] digits ]
// ("1", "1.", ".5", "0.688357", "-1.2e-3", "1e3" are all fast shapes).
//
// Precision rule: a mantissa digit is accumulated only while
// mant * 10 + d <= 2^53 - 1 (the strict form of keeping mant <=
// (2^53 - 1) / 10, which alone does not bound the final '+ d'), so the
// later (double)mantissa conversion stays EXACT. Combined with an exact
// power of ten -- one multiply for e in [0, 22], one correctly rounded
// division by an exact 10^-e for e in [-22, -1] -- the result is
// correctly rounded except for truncation of digits beyond the 15-16
// significant decimals kept in the mantissa, an error below ~1e-16
// relative, well within float tolerance. The mantissa therefore cannot
// overflow: digits past the 2^53 - 1 budget are dropped, never
// accumulated.
//
// Decimal-exponent bookkeeping (mantissa = accepted digit string):
//   - every ACCEPTED fraction digit scales the value by 1/10,
//   - every integer digit dropped after the mantissa filled up scales
//     the value by 10 (dropped fraction digits sit at the tail and are
//     pure truncation -- they shift nothing),
//   - plus the optional eE exponent.
// Counters are 64-bit so pathological token lengths cannot overflow
// them; any total outside [-22, 22] falls back anyway.
//
// Fallback to std::from_chars(double->float) when the token is empty or
// contains no digit at all ("", "+", ".", "-.", "inf", "nan", hex
// tokens), has a malformed exponent ("1e", "1e+", more than 9 exponent
// digits), leaves trailing bytes outside the grammar, or needs a total
// decimal exponent outside [-22, 22]. from_chars failure returns 0,
// matching the previous wrapper.
// ---------------------------------------------------------------------------
inline float parseFloat(const char* s, std::size_t n) noexcept
{
    constexpr std::uint64_t kMaxExactMant = (1ull << 53) - 1u; // 9'007'199'254'740'991

    std::size_t i = 0;
    bool neg = false;
    if (n > 0 && (s[0] == '+' || s[0] == '-')) {
        neg = (s[0] == '-');
        i = 1;
    }

    std::uint64_t mant = 0;
    std::int64_t exp10 = 0;
    bool anyDigits = false;

    std::size_t k = i;
    while (k < n) { // integer part
        const unsigned d =
            static_cast<unsigned>(static_cast<unsigned char>(s[k])) - static_cast<unsigned>('0');
        if (d > 9u) {
            break;
        }
        anyDigits = true;
        const std::uint64_t m2 = mant * 10u + d; // stays < 2^64: mant <= 2^53 - 1
        if (m2 <= kMaxExactMant) {
            mant = m2;
        } else {
            ++exp10; // dropped integer digit: value scales by 10
        }
        ++k;
    }

    if (k < n && s[k] == '.') { // fraction part
        ++k;
        while (k < n) {
            const unsigned d =
                static_cast<unsigned>(static_cast<unsigned char>(s[k])) - static_cast<unsigned>('0');
            if (d > 9u) {
                break;
            }
            anyDigits = true;
            const std::uint64_t m2 = mant * 10u + d;
            if (m2 <= kMaxExactMant) {
                mant = m2;
                --exp10; // accepted fraction digit: value scales by 1/10
            }
            // Dropped fraction digits are tail truncation; no exponent shift.
            ++k;
        }
    }

    if (!anyDigits) {
        return detail::parseFloatFallback(s, n);
    }

    std::int64_t ee = 0;
    if (k < n && (s[k] == 'e' || s[k] == 'E')) {
        ++k;
        bool eneg = false;
        if (k < n && (s[k] == '+' || s[k] == '-')) {
            eneg = (s[k] == '-');
            ++k;
        }
        const std::size_t estart = k;
        while (k < n) {
            const unsigned d =
                static_cast<unsigned>(static_cast<unsigned char>(s[k])) - static_cast<unsigned>('0');
            if (d > 9u) {
                break;
            }
            if (k - estart < 9u) {
                ee = ee * 10 + static_cast<std::int64_t>(d); // <= 9 digits kept: no overflow
            }
            ++k;
        }
        const std::size_t edigits = k - estart;
        if (edigits == 0 || edigits > 9u) {
            return detail::parseFloatFallback(s, n); // "1e", "1e+", runaway exponent
        }
        if (eneg) {
            ee = -ee;
        }
    }

    if (k != n) {
        return detail::parseFloatFallback(s, n); // trailing bytes outside the grammar
    }

    const std::int64_t eTot = exp10 + ee;
    if (eTot < -22 || eTot > 22) {
        return detail::parseFloatFallback(s, n); // outside the exact-power table
    }

    const std::size_t ae = static_cast<std::size_t>(eTot < 0 ? -eTot : eTot); // 0..22
    const double pow10 = detail::kPow10[ae];
    const double mag = (eTot < 0) ? (static_cast<double>(mant) / pow10)
                                  : (static_cast<double>(mant) * pow10);
    return static_cast<float>(neg ? -mag : mag);
}

} // namespace ap::fast
