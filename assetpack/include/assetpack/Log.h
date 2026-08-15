#pragma once
// ============================================================
// Log - minimal thread-safe stage logger
//
// Every line: [ms-since-start][thread][stage] message.
// Used to trace parse-graph stage events in real time.
// ============================================================

#include <cstdint>
#include <string_view>

namespace ap {

enum class LogLevel : uint8_t { Info, Warn, Error };

// Install/reset the log sink (default: stdout, line-buffered).
// The sink receives fully formatted lines.
void setLogSink(void (*sink)(std::string_view line));

void logLine(LogLevel level, std::string_view stage, std::string_view msg);

// milliseconds since first log call (process-anchored)
uint64_t logClockMs();

} // namespace ag

// printf-style logging: AP_LOG("import", "loaded %d meshes", n)
#include <cstdio>
#define AP_LOG(stage, ...) do { \
    char _buf[512]; \
    std::snprintf(_buf, sizeof(_buf), __VA_ARGS__); \
    ::ap::logLine(::ap::LogLevel::Info, stage, _buf); \
} while (0)

#define AP_LOG_WARN(stage, ...) do { \
    char _buf[512]; \
    std::snprintf(_buf, sizeof(_buf), __VA_ARGS__); \
    ::ap::logLine(::ap::LogLevel::Warn, stage, _buf); \
} while (0)
