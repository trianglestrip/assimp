#include <assetpack/AssetPack.h>

#include <chrono>
#include <cstdio>
#include <mutex>

namespace ap {

namespace {
std::mutex g_mu;
void (*g_sink)(std::string_view) = nullptr;

uint64_t nowMs() {
    using Clock = std::chrono::steady_clock;
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now().time_since_epoch()).count());
}

uint64_t startMs() {
    static uint64_t t0 = nowMs();
    return t0;
}
} // namespace

void setLogSink(void (*sink)(std::string_view line)) {
    std::lock_guard lock(g_mu);
    g_sink = sink;
}

uint64_t logClockMs() { return nowMs() - startMs(); }

void logLine(LogLevel level, std::string_view stage, std::string_view msg) {
    char line[640];
    const char* lvl = level == LogLevel::Info ? "INFO"
                    : level == LogLevel::Warn ? "WARN" : "ERR ";
    std::snprintf(line, sizeof(line), "[%8.3fs][%-5s] %.*s: %.*s\n",
                  logClockMs() / 1000.0, lvl,
                  int(stage.size()), stage.data(),
                  int(msg.size()), msg.data());
    std::lock_guard lock(g_mu);
    if (g_sink) g_sink(line);
    else { std::fputs(line, stdout); std::fflush(stdout); }
}

} // namespace ag
