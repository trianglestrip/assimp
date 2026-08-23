#pragma once
#include <string>
#include <cstdint>

namespace ap {

struct Config {
    bool cacheImage = false;
    std::string cacheDir = "cache/textures";

    static Config& instance();
    bool loadFromFile(const std::string& path = "config.json");
    bool saveToFile(const std::string& path = "config.json") const;
};

}
